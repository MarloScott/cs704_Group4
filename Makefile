# CS704 / AWSAM5 Makefile
# Ryan Kurte, Zac Roberts 2016

TARGET = cs704
BUILD_DIR = build
FLASH_START=0x08000000

DBGFLAGS=-g -gdwarf-2 -O0 -DPLATFORM_SLEEP_MS=delay_ms -DPLATFORM_SLEEP_US=delay_us

# Tools

TOOLCHAIN=arm-none-eabi
CC  = ${TOOLCHAIN}-gcc
CXX = ${TOOLCHAIN}-g++
GDB = ${TOOLCHAIN}-gdb
AR  = ${TOOLCHAIN}-ar
NM  = ${TOOLCHAIN}-nm
OBJCOPY = ${TOOLCHAIN}-objcopy
OBJDUMP = ${TOOLCHAIN}-objdump
OBJSIZE = ${TOOLCHAIN}-size
DFU = dfu-util
FLASH = st-flash
GDBSERV = st-util

# Output targets

TARGET_ELF = ${BUILD_DIR}/${TARGET}.elf
TARGET_BIN = ${BUILD_DIR}/${TARGET}.bin
TARGET_HEX = ${BUILD_DIR}/${TARGET}.hex
TARGET_MAP = ${BUILD_DIR}/${TARGET}.map
TARGET_DMP = ${BUILD_DIR}/${TARGET}.dmp

all: dir version ${BUILD_DIR} ${TARGET_BIN} ${TARGET_HEX} ${TARGET_DMP} size

# Application files

APP_BASE = app
APP_INC = -I${APP_BASE}/include -I${BUILD_DIR}

APP_LIB = ${BUILD_DIR}/lib${APP_BASE}.a
APP_SRC := $(wildcard $(APP_BASE)/source/*.c)
APP_HDR := $(wildcard $(APP_BASE)/include/*.h)
APP_OBJ := $(patsubst $(APP_BASE)/source/%.c, ${BUILD_DIR}/%.o, ${APP_SRC})
APP_DEPS := $(patsubst %.o, %.d, ${APP_OBJ})
-include ${APP_DEPS}

# STM32 Device Files
STM32_BASE = stm32l1
include stm32l1/stm32l1.mk

# MPU9250 library
MPU9250_BASE = modules/libmpu9250
include modules/libmpu9250.mk

AT86RF212_BASE = modules/libat86rf212
include modules/libat86rf212.mk

# Toolchain configuration
ARM_FLAGS = -DUSE_STDPERIPH_DRIVER=1 -DUSE_STM32L152_EVAL=1 -DSTM32L1XX_MDP=1 -DSTM32L1XX_MD_PLUS=1 -mcpu=cortex-m3 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections
CFLAGS    = -std=gnu11 -Wall -Wpedantic ${DBGFLAGS} -lm ${IFLAGS} ${ARM_FLAGS} --specs=nano.specs -MMD
LDFLAGS   = -T${STM32_LD} -Wl,-Map=${TARGET_MAP} -Xlinker --gc-sections
IFLAGS    = ${APP_INC} ${STM32_INC} ${MPU9250_INC} ${AT86RF212_INC}

# Output configuration

${TARGET_BIN}: ${TARGET_ELF}
${TARGET_HEX}: ${TARGET_ELF}
${TARGET_DMP}: ${TARGET_ELF}

# Generic mappings

${BUILD_DIR}/%.bin: ${BUILD_DIR}/%.elf
	@${OBJCOPY} -O binary $< $@

${BUILD_DIR}/%.hex: ${BUILD_DIR}/%.elf
	@${OBJCOPY} -O ihex $< $@

${BUILD_DIR}/%.dmp: ${BUILD_DIR}/%.elf
	@${OBJDUMP} -d -S $< > $@

# Source to Object Mappings

${BUILD_DIR}/%.o: ${APP_BASE}/source/%.c
	@echo "[application] Building: $<"
	@${CC} ${CFLAGS} -c $< -o $@

${APP_LIB}: ${APP_OBJ}
	@echo "[application] Linking app library"
	@${AR} qc $@ $^

dir:
	@mkdir -p ${BUILD_DIR}

version: dir
	@./version.py CS704 ${BUILD_DIR}/cs704_version.h

# Application Linking

${TARGET_ELF}: ${APP_OBJ} ${STM32_LIB} ${MPU9250_LIB} ${AT86RF212_LIB}
	@echo "Linking Application"
	@${CC} ${CFLAGS} ${LDFLAGS} -o ${BUILD_DIR}/${TARGET}.elf $^ -lm -lc -lgcc -lnosys

size: ${TARGET_ELF}
	${OBJSIZE} $<

${BUILD_DIR}:
	@mkdir -p ${BUILD_DIR}

# Device programming

dfu: ${TARGET_BIN}
	@echo "Writing target via DFU"
	${DFU} -v -d , -a 0 -s ${FLASH_START} -D $<

flash: ${TARGET_BIN}
	@echo "Writing to target flash"
	${FLASH} --reset write $< ${FLASH_START}

erase:
	@echo "Erasing target flash"
	${FLASH} erase

debug-server: ${TARGET_ELF}
	@echo "Starting debug server"
	${GDBSERV} --listen_port 2331

debug:	${TARGET_ELF}
	@echo "Launching debugger"
	${GDB} -tui -command remote.gdbconf $<

f: flash
d: debug
ds: debug-server

#### MISC #####
.PHONY: clean size dir version

clean:
	rm -rf ${BUILD_DIR}/*
