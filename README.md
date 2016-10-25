# CS704 Group4 SW
Authors: Elliot Sayes, Marlo Hermansson-Scott, James Tizard

## Overview

The application is built using a [makefile](Makefile), which also provides shortcuts for common functions.
Application components are located in [app](/app), processor support is in [stm32l1](/stm32l1).
Modules and drivers are included as git submodules in [modules](/modules).

Files authored by us include:
 - trilaterate.(c/h)
 - imu.(c/h)
 - kalman.(c/h)
 - main.c (partial)

Misc Python scripts used in the development of trilateration in [py](/py) folder.

Visualisation program in [visualisation](/visualisation) folder.

## Testing guide

Testing hardware requirements:
 - Base station
 - Mobile node
 - Active beacons

Testing Proceedure:
1. Connect antennae to both base station and mobile node.
2. In the `main.c` file, ensure the `BASE_STATION` defiition macro is commented on the first line
3. Make the project using `make`
4. Connect the mobile node via USB and put it into dfu mode by holding the 'prog' button and tapping 'reset'
5. Program the mobile node using `make dfu`
6. Uncomment the `BASE_STATION` defiition macro on the first line of `main.c`
7. Repeat steps 3-5 for the Base Station board
8. Load up the Java visualisation
9. Hit the reset button on both boards

## Build Requirements

 - make
 - gcc-arm-none-eabi
 - texane/stlink
 - dfu-util

## Usage

 - `make` to build
 - `make dfu` to flash via DFU
 - `make flash` to flash via stlink
 - `make clean` to clean
 - `make ds` to launch debug server
 - `make d` to launch gdb session
