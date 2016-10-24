import math

x = input("x: ")
y = input("y: ")

X_S = 0;
Y_S = 0;

beacon_locations =[
    [X_S+10600, Y_S+1400      ],
    [X_S+0,     Y_S+2500      ],
    [X_S+0,     Y_S+2500+12300],
    [X_S+10600, Y_S+11400     ]
]

d = []

for beacon in beacon_locations:
    d.append( math.sqrt((x-beacon[0])**2+(y-beacon[1])**2) )

print(d)
