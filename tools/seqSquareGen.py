import math


def add_line(x, y, z):
    print('{x: XB + ' + str(x) + ', y: YB + ' + str(y) + ', z: ZB + ' + str(z) + '},')

r = 0.4
steps_side = 20

for i in range(-steps_side / 2, steps_side / 2):
    x = r * i / (steps_side / 2)
    y = r
    print('{x: XB + ' + str(x) + ', y: YB + ' + str(y) + ', z: ZB + 1.5},')

for i in range(-steps_side / 2, steps_side / 2):
    x = r
    y = r * (-i) / (steps_side / 2)
    print('{x: XB + ' + str(x) + ', y: YB + ' + str(y) + ', z: ZB + 1.5},')

for i in range(-steps_side / 2, steps_side / 2):
    x = r * (-i) / (steps_side / 2)
    y = -r
    print('{x: XB + ' + str(x) + ', y: YB + ' + str(y) + ', z: ZB + 1.5},')

for i in range(-steps_side / 2, steps_side / 2):
    x = -r
    y = r * i / (steps_side / 2)
    print('{x: XB + ' + str(x) + ', y: YB + ' + str(y) + ', z: ZB + 1.5},')
