import math

steps = 60
r = 0.4
for i in range(0, steps):
    a = 3.14 * 2 * i / steps
    x = r * math.cos(a)
    y = r * math.sin(a)
    print('{x: XB + ' + str(x) + ', y: YB + ' + str(y) + ', z: ZB + 1.5},')
