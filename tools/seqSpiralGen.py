import math

steps_per_rev = 60
steps_up = 30
revs = 3
r = 0.4
top = 2.0
bottom = 1.0


def add_line(x, y, z):
    print('{x: XB + ' + str(x) + ', y: YB + ' + str(y) + ', z: ZB + ' + str(z) + '},')


# Going up
for i in range(0, steps_up):
    z = bottom + (top - bottom) * i / steps_up
    add_line(0, 0, z)

# Spiral down
total_steps = steps_per_rev * revs
for i in range(0, total_steps):
    a = 3.14 * 2 * i / steps_per_rev
    x = r * math.cos(a)
    y = r * math.sin(a)
    z = top - (top - bottom) * i / total_steps
    add_line(x, y, z)
