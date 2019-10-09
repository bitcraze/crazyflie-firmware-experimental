#!/usr/bin/env python3
#
# Rotate geometry data for base stations
#


import sys
import math
import numpy as np


# Add your base station information from the get_bs_position.py script
lighthouseBaseStationsGeometry  = [
    [[0.021855, 2.318295, 1.666880, ], [[0.999998, -0.001797, 0.000000, ], [0.001451, 0.807746, 0.589529, ], [-0.001059, -0.589528, 0.807747, ], ]],
    [[-0.176309, 2.290894, -1.768068, ], [[-0.995269, 0.091770, -0.031898, ], [0.053428, 0.791198, 0.609222, ], [0.081146, 0.604635, -0.792358, ], ]],
]

# Rotation angle in degrees
angle = math.radians(-45 + 180.0)

# Rotation matrix around Y-axis
c = math.cos(angle)
s = math.sin(angle)
r = np.array([
    [c, 0, s],
    [0, 1, 0],
    [-s, 0, c],
])


for pose in lighthouseBaseStationsGeometry:
    position = np.dot(r, pose[0])
    rotation = np.dot(r, pose[1])

    print("{.origin = {", end='')
    for i in position:
        print("{:0.6f}, ".format(i), end='')

    print("}, .mat = {", end='')

    for i in rotation:
        print("{", end='')
        for j in i:
            print("{:0.6f}, ".format(j), end='')
        print("}, ", end='')

    print("}},")
