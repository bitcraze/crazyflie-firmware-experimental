ts = 0.05
N = 50
N_MAV = 2
(nu, nx) = (3, 3)  # control and state dimensions
# Costs Constants
(qX, qU, qN) = (500, 0.1, 89000)
(qObs, qBet) = (200, 1000000)

# Obsatcles Constants
(cyl_x, cyl_y, cyl_z, cyl_r, cyl_h) = (0.5, 0.5, 0.5, 0.2, 1.0)

# Disatnce between MAVs
(dMAV2D, dMAVZ) = (0.5, 0.2)
