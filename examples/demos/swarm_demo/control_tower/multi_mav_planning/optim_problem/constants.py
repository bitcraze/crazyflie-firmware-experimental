ts = 0.05
N = 50
N_MAV = 4
(nu, nx) = (3, 3)  # control and state dimensions
# Costs Constants
(qX, qU, qN) = (500, 0.1, 89000)
(qObs, qBet) = (200, 1000000)

# Obstacles Constants
(cyl_x, cyl_y, cyl_z, cyl_r, cyl_h) = (0.5, 0.5, 0.5, 0.2, 1.0)
SWARM_COLLISION_DISTANCE = 0.6
# Distance between MAVs
# (dMAV2D, dMAVZ) = (0.5, 0.2) (not used for now)
