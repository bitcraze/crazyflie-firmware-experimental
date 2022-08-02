<!-- Title -->
# Useful Data

Each term and the duration is encoded as standard IEEE single-precision floats, meaning that a single segment requires 132 bytes: 8x4 floats for the X, Y, Z and yaw components per trajectory segment, plus one additional float per segment to store its duration. Given that the default size of the trajectory memory is 4 Kbytes, **you can only store 31 segments.**
