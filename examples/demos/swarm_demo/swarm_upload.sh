#!/usr/bin/env bash

cd /home/oem/MARIOS/crazyflie-firmware-experimental/examples/demos/swarm_demo

CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E702" make cload
CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E704" make cload
CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E706" make cload
CLOAD_CMDS="-w radio://0/20/2M/E7E7E7E707" make cload