# Hyper demo 2020

This folder contains the 2020 hyper demo. It demonstrates a swarm of Crazyflies flying through the Bitcraze office
using a combination of lighthouse, flow and LPS positioning.

## Build

Make sure that you are in the hyperdemo folder (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make
make cload
```

If you want to compile the application elsewhere in your machine, make sure to update ```CRAZYFLIE_BASE``` in the **Makefile**.

## Start script

There is a python script called hyper.py that starts the demo.

## Trajectory

The trajectory is hardcoded to fit the Bitcraze office

## Firmware modifications

1. The number of lighthouse base stations has been increased to work with more than 2 lighthouse 2 base stations.
1. The maximum speed of the PID controller has been increased to make it more exciting
