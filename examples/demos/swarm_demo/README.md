# Demo application

This folder contains an app layer application for the Crazyflie that is used as a demo.

## Build

Make sure that you are in the demo folder (not the main folder of the crazyflie firmware). Then type the following to build and flash it while the crazyflie is put into bootloader mode:

```
make clean
make
make cload
```

## Running the demo

The demo is started by running the control_tower.py script

## Documentation
In this demo a swarm of Crazyflies is created and controlled by a control tower.
As soon as the desired number of flying copters have been taken off,random points of a rectangular area are chosen for each copter.A optimization problem is solved to find the best way to reach the chosen point while maintaining a collision free status.Both between the copters and static obstacles.

As soon the desired waypoints have been created from the solver a minimum snap trajectory is generated through them and sent to the copters.

Then each Crazyflie using the high level commander ,follows the generated trajectory.

While flying the next trajectories are calculated,generated and uploaded.As a result there is a smooth transition between the trajectories and constant motion of the Crazyflies.

Finally,after a certain amount of trajectories a trajectory that leads back to the charging station is generated and sent to each drone.

After that the copters are brought back to the charging station and the swarm is reset.



![Alt text](figures/planning.png?raw=true "Title")