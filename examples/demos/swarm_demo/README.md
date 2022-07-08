# Swarm wih dynamic Trajectories demo application

This folder contains both an app layer application for all the Crazyflies that are used at the demo and a control_tower python script that supervises the whole process.It also generates the paths for the and generates the trajectories for the Crazyflies.

## <u>Build</u>

### Crazyflie App Layer-Firmware
this is underlined text in HTML or markdown, which accepts HTML
Make sure that you are in the demo folder (not the main folder of the crazyflie firmware). Then type the following to build the application:

```
make clean
make
```
In order to flash the application to each Crazyflie, you can use the following command:

```
make cload
```

or run the ```cload-all.sh``` script which automatically uploads the firmware to all of them . 

### Control Tower
This demo also uses an optimization engine solver for solving the path planning problem. This  [OpEn](https://alphaville.github.io/optimization-engine/) is a  fast solver that is used to generate the paths for all the Crazyflies.In order to install it you need to follow the installation instructions [here](https://alphaville.github.io/optimization-engine/docs/installation) but only the **OpEn Requirements: Rust and clang** and **Python Interface** steps are needed.

Then you will have to go to the following directory  

```
cd examples/demos/swarm_demo/control_tower/multi_mav_planning
```
and run the ```  solver_creation_multiple.py ``` script.


## <u>Running the demo</u>
The demo is started by running the control_tower.py script

## <u>Description</u>
In this demo a swarm of Crazyflies is created and controlled by a control tower.
As soon as the desired number of flying copters have been taken off,random points of a rectangular area are chosen for each copter.A optimization problem is solved to find the best way to reach the chosen point while maintaining a collision free status .Both between the copters and static obstacles(in this code there is an obstacle modeled as a cylinder).

As soon the desired waypoints have been created from the solver a minimum snap trajectory is generated through them and sent to the copters.

Then each Crazyflie using the high level commander ,follows the generated trajectory.

While flying the next trajectories are calculated,generated and uploaded.As a result there is a smooth transition between the trajectories and constant motion of the Crazyflies.

Finally,after a certain amount of trajectories a trajectory that leads back to the charging station is generated and sent to each drone.

After that the copters are brought back to the charging station and the swarm is reset.




![Alt text](figures/planning.png?raw=true "Title")