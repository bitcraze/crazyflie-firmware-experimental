# Swarm wih dynamic Trajectories demo application

This folder contains both an app layer application for all the Crazyflies that are used at the demo and a control_tower python script that supervises the whole process.It also generates the paths for the and generates the trajectories for the Crazyflies.

## <u>Building the App</u>

### Crazyflie App Layer-Firmware
Before buildind the app you need to clean the build space and get the default configuration.

```
make clean
make cf2_defconfig
```

### 2 Lighthouse base stations
Make sure that you are in the demo folder (not the main folder of the crazyflie firmware). Then type the following to build the application:

```
make
```

### More than 2 Lighthouse base stations
If you want to run the demo with more than 2 lighthouse base stations, you need to build the application through the menuconfig option running the following command:

```
make clean
make cf2_defconfig
make menuconfig
```
Change  the option ```"Expansion Deck Configuration-->Max Number of Base stations"``` to the number you want to and select ```"App Layer COnfiguration-->Enable app entry point"``` while  following the instructions for the [app layer entry-point](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/).
After saving the kbuild configuration, it is also needed to add this line in the Kbuild file ```src/Kbuild```
```
obj-y += ../examples/demos/swarm_demo/
```

Finally,type the following to build the application:

```
make 
```

## <u>Flashing</u>
In order to flash the application to each Crazyflie, you can use the following command:

```
make cload
```

or run the ```cload-all.sh``` script which automatically uploads the firmware to all of them . 



## Control Tower 
This demo also uses an optimization engine solver for solving the path planning problem. This  [OpEn](https://alphaville.github.io/optimization-engine/) is a  fast solver that is used to generate the paths for all the Crazyflies.In order to install it you need to follow the installation instructions [here](https://alphaville.github.io/optimization-engine/docs/installation) but only the **OpEn Requirements: Rust and clang** and **Python Interface** steps are needed.

Then you will have to go to the following directory  

```
cd examples/demos/swarm_demo/control_tower/multi_mav_planning/optim_problem
```
and run the ```  solver_creation_multiple.py ``` script.

Before running the demo you need to install all necessary dependencies.

```
pip install -r examples/demos/swarm_demo/control_tower/requirements.txt
```

## <u>Running the demo</u>
The demo is started by running the ```control_tower.py``` script

## <u>Description</u>
In this demo a swarm of Crazyflies is created and controlled by a control tower.
As soon as the desired number of flying copters have been taken off,random points of a rectangular area are chosen for each copter.A optimization problem is solved to find the best way to reach the chosen point while maintaining a collision free status .Both between the copters and static obstacles(in this code there is an obstacle modeled as a cylinder).

As soon the desired waypoints have been created from the solver a minimum snap trajectory is generated through them and sent to the copters.

Then each Crazyflie using the high level commander ,follows the generated trajectory.

While flying the next trajectories are calculated,generated and uploaded.As a result there is a smooth transition between the trajectories and constant motion of the Crazyflies.

Finally,after a certain amount of trajectories a trajectory that leads back to the charging station is generated and sent to each drone.

After that the copters are brought back to the charging station and the swarm is reset.The number of copters flying is changing dynamically based on their voltage in order to maintain a constant battery level for the swarm and give enough time for them to charge.When only one copter is flying it executes a predefine spiral trajectory which is not generated from the solver.




![Alt text](figures/planning.png?raw=true "Title")