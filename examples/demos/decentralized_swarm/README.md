# Decentralized Swarm Demo

## Description

This folder contains the app layer application for a swarm of Crazyflies that fly autonomously by communicating each other through the Peer to Peer Protocol (P2P). The swarm is composed of an arbitrary  number of Crazyflies with the maximum number of Crazyflies allowed being defined in the `MAX_ADDRESS` (no more than 9) definition in settings.h .All the copters start placed on their charging pads.The goal is to have a certain number of drones flying and avoid collision  while the rest are being charged and wait.

The demo starts by setting the parameter  `take_off`  to 1, in  at least one of the Crazyflie of the swarm. After that, all the copters assign a random time to wait before taking off. In case the desired number of flying copters is reached during this time, the swarm is considered to be ready and the particular copter will remain landed,otherwise it takes off.Through this randomization the swarm is able to be led to consensus on the flying copters at each time step,despite some false take offs that may happen.

As soon as a Crazyflie is airborne, it starts to broadcast each position through the peer to peer protocol and it enables the on board, online collision avoidance algorithm based on Buffered Voronoi cells.In this way ,each copter is aware of the position of the other ones and can avoid collision.While flying , copters pick random positions on a circle and move to them until a certain flight time pass by.After that, they land and wait for a new random time to take off as explained above.

In order to terminate the demo, the user must set the parameter `terminateApp` to 1 in at least one of the Crazyflie of the swarm.Then this information is broadcasted to all the copters and they will stop flying and land.

## GUI
A GUI is also provided to control and monitor the swarm.The communication with it is achieved through a static Crazyflie which acts as a sniffer for all the P2P packets sent by the copters.Keep in mind that the sniffer must be connected through USB to the PC in order not to interfere with the P2P radio communication.The user can also command the take off and the termination through the GUI and monitor the state and voltage of each copter.


## Building the Demo
Make sure that you are in the decentralized_swarm folder (not the main folder of the crazyflie firmware). **It is assumed that the sniffer has a radio address ending in '00' and all the swarm copters from '01' to '09'**.

If you want to build the sniffer app the `BUILD_SNIFFER_APP` flag must be defined in the `choose_app.h` header file and then type the following to build and flash it while the crazyflie is put into bootloader mode:
```
make clean
make 
make cload
```

If you want to build the pilot app for the swarm copters the `BUILD_PILOT_APP` flag must be defined in the `choose_app.h` header file and then the standard flashing procedure can be used for each crazyflie or use the `cload-all.sh` script to flash all the copters automatically:
```
./cload-all.sh
```


If you want to use the GUI for monitoring the state of each copter and controlling the swarm ,the python libraries in requirements.txt must be installed by running the following command:
```
pip install -r requirements.txt
```

and then executing the script `towergui.py` in the folder GUI.
## Resources
You can find on Bitcraze's website the [API documentation for P2P](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/p2p_api/) as well as the [App layer API guide](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

Further information bout the collision avoidance algorithm can be found on the original [paper](https://web.stanford.edu/~schwager/MyPapers/ZhouEtAlRAL17CollisionAvoidance.pdf) and on the file  ``collision_avoidance.h``

## Limitations

Since P2P communication happens asynchronously on the radio, this example does not work well when connecting a PC to the Crazyflies via the Radio.This is a fundamental limitation of the current P2P implementation.You should only connect the Crazyflies to start and terminate the swarm but the suggested way of interaction with the swarm is through the sniffer and GUI.

## Limitations
In case a reset of the copters is needed, the script `power_reset.py` can be used to power reset all the copters automatically.
