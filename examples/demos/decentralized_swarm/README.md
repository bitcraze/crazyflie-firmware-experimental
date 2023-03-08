# Decentralized Swarm Demo

## Description

This folder contains the app layer application for a swarm of Crazyflies that fly autonomously by communicating with each other through the Peer to Peer Protocol (P2P). The swarm is composed of an arbitrary  number of Crazyflies with the maximum number of Crazyflies allowed being defined in the `MAX_ADDRESS` (no more than 9) definition in settings.h. All the copters start placed on their charging pads. The goal is to have a certain number of drones flying and avoid collision while the rest are being charged and wait.

The demo it intended to be used with the Lighthouse positioning system.

There are three types of participants in the system:
* Pilot - one of the flying Crazyflies
* Sniffer - a Crazyflie that is listening to the P2P traffic and is used to visualizing the system state. It is also used to broadcast changes in the desired nr of flying Crazyflies.
* Human tracker - a Crazyflie that broadcasts its position, much like the pilots, but without flying. Pilots will avoid the human tracker and it can be used to enter the flying space.

## Set up

Get a bunch of Crazyflies, configure them to use the same channel and addresses where the last byte is 00 for the sniffer and 01 to 09 for the others.
Connect the shiffer to a computer via USB.

Place charging pads in the flying space.

## Building the Demo

Make sure that you are in the decentralized_swarm folder (not the main folder of the crazyflie firmware).

Edit `settings.h` and set the bounds (MIN_X_BOUND, MAX_X_BOUND, MIN_Y_BOUND...) to fit the flying space (cage).

Chose which type of app (pilot, sniffer or human tracker) to build by modifying the `chose_app.h` file.

Type the following to build and flash it while the crazyflie is put into bootloader mode:
```
make clean
make
make cload
```

The pilot firmware can be flashed to multiple Crazyflies using the `cload-all.sh` script, to speed up the process.
```
./cload-all.sh
```

Install requirements for the GUI
```
pip install -r requirements.txt
```

## Running the demo

Put the pilot Crazyflies on the charging pads and turn them on.

Connect the sniffer to your computer via USB and start the gui
```
./GUI/towergui.py
```

In the GUI, click the "More" and "Less" buttons to set the desired nr of flying Crazyflies.

## Handling crashes

If a pilot crashes. put it back on the charging pad (the correct one) and restart it.

## Entering the space with a human tracker

To use a Crazyflie as a human tracker, flash it with the human tracker flavor of the firmware. The Crazyflie must be configured
as one of the pilot Crazyfles, that is address 01 to 09. Hold the tracker (or put it on your head) so that it can receive
the lighthouse base stations and track its position.
The pilot Crazyflies will (should) avoid the space around the tracker (human), but note that the space is fairly small.

## Resources
You can find on Bitcraze's website the [API documentation for P2P](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/p2p_api/) as well as the [App layer API guide](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

Further information bout the collision avoidance algorithm can be found on the original [paper](https://web.stanford.edu/~schwager/MyPapers/ZhouEtAlRAL17CollisionAvoidance.pdf) and on the file  ``collision_avoidance.h``

## Limitations

Since P2P communication happens asynchronously on the radio, this example does not work well when connecting a PC to the Crazyflies via the Radio.This is a fundamental limitation of the current P2P implementation.You should only connect the Crazyflies to start and terminate the swarm but the suggested way of interaction with the swarm is through the sniffer and GUI.

## Limitations
In case a reset of the copters is needed, the script `power_reset.py` can be used to power reset all the copters automatically.
