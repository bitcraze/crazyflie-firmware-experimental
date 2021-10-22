# De-centralized swarm demo for BAM 2021

This repos contains the swarm demo for the BAM days 2021.
It uses the out-of-tree build functionality of the Crazyflie firmware and is implemented using the app entry-point.

## Overview

This demo implements a swarm with de-centralized decision making to control which Crazyflie that is flying. It uses
P2P communication and a paxos scheme to agree and update a shared knowledge of the swarm state.

## Usage

Nine Crazyflies are placed on charging pads on the floor around the origin. When the swarm is activated one
Crazyflie will take off and fly a spiral trajectoy. When done it goes back to its charging pad and lands, at
the same time the next Crazyflie takes of to fly the trajectory and so on.

### Activation

The swarm is activated by connecting to one of the Crazyflies with the client and setting the "app.active" parameter to 1.
The other Crazyflies will automatically be activated when they receive messages on the P2P network.

If a Crazyflie crashes it can simply be restarted and put back on the charging pad, it will automatically
re-join the swarm.

Note: the automatic activation only works after a re-boot.

### De-activation

Connect to one of the Crazyflies and set the "app.deactiveAll" parameter to 1. This will broadcast a
deactivation message. You might have to repeat this a few times to make sure all nodes received the message.

## Hardware requirements

9 Crazyflies with Lighthouse decks and Qi-charger decks.
9 3D-printed charging pads with Qi-chargers from IKEA.
2 Lighthouse V2 base stations
1 Computer with a Crazyradio

## Build

You must have the required tools to build the [Crazyflie firmware](https://github.com/bitcraze/crazyflie-firmware).

Clone the repos with ```--recursive```. If you did not do so, pull submodules with:
```
git submodule update --init --recursive
```

Then build and bootload:
```
make -j$(nproc)
make cload
```

## Sniffer

* Build the special sniffer flavour of the Crazyflie firmware by defining BUILD_SNIFFER when building.
* Flash the binary to a separate Crazyflie and connect it via USB to the computer.
* Run the sniffer.py script to display a log of all received messages
