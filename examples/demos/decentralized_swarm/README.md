# Decentralized Swarm Demo

## Description

This folder contains the app layer application for a swarm of Crazyflies that fly autonomously by communicating each other through the Peer to Peer Protocol (P2P). The swarm is composed of an arbitrary  number of Crazyflies with the maximum number of Crazyflies allowed being defined in the `MAX_ADDRESS` (no more than 9) definition in settings.h .All the copters start placed on their charging pads.The goal is to have a certain number of drones flying and avoid collision  while the rest are being charged and wait.

The demo starts by setting the parameter  `take_off`  to 1, in  at least one of the Crazyflie of the swarm. After that, all the copters assign a random time to wait before taking off. In case the desired number of flying copters is reached during this time, the swarm is considered to be ready and the particular copter will remain landed,otherwise it takes off.Through this randomization the swarm is able to be led to consensus on the flying copters at each time step,despite some false take offs that may happen.

As soon as a Crazyflie is airborne, it starts to broadcast each position through the peer to peer protocol and it enables the on board, online collision avoidance algorithm based on Buffered Voronoi cells.In this way ,each copter is aware of the position of the other ones and can avoid collision.While flying , copters pick random positions on a circle and move to them until a certain flight time pass by.After that, they land and wait for a new random time to take off as explained above.

In order to terminate the demo, the user must set the parameter `terminateApp` to 1 in at least one of the Crazyflie of the swarm.Then this information is broadcasted to all the copters and they will stop flying and land.

The structure of each P2P packet is as follows:
<pre>
|--------------------|
|   <b>PACKET FORMAT</b>    |
|--------------------|
| Id (1 byte)        |
|--------------------|
| counter (1 byte)   |
|--------------------|
| state (1 byte)     |
|--------------------|
| position (12 bytes)|
|--------------------|
| Voltage (1 byte)   |
|--------------------|
| Terminate (1 byte) |
|   signal           |
|--------------------|
</pre>


## GUI
A GUI is also provided to control and monitor the swarm.The communication with it is achieved through a static Crazyflie which acts as a sniffer for all the P2P packets sent by the copters.Keep in mind that the sniffer must be connected through USB to the PC in order not to interfere with the P2P radio communication.The user can also command the take off and the termination through the GUI and monitor the state and voltage of each copter.


## Project Structure

```
.
├── config/
│   ├── drones_config.yaml      # Swarm configuration (drones, URIs, platforms)
│   ├── CageGeoEst.yaml         # Lighthouse geometry
│   ├── app-config              # App config for CF2
│   └── app-config-brushless    # App config for CF21BL
├── scripts/                    # Utility scripts
├── flash_all.py                # Mass flashing tool
├── towergui.py                 # GUI for monitoring/control
└── src/                        # Firmware source
```

## Setup

Install dependencies (creates virtual environment):
```bash
source .venv/bin/activate  # or: .venv/bin/python
```

## Configuration

Edit `config/drones_config.yaml` to define your swarm:
- Drone IDs and URIs
- Platform types (cf2, cf21bl)
- App types (pilot, sniffer)

**Default**: Sniffer on ID 0 (`...EA00`), pilots on IDs 1-9 (`...EA01` to `...EA09`)

## Building and Flashing

The `flash_all.py` script handles building and flashing with automatic app selection:

```bash
# Flash all drones
python flash_all.py --all

# Flash specific drones
python flash_all.py --ids 1 2 3

# Flash by type
python flash_all.py --app-type sniffer
python flash_all.py --platform cf21bl

# Flash a range
python flash_all.py --range 1-6
```

The script automatically:
- Builds firmware with correct app flags (`BUILD_PILOT_APP` or `BUILD_SNIFFER_APP`)
- Handles different platforms (cf2, cf21bl)
- Flashes via radio with warm boot
- Shows progress bars

## GUI

Run the tower GUI for swarm monitoring and control:
```bash
python towergui.py
```

The GUI connects to the sniffer drone (USB) to monitor P2P packets and control takeoff/termination.
## Resources
You can find on Bitcraze's website the [API documentation for P2P](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/p2p_api/) as well as the [App layer API guide](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/userguides/app_layer/)

Further information bout the collision avoidance algorithm can be found on the original [paper](https://web.stanford.edu/~schwager/MyPapers/ZhouEtAlRAL17CollisionAvoidance.pdf) and on the file  ``collision_avoidance.h``

## Utility Scripts

- `scripts/power_reset.py` - Power cycle all drones
- `scripts/turn_off.py` - Turn off all drones

## Limitations

Since P2P communication happens asynchronously on the radio, this example does not work well when connecting a PC to the Crazyflies via the Radio. This is a fundamental limitation of the current P2P implementation. You should only connect the Crazyflies to start and terminate the swarm but the suggested way of interaction with the swarm is through the sniffer and GUI.
