#!/usr/bin/env python3

import logging
import sys

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization import LighthouseConfigWriter


uris = [
    'radio://0/90/2M/E7E7E7E701',
    'radio://0/90/2M/E7E7E7E702',
    'radio://0/90/2M/E7E7E7E703',
    'radio://0/90/2M/E7E7E7E704',
    'radio://0/90/2M/E7E7E7E705',
    'radio://0/90/2M/E7E7E7E706',
    'radio://0/90/2M/E7E7E7E707',
    'radio://0/90/2M/E7E7E7E708',
    'radio://0/90/2M/E7E7E7E709',
]


def data_stored_cb(success):
    if success:
        print("Geometry uploaded")
    else:
        print("WARNING: Failed to upload geometry!")


def write_one(file_name: str, uri: str):
    print(f'Writing to {uri}')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        writer = LighthouseConfigWriter(scf.cf)
        writer.write_and_store_config_from_file(data_stored_cb, file_name)


if len(sys.argv) < 2:
    raise ValueError("File name missing")
file_name = sys.argv[1]

print(f"Using file {file_name}")

logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers()


for uri in uris:
    write_one(file_name, uri)
