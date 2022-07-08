#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2022 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

from simpleSwarmInterface import *

if __name__ == '__main__':
    param_value=("app.reset_crash_state",1)
    # param_value=("led.bitmask",255)

    cflib.crtp.init_drivers(enable_debug_driver=False)

    h=Handler(uris,param_value)
    while True:
        got_callback=[i.is_connected!=None for i in h.cfs]

        if all(got_callback):
            time.sleep(2)
            for i in h.cfs:
                i.terminate_thread()
            break

        time.sleep(0.5)
        
