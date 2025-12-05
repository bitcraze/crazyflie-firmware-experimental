import time
from cflib.utils.power_switch import PowerSwitch
from colorama import init,Fore, Back, Style

uris = [
    'radio://0/80/2M/ABAD1DEA01',
    'radio://0/80/2M/ABAD1DEA02',
    'radio://0/80/2M/ABAD1DEA03',
    'radio://0/80/2M/ABAD1DEA04',
    'radio://0/80/2M/ABAD1DEA05',
    'radio://0/80/2M/ABAD1DEA06',
    'radio://0/80/2M/ABAD1DEA07',
    'radio://0/80/2M/ABAD1DEA08',
    'radio://0/80/2M/ABAD1DEA09',
]

for uri in uris:
    try:
        pwr_switch = PowerSwitch(uri)
        pwr_switch.stm_power_cycle()
        time.sleep(1)
        pwr_switch.close()
        print(Fore.GREEN + 'Successfully reset {}'.format(uri))
    except Exception as e:
        print(Fore.RED+'Error: ', e,Fore.RESET)
        pass
