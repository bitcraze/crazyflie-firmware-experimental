import time
from cflib.utils.power_switch import PowerSwitch
from colorama import Fore

uris = [
    'radio://0/10/2M/E7E7E7E700',
]

for uri in uris:
    try:
        pwr_switch = PowerSwitch(uri)
        pwr_switch.stm_power_cycle()
        time.sleep(1)
        pwr_switch.close()
        print(Fore.GREEN + 'Successfully reset {}'.format(uri))
    except Exception as e:
        print(Fore.RED + 'Error: ', e, Fore.RESET)
        pass
