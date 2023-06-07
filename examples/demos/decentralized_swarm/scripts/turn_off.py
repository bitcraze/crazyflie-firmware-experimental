import time
from cflib.utils.power_switch import PowerSwitch
from colorama import Fore

uris = [
    'radio://0/60/2M/E7E7E7E700',
    'radio://0/60/2M/E7E7E7E701',
    'radio://0/60/2M/E7E7E7E702',
    'radio://0/60/2M/E7E7E7E703',
    'radio://0/60/2M/E7E7E7E704',
    'radio://0/60/2M/E7E7E7E705',
    'radio://0/60/2M/E7E7E7E706',
    'radio://0/60/2M/E7E7E7E707',
    'radio://0/60/2M/E7E7E7E708',
    'radio://0/60/2M/E7E7E7E709'
]

for uri in uris:
    try:
        pwr_switch = PowerSwitch(uri)
        pwr_switch.platform_power_down()
        time.sleep(1)
        pwr_switch.close()
        print(Fore.GREEN + 'Successfully turned off {}'.format(uri))
    except Exception as e:
        print(Fore.RED + 'Error: ', e, Fore.RESET)
        pass
