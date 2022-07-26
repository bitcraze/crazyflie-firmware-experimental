import time
from cflib.utils.power_switch import PowerSwitch

uris = [
    # 'radio://0/20/2M/E7E7E7E701',
    'radio://0/20/2M/E7E7E7E702',
    'radio://0/20/2M/E7E7E7E703',
    'radio://0/20/2M/E7E7E7E704',
    # 'radio://0/20/2M/E7E7E7E705',
    'radio://0/20/2M/E7E7E7E706',
    # 'radio://0/20/2M/E7E7E7E707',
    # 'radio://0/20/2M/E7E7E7E708',
    # 'radio://0/20/2M/E7E7E7E709'
]

for uri in uris:
    try:
        pwr_switch = PowerSwitch(uri)
        pwr_switch.stm_power_cycle()
        time.sleep(1)
        pwr_switch.close()
    except Exception as e:
        print('Error: ', e)
        pass