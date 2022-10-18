import threading
import time
from typing import List
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

from colorama import Fore
import cflib.crtp
import zmq
from common import MAX_COPTERS


class Copter():
    VOLTAGE_MIN = 3.0
    VOLTAGE_MAX = 4.2

    def __init__(self):
        self.state = 255
        self.voltage = 0.0
        self.counter = 0

    def get_voltage(self):
        return self._decompress_voltage(self.voltage)

    @staticmethod
    def _decompress_voltage(voltage):
        return (voltage / 255.0) * (Copter.VOLTAGE_MAX - Copter.VOLTAGE_MIN) + Copter.VOLTAGE_MIN


class SnifferInterface:
    REPORT_FREQUENCY = 2  # Hz
    LOG_FREQUENCY = 4     # Hz
    LOG_ACTIONS_FREQUENCY = 4 # Hz

    def __init__(self, uri, report_socket: zmq.Socket = None, command_socket: zmq.Socket = None):
        self.uri = uri
        self.copters: List[Copter] = None

        self.cf = Crazyflie(rw_cache='./cache')

        self.cf.fully_connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        # self.cf.console.receivedChar.add_callback(self._console_incoming) #print debug messages from Crazyflie

        self.connection_successful = None

        self.cf.open_link(uri)

        self._initialize_copters()

        self.report_socket = report_socket
        self.command_socket = command_socket

        self._console_buffer = ""

        self.desired_cfs = 0

    def _console_incoming(self, console_text):
        # print each message in one line
        if console_text[-1] != '\n':
            self._console_buffer += console_text
        else:
            self._console_buffer += console_text

            print(Fore.YELLOW + "CF {} DEBUG:".format(self.uri[-2:]), self._console_buffer, Fore.RESET)

            self._console_buffer = ""

    def _initialize_copters(self) -> List[Copter]:
        copters = []
        for i in range(MAX_COPTERS):
            copters.append(Copter())

        self.copters = copters

    def _connected(self, link_uri):
        print(Fore.GREEN + "Connected to {}".format(link_uri), Fore.RESET)
        self.connection_successful = True
        self._setup_logging()

    def _disconnected(self, link_uri):
        print(Fore.RED + "Disconnected from {}".format(link_uri))
        self.connection_successful = False

    def _connection_failed(self, link_uri, msg):
        print(Fore.RED + "Connection to {} failed: {}".format(link_uri, msg), Fore.RESET)
        self.connection_successful = False

    def _connection_lost(self, link_uri, msg):
        print(Fore.RED + "Connection to {} lost: {}".format(link_uri, msg))
        self.connection_successful = False

    def _setup_logging(self):
        # print("Setting up logging")
        self._log_conf = LogConfig(name='Sniffer', period_in_ms=1 / self.LOG_FREQUENCY * 1000)
        for i in range(MAX_COPTERS):
            self._log_conf.add_variable('id_{}.state'.format(i + 1), 'uint8_t')
            self._log_conf.add_variable('id_{}.voltage'.format(i + 1), 'uint8_t')
            # The maximum block size is 26 bytes, so the counter for the cf with id 9 is not included
            if i != 8:
                self._log_conf.add_variable('id_{}.counter'.format(i + 1), 'uint8_t')

        self.cf.log.add_config(self._log_conf)
        self._log_conf.data_received_cb.add_callback(self.log_data)
        self._log_conf.start()

        self._log_actions_conf = LogConfig(name='Actions', period_in_ms=1 / self.LOG_ACTIONS_FREQUENCY * 1000)
        self._log_actions_conf.add_variable('ds.desired')
        self.cf.log.add_config(self._log_actions_conf)
        self._log_actions_conf.data_received_cb.add_callback(self.log_actions)
        time.sleep(0.1)  # sleep to provide time for the other log
        self._log_actions_conf.start()

    def log_data(self, timestamp, data, logconf):
        # print("==========================")
        # for i in range(MAX_COPTERS):
        #     print("{}: {}".format(i, data['id_{}.state'.format(i)]))
        #     print("{}: {}".format(i, data['id_{}.voltage'.format(i)]))
        #     print("-----------------------")

        # print(data)
        for i, cop in enumerate(self.copters):
            cop.state = data['id_{}.state'.format(i + 1)]
            cop.voltage = data['id_{}.voltage'.format(i + 1)]
            cop.counter = data['id_{}.counter'.format(i + 1)] if i != 8 else -1

    def log_actions(self, timestamp, data, logconf):
        self.desired_cfs = int(data['ds.desired'])

    def send_report(self):
        if self.report_socket is None or self.connection_successful is None:
            return

        report = []
        if not self.connection_successful:
            report.append("connection_failed")
        else:
            for i, cop in enumerate(self.copters):
                data = {
                    'id': i + 1,
                    'state': cop.state,
                    'battery': cop.get_voltage(),
                    'counter': cop.counter
                }
                report.append(data)

            actions_data = {
                'id': "action",
                'desired': self.desired_cfs
            }

            report.append(actions_data)

        try:
            self.report_socket.send_json(report, zmq.NOBLOCK)
        except Exception as e:
            print(Fore.RED + "Error sending report: {}".format(e), Fore.RESET)

    def monitor(self):
        self.send_report()

        self.check_for_commands()

        time.sleep(1 / self.REPORT_FREQUENCY)

    def check_for_commands(self):
        # receive commands from the GUI

        # It contains the mapping of command(string) to function to call
        commands_map = {
            "more": self.more,
            "less": self.less,
        }

        try:
            report = self.command_socket.recv_json()
            function_to_call = commands_map[report['command']]
            function_to_call()

        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass  # no message was ready (yet!)
        except Exception as e:
            print(Fore.RED + "Error receiving command: {}".format(e), Fore.RESET)

    def disconnect(self):
        self.cf.close_link()

    def more(self):
        self.cf.param.set_value('app.more', 1)

    def less(self):
        self.cf.param.set_value('app.less', 1)

class snifferThread(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(snifferThread, self).__init__(*args, **kwargs)
        self.daemon = True

        self._stop_thread = threading.Event()

    def stop_sniffer(self):
        self._stop_thread.set()

    def stopped(self):
        return self._stop_thread.isSet()

    def run(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)

        context = zmq.Context()

        pub_socket = context.socket(zmq.PUSH)
        pub_socket.bind("tcp://*:5555")

        sub_socket = context.socket(zmq.PULL)
        # report_socket.connect("tcp://bitcrazeDemo:5556")
        sub_socket.connect("tcp://127.0.0.1:5556")
        sub_socket.setsockopt(zmq.RCVTIMEO, 1000)

        uri = 'usb://0'
        sniffer = SnifferInterface(uri, report_socket=pub_socket, command_socket=sub_socket)

        while True:
            if self.stopped():
                print(Fore.RED + "Sniffer thread stopped", Fore.RESET)
                sniffer.disconnect()
                break

            sniffer.monitor()


def sniffer_interface_main():
    cflib.crtp.init_drivers(enable_debug_driver=False)

    context = zmq.Context()

    pub_socket = context.socket(zmq.PUSH)
    pub_socket.bind("tcp://*:5555")

    sub_socket = context.socket(zmq.PULL)
    # report_socket.connect("tcp://bitcrazeDemo:5556")
    sub_socket.connect("tcp://127.0.0.1:5556")
    sub_socket.setsockopt(zmq.RCVTIMEO, 1000)

    uri = 'usb://0'
    sniffer = SnifferInterface(uri, report_socket=pub_socket, command_socket=sub_socket)

    while True:
        sniffer.monitor()


if __name__ == "__main__":
    sniffer_interface_main()
