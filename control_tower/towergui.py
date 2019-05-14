#!/usr/bin/env python3
import tkinter
from tkinter import *
import tkinter.ttk as ttk
import zmq
import threading


class Crazyflie(ttk.Frame):
    def __init__(self, parent, ident):
        ttk.Frame.__init__(self, parent)

        self['padding'] = 10

        self._name = Label(self, text="Crazyflie #{}".format(ident+1))
        self._name.grid(row=0, column=0)

        self._status = Label(self, text="IDLE", fg='grey', font=("ubuntu", 33), width=10)
        self._status.grid(row=1, column=0)

        self._battery_label = Label(self, text="Battery:")
        self._battery_label.grid(row=2, column=0)

        self._battery_frame = ttk.Frame(self)
        self._battery_frame.grid(row=3, column=0, sticky="ew")
        self._battery_frame.columnconfigure(1, weight=2)

        self._battery_voltage = ttk.Label(self._battery_frame, text="3.0V", padding=(0,0,10,0))
        self._battery_voltage.grid(row=0, column=0)
        
        self._battery_bar = ttk.Progressbar(self._battery_frame, orient=HORIZONTAL)
        self._battery_bar['value'] = 50
        self._battery_bar.grid(row=0, column=1, sticky="ew")
    
    def set_state(self, state):
        if state == "idle":
            self._status.config(text="IDLE", fg="grey")
        elif state == "charging":
            self._status.config(text="Charging", fg="red")
        elif state == "ready":
            self._status.config(text="Ready", fg="orange")
        elif state == "flying":
            self._status.config(text="Flying", fg="green")
        elif state == "hovering":
            self._status.config(text="Hovering", fg="green")
        elif state == "landing":
            self._status.config(text="Landing", fg="green")
        else:
            self._status.config(text="ERROR", fg="purple")

    def set_battery(self, voltage):
        self._battery_voltage['text'] = "{:.1f}V".format(voltage)

        percent = (voltage - 3.0)*100.0/1.1

        self._battery_bar['value'] = percent


root = tkinter.Tk()
root.title("ICRA2019 control tower")

content = ttk.Frame(root)
content.grid(column=0, row=0)
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

cfs = []

for i in range(8):
    r = int(i/4)
    c = int(i%4)
    cf = Crazyflie(content, i)
    cf.grid(column=c, row=r)
    cfs.append(cf)

    cf.set_state("charging")
    cf.set_battery(3.0+(i/10.0))

cfs[0].set_state("charging")
cfs[1].set_state("ready")
cfs[2].set_state("flying")
cfs[3].set_state("plop")
cfs[4].set_state("idle")
cfs[0].set_battery(5.0)
cfs[1].set_battery(1.0)

context = zmq.Context()

socket = context.socket(zmq.PULL)
socket.connect("tcp://127.0.0.1:5555")


def receive_thread():
    while True:
        report = socket.recv_json()
        print(report)

        cfs[report['id']].set_battery(report['battery'])
        cfs[report['id']].set_state(report['state'])


threading.Thread(target=receive_thread, daemon=True).start()

root.mainloop()
