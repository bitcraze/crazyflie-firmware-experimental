#!/usr/bin/env python3
import tkinter as tk
import tkinter
import tkinter.messagebox
from tkinter import *
import tkinter.ttk as ttk
from colorama import Fore
from typing import List

import zmq
import threading
import time
from common import *

from sniffer_interface import snifferThread
from queue import Queue

def _from_rgb(r,g,b):
    """
    Translates an rgb tuple of int to a tkinter friendly color code
    """
    rgb=(r,g,b)
    return "#%02x%02x%02x" % rgb

def showError():
    #Shows the error message in the GUI through a queue that is handled by the main thread
    q.put(( tkinter.messagebox.showerror,("Error Detected.",  "Unsuccessful connection with sniffer. Please try again."),{}))

class Crazyflie(ttk.Frame):
    def __init__(self, parent, ident):
        ttk.Frame.__init__(self, parent)

        self['padding'] = 15        

        self._name = Label(self, text="Crazyflie #{}".format(ident))
        self._name.grid(row=0, column=0)

        self._status = Label(self, text="IDLE", fg='grey',
                             font=("ubuntu", 33), width=15)
        self._status.grid(row=1, column=0)

        self._battery_label = Label(self, text="Battery:")
        self._battery_label.grid(row=2, column=0)

        self._battery_frame = ttk.Frame(self)
        self._battery_frame.grid(row=3, column=0, sticky="ew")
        self._battery_frame.columnconfigure(1, weight=2)

        self._battery_voltage = ttk.Label(
            self._battery_frame, text="3.0V", padding=(0, 0, 10, 0))
        self._battery_voltage.grid(row=0, column=0)

        self._battery_bar = ttk.Progressbar(
            self._battery_frame, orient=HORIZONTAL)
        self._battery_bar['value'] = 50
        self._battery_bar.grid(row=0, column=1, sticky="ew")

    def set_state(self, state):
        if state in state_dict:
            self._status.config(text=state_dict[state][0], fg=state_dict[state][1])
        else:
            self._status.config(text="ERROR", fg="grey")
            # print("Error, state", state, "not handled")

    def set_battery(self, voltage):
        self._battery_voltage['text'] = "{:.2f}V".format(voltage)

        percent = (voltage - 3.0)*100.0/1.1

        self._battery_bar['value'] = percent

    def set_uptime(self, ms):
        seconds = int(ms/1000) % 60
        minutes = int((ms/1000)/60) % 60
        hours = int((ms/1000)/3600)
        self._up_time_label['text'] = "{}:{:02}:{:02}".format(
            hours, minutes, seconds)
        if ms == 0:
            self._up_time_label['fg'] = "grey"
        else:
            self._up_time_label['fg'] = "black"

    def set_flighttime(self, ms):
        seconds = int(ms/1000) % 60
        minutes = int((ms/1000)/60) % 60
        hours = int((ms/1000)/3600)
        self._flight_time_label['text'] = "{}:{:02}:{:02}".format(
            hours, minutes, seconds)
        if ms == 0:
            self._flight_time_label['fg'] = "grey"
        else:
            self._flight_time_label['fg'] = "black"

class ButtonsFrame(ttk.Frame):
    WIDTH = 40
    HEIGHT = 2
    PADX = 20
    PADY = 10

    def __init__(self, parent,interface_socket:zmq.Socket):
        ttk.Frame.__init__(self, parent)

        self.socket = interface_socket

        # Buttons frame
        buttons_frame = ttk.Frame(parent)

        # Take off button
        takeoff_button = tk.Button(buttons_frame, text="TAKE OFF", command=self.take_off, 
                                    width=self.WIDTH, height=self.HEIGHT,
                                    background=_from_rgb(93, 227, 9),
                                    activebackground=_from_rgb(93, 215, 9), 
                                    activeforeground="#000")
        takeoff_button.grid(row=0, column=0, padx=self.PADX, pady=self.PADY)

        # land button
        land_button = tk.Button(buttons_frame, text="LAND", command=self.terminate , 
                                    width=self.WIDTH, height=self.HEIGHT,
                                    background=_from_rgb(240, 20, 11),
                                    activebackground=_from_rgb(212, 20, 11), 
                                    activeforeground="#000")

        land_button.grid(row=0, column=1, padx=self.PADX, pady=self.PADY)

        # insert buttons frame in the content
        buttons_frame.grid(row=3, column=0, columnspan=3)

    def _send_command(self, command):
        command_obj={
            "command": command,
        }

        try:
            self.socket.send_json(command_obj, zmq.NOBLOCK)
        except Exception as e:
            print(Fore.RED + "Error sending report: {}".format(e),Fore.RESET)

    def take_off(self):
        print("Take off")
        self._send_command("take_off")
        

    def terminate(self):
        print("Terminate")
        self._send_command("terminate")

sniffer_thread = snifferThread()
sniffer_thread.start()

context = zmq.Context()

report_socket = context.socket(zmq.PULL)
# report_socket.connect("tcp://bitcrazeDemo:5555")
report_socket.connect("tcp://127.0.0.1:5555")
report_socket.setsockopt(zmq.RCVTIMEO, 1000)

command_socket = context.socket(zmq.PUSH)
command_socket.bind("tcp://*:5556")

root = tkinter.Tk()
root.title("Decentralized Swarm Control Tower")

content = ttk.Frame(root)
content.grid(column=0, row=0)
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

cfs: List[Crazyflie] = []

for i in range(MAX_COPTERS):
    r = int(i/3)
    c = int(i % 3)
    cf = Crazyflie(content, i+1)# +1 to avoid 0 which is reserved for the sniffer
    cf.grid(column=c, row=r)
    cfs.append(cf)

    cf.set_state("error")
    cf.set_battery(3.0+(i/10.0))

buttons = ButtonsFrame(content,command_socket)

q = Queue()

def tkloop():
    #Used in order to show the error message in the GUI if connection with the sniffer fails
    try:
        while True:
            f, a, k = q.get_nowait()
            f(*a, **k)
            root.destroy()
            return
    except Exception as e:
        pass

    root.after(100, tkloop)


def receive_thread():
    last_updated = [0]*len(cfs)
    while True:
        try:
            report = report_socket.recv_json()
            
            if report[0] == "connection_failed":
                print(Fore.RED,"Connection with sniffer failed!",Fore.RESET)
                showError()
                break

            print("===================================================================================")
            for i,data in enumerate(report):
                # print(report['id'],type(report['id']))
                # -1 because index starts at 0 and all flying copters have adrreses >=
                print(data)
                id = data['id'] -1 
                cfs[id].set_battery(data['battery'])
                cfs[id].set_state(data['state'])
                last_updated[id] = time.time()

        except zmq.error.Again:
            pass

        for i in range(len(cfs)):
            if last_updated[i] < (time.time()-2):
                cfs[i].set_state("idle")
                cfs[i].set_battery(0)


receiving_thread = threading.Thread(target=receive_thread, daemon=True)
receiving_thread.start()

tkloop()
root.mainloop()

#Terminate the sniffer thread
sniffer_thread.stop_sniffer()
sniffer_thread.join()
