#!/usr/bin/python

import time
import thread
import serial
import io
import signal
import sys

class ESP32Scanner(object):
    """docstring for ESP32Scanner."""
    # def __init__(self, port, baud):
    def __init__(self):
        super(ESP32Scanner, self).__init__()
        signal.signal(signal.SIGINT, self.signal_handler)
        # self.port = None
        # self.baudrate = None
        # self.ser = None

    def init_port(self, port, baud):
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud)
            return 0
        except Exception as e:
            print "Exception caught while openning the serial port"
            print e
            return -1

    def read_line(self):
        return self.ser.readline()

    def parse_line(self, text, delimiter=">"):
        return text.split(delimiter)

    def init_AP_obj(self, ls, delimiter="|"):
        ap = list()
        for i in range(len(ls)):
            ap.append(ls[i].split(delimiter))
        return ap

    def trim_list(self, ls):
        if len(ls)>1:
            ls.pop()
            ls.pop(0)
        return ls

    def signal_handler(self, signal, frame):
        print('You pressed Ctrl+C!')
        self.ser.close()
        sys.exit(0)

if __name__ == "__main__":
    pass
