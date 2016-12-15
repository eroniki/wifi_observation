#!/usr/bin/python

import subprocess

class WifiHandler(object):
    """docstring for WifiHandler"""
    def __init__(self):
        super(WifiHandler, self).__init__()
        self.response = None

    def query(self):
        try:
            self.response = subprocess.check_output(["iw", "dev", "wlan0", "scan"])
            # self.response = subprocess.check_output(["whoami"])
        except subprocess.CalledProcessError:
            print "No access to WiFi card, try running the script with sudo priviligies."
        return self.response

def main():
    pass

if __name__ == "__main__":
    main()
