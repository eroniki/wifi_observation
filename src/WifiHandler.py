#!/usr/bin/python

import subprocess
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__))+'/python-nm-tool/')
import nm_tool

class WifiHandler(object):
    """docstring for WifiHandler"""
    def __init__(self):
        super(WifiHandler, self).__init__()
        self.response = None
        print sys.path
    def query(self):
        self.response = nm_tool.get_dict()
        return self.response['wlan0']['scan_results']

def main():
    pass

if __name__ == "__main__":
    main()
