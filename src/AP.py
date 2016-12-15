#!/usr/bin/python
import re

class AP(object):
    """docstring for AP"""
    def __init__(self, BSS, RSS, SSID):
        super(AP, self).__init__()
        self.BSS = BSS
        self.RSS = self.num(RSS)
        self.SSID = SSID

    def num(self, s):
        try:
            return int(s)
        except ValueError:
            return float(s)

def main(arg):
    pass

if __name__ == "__main__":
    main()
