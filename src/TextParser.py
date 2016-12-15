#!/usr/bin/python
import re
import AP

class TextParser(object):
    """docstring for TextParser"""
    def __init__(self):
        super(TextParser, self).__init__()
        self.__BSSList    = None
        self.__RSSList    = None
        self.__SSIDList   = None
        self.__zippedList = None
        self.__RSS        = None
        self.__BSS        = None
        self.__ap         = None
        self.APList       = list()

    def parse(self, iw_dump):
        self.__BSSList    = None
        self.__RSSList    = None
        self.__SSIDList   = None
        self.__reading    = iw_dump
        self.APList = list()
        self.findBSS(r"\bBSS\b")
        self.findRSS(r"\bsignal\b")
        self.findSSID()
        self.createAPList()


    def findBSS(self, hash):
        iter = re.finditer(hash, self.__reading)
        self.__BSSList = [m.start(0) for m in iter]

    def findRSS(self, hash):
        iter = re.finditer(hash, self.__reading)
        self.__RSSList = [m.start(0) for m in iter]

    def findSSID(self):
        self.__SSIDList = re.findall(r"SSID:\s*([^\n\r]*)", self.__reading)

    def createAPList(self):
        zippedList = zip(self.__RSSList, self.__BSSList, self.__SSIDList)

        for i in zippedList:
            self._RSS = self.__reading[i[0]+7:i[0]+14]
            self._BSS = self.__reading[i[1]+4:i[1]+21]
            self._ap = AP.AP(self._BSS, self._RSS, i[2])
            self.APList.append(self._ap)

    def printAPList(self):
        for i in range(len(self.APList)):
            print self.APList[i].SSID, self.APList[i].BSS, self.APList[i].RSS

def main(arg):
    pass

if __name__ == "__main__":
    main()
