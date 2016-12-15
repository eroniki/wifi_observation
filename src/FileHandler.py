#!/usr/bin/python

import time
import thread
import os.path

class FileHandler(object):
    """docstring for FileHandler"""
    def __init__(self, file_name):
        super(FileHandler, self).__init__()
        self.file_name = file_name
        self._raw_reading = None
        self.reading = None
    def check_file_exists(self):
        return os.path.isfile(self.file_name)

    def read_from_file(self):
        if(self.check_file_exists):
            try:
                with open(self.file_name, "r") as myfile:
                    self._raw_reading=myfile.read()
                    self.reading = self._raw_reading
            except IOError:
                return "IOError"
                self.reading = None
            return self.reading
        else:
            return None

def main(arg):
    pass

if __name__ == "__main__":
    main()
