#!/usr/bin/env python

# originally from johnl found at
# http://ubuntuforums.org/showthread.php?t=1604981&p=10024295#post10024295

import subprocess
import re

class line_matcher:
    def __init__(self, regexp, handler):
        self.regexp  = re.compile(regexp)
        self.handler = handler


def handle_new_network(line, result, networks):
    # group(1) is the mac address
    networks.append({})
    networks[-1]['Address'] = result.group(1)

def handle_essid(line, result, networks):
    # group(1) is the essid name
    networks[-1]['ESSID'] = result.group(1)

def handle_quality(line, result, networks):
    # group(1) is the quality value
    # group(2) is probably always 100
    networks[-1]['Quality'] = result.group(1) + '/' + result.group(2)

def handle_unknown(line, result, networks):
    # group(1) is the key, group(2) is the rest of the line
    networks[-1][result.group(1)] = result.group(2)

if __name__ == '__main__':
    proc = subprocess.Popen(['/sbin/iwlist', 'wlan0', 'scan'], stdout=subprocess.PIPE)
    stdout, stderr =  proc.communicate()

    lines = stdout.decode('utf-8').split('\n')

    networks = []
    matchers = []

    # catch the line 'Cell ## - Address: XX:YY:ZZ:AA:BB:CC'
    matchers.append(line_matcher(r'\s+Cell \d+ - Address: (\S+)', handle_new_network))

    # catch the line 'ESSID:"network name"
    matchers.append(line_matcher(r'\s+ESSID:"([^"]+)"', handle_essid))

    # catch the line 'Quality:X/Y Signal level:X dBm Noise level:Y dBm'
    matchers.append(line_matcher(r'\s+Quality:(\d+)/(\d+)', handle_quality))

    # catch the line 'Quality=X/Y  Signal level=X dBm'
    matchers.append(line_matcher(r'\s+Quality=(\d+)/(\d+)', handle_quality))

    # catch any other line that looks like this:
    # Key:value
    matchers.append(line_matcher(r'\s+([^:]+):(.+)', handle_unknown))

    # read each line of output, testing against the matches above
    # in that order (so that the key:value matcher will be tried last)
    for line in lines:
        for m in matchers:
            result = m.regexp.match(line)
            if result:
                m.handler(line, result, networks)
                break

    for n in networks:
        print('Found network', n['ESSID'], 'Quality', n['Quality'])
        # to see the whole dictionary:
        # print(n)
