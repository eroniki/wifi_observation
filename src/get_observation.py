#!/usr/bin/python

import time
import thread

# import FileHandler
import WifiHandler
import TextParser
import SerialHandler as sh

from wifi_observation.srv import WifiArrayService
from wifi_observation.srv import WifiArrayServiceResponse
from wifi_observation.msg import WifiMessage
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Int64

import rospy

class wifi_obs2(object):
    """docstring for wifi_obs"""
    def __init__(self):
        super(wifi_obs, self).__init__()
        rospy.init_node('wifi_obs_server')
        self.s = rospy.Service('wifi_observation', WifiArrayService, self.handle_wifi_obs)
        self.APList = list()
        self.wifi_handle = WifiHandler.WifiHandler()
        self.text_parser = TextParser.TextParser()
        rospy.loginfo("Ready to send wifi observations")
        rospy.spin()

    def construct_response(self):
        response = WifiArrayServiceResponse()
        obsList = list()
        for i in range(len(self.APList)):
            obs = WifiMessage()
            obs.ssid = self.APList[i].SSID
            obs.mac = self.APList[i].BSS
            obs.rss = self.APList[i].RSS
            obsList.append(obs)

        if(len(self.APList)>0):
            response.success = True
            response.message = "Success!"
            response.observations = obsList
        else:
            response.success = False
            response.message = "Error!"

        return response

    def handle_wifi_obs(self, data):
        rospy.loginfo("Service request captured")
        self.reading =  self.wifi_handle.query()
        self.text_parser.parse(self.reading)
        self.APList = self.text_parser.APList
        # self.text_parser.printAPList()
        return self.construct_response()

class wifi_obs(object):
    """docstring for wifi_obs."""
    def __init__(self):
        super(wifi_obs, self).__init__()
        rospy.init_node('wifi_obs_server')
        self.s = rospy.Service('wifi_observation', WifiArrayService, self.handle_wifi_obs)
        self.serialHandle = sh.ESP32Scanner()
        self.get_params()

        if self.serialHandle.init_port(self.port, self.baud)>=0:
            rospy.loginfo("Ready to send wifi observations")
            rospy.spin()
        else:
            rospy.logerr("Could NOT initialize the serial port, WiFi scanning is exiting")

    def get_params(self):
        self.port = rospy.get_param("/port_wifi")
        self.baud = rospy.get_param("/baud_wifi")

    def handle_wifi_obs(self, data):
        rospy.loginfo("Service request captured")
        text = self.serialHandle.read_line()
        qq = self.serialHandle.parse_line(text)
        qq = self.serialHandle.trim_list(qq)
        qq = self.serialHandle.init_AP_obj(qq)
        return self.construct_response(qq)

    def construct_response(self, ls):
        response = WifiArrayServiceResponse()
        if(len(ls)==1):
            response.success = False
            response.message = "Error!"
            return response

        obsList = list()
        header = Header()
        header.stamp = rospy.get_rostime()

        for i in range(len(ls)):
            obs = WifiMessage()
            obs.header = header
            obs.ssid = ls[i][0]
            obs.rss = int(ls[i][1])
            obs.mac = ls[i][2]
            obsList.append(obs)

        if(len(ls)>1):
            response.success = True
            response.message = "Success!"
            response.observations = obsList
        else:
            response.success = False
            response.message = "Error!"

        return response

if __name__ == "__main__":
    wifi_obs = wifi_obs()
