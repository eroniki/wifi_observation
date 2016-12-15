#!/usr/bin/python

import time
import thread

# import FileHandler
import WifiHandler
import TextParser

from wifi_observation.srv import WifiArrayService
from wifi_observation.srv import WifiArrayServiceResponse
from wifi_observation.msg import WifiMessage
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Int64

import rospy

class wifi_obs(object):
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

if __name__ == "__main__":
    wifi_obs = wifi_obs()
