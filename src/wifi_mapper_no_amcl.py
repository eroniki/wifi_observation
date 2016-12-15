#!/usr/bin/python
import time
import os
import thread
import numpy as np
import cv2

from wifi_observation.srv import WifiArrayService
from wifi_observation.srv import WifiArrayServiceResponse
from wifi_observation.srv import SignalMapArrayService
from wifi_observation.srv import SignalMapArrayServiceResponse
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse

from wifi_observation.msg import WifiMessage
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Int64
from std_msgs.msg import Empty

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

import rospy

class wifi_map(object):
    """docstring for wifi_map"""
    def __init__(self):
        super(wifi_map, self).__init__()
        rospy.init_node('wifi_obs_server_no_map')
        self.obs_response = None
        self.epoch = int(time.time())
        self.foldername = rospy.get_param("/db_folder")
        self.filename = self.foldername + str(self.epoch) + ".txt"
        self.r = rospy.Rate(100)
        self.id = None
        try:
            open(self.filename, "w")
        except Exception as e:
            print "File couldn't opened"
            open(self.filename, "w+")


        rospy.loginfo("Ready to acquire wifi observations")
        while not rospy.is_shutdown():
            self.id = raw_input("Please indicate the location index:")
            print self.id
            self.get_wifi_observation()
            self.r.sleep()

    def get_wifi_observation(self):
        rospy.wait_for_service('wifi_observation')
        wifi_observation = rospy.ServiceProxy('wifi_observation', WifiArrayService)
        self.obs_response = None
        try:
            self.obs_response = wifi_observation()
            rospy.loginfo("WiFi observations acquired with return code and message %s, %s.\n Total number of observations acquired: %d", self.obs_response.success, self.obs_response.message, len(self.obs_response.observations))
            epoch = int(time.time())
            with open(self.filename, "a") as text_file:
                for i in range(len(self.obs_response.observations)):
                    line = str(epoch) + '\t' + str(self.obs_response.observations[i].mac) + '\t' + str(self.obs_response.observations[i].rss) + '\t' + \
                    str(self.obs_response.observations[i].ssid) + '\t' + str(self.id) + '\n'
                    text_file.write(line)
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: %s" + str(exc))

    def maintain_maps(self):
        ### TODO: Define what posx, posy is.
        ### TODO: Find out a way to represent position in OccupancyGrid with the scale of meters

        pos = self.pose_to_grid_pixel()
        posx = pos[0]
        posy = pos[1]

        rospy.loginfo("X: %d Y: %d", posx, posy)

        for i in range(len(self.obs_response.observations)):
            rospy.loginfo("The signal map is being updated/created with corresponding mac label: %s", self.obs_response.observations[i].mac)
            try:
                previous_map = self.signal_maps_dict[self.obs_response.observations[i].mac]
            except KeyError:
                previous_map = OccupancyGrid()
                previous_map.info = self.map_response.map.info
                zeros_list = np.zeros((self.map_response.map.info.width*self.map_response.map.info.height,1))
                previous_map.data = list(zeros_list)
            except Exception as e:
                rospy.loginfo("Exception caught while acquiring the previous map: %s", e)
                return

            try:
                # print type(previous_map.data), len(previous_map.data), posx, posy, posy*self.map_response.map.info.width+posx
                previous_map.data[posy*self.map_response.map.info.width+posx] = -1*self.obs_response.observations[i].rss
                # previous_data_as_np_array = np.asarray(previous_map.data).reshape(-1)

                ## Gaussian Assumption (Wrong!)
                # gx = self.gaussian2D(np.array([posy,posx]), self.std, np.array([self.map_response.map.info.height,self.map_response.map.info.width]), self.spatial_resolution).ravel()
                # print np.amin(gx), np.amax(gx), self.obs_response.observations[i].rss, np.isnan(gx).any(), np.isnan(previous_data_as_np_array).any()
                # previous_map.data = list(np.int8(previous_data_as_np_array-1*self.obs_response.observations[i].rss*gx))


            except Exception as e:
                rospy.loginfo("Exception caught while updating the signal map: %s", e)

            self.signal_maps_dict[self.obs_response.observations[i].mac] = previous_map

if __name__ == "__main__":
    wifi_map = wifi_map()
