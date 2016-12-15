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
        rospy.init_node('wifi_obs_server')
        self.s = rospy.Service('/signal_map', SignalMapArrayService, self.handle_signal_map)
        self.poseSubscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.location_callback)
        self.mapSubscriber = rospy.Subscriber("/save_signal_maps", Empty, self.map_callback)
        self.map_response = None
        self.obs_response = None
        self.pose = None
        self.signal_maps_dict = dict()
        self.std = [3,3]
        self.spatial_resolution = [1,1]
        self.epoch = int(time.time())
        self.filename = "/home/murat/deep_ws/src/wifi_observation/maps/txt/" + str(self.epoch) + ".txt"

        open(self.filename, "w")

        rospy.loginfo("Ready to acquire the map, location and wifi observations")
        rospy.spin()

    def location_callback(self, data):
        ### TODO: Mutex lock might be required here! ###
        rospy.loginfo("The pose is acquired")
        self.pose = data

    def map_callback(self, data):
        rospy.loginfo("Map saving request was received")
        rospy.loginfo("Dictionary of size: %d", len(self.signal_maps_dict))
        # np.save('/home/murat/deep_ws/maps.npy',self.signal_maps_dict)
        for i in range(len(self.signal_maps_dict)):
            rospy.loginfo("index: %d, key: %s, type: %s", i, self.signal_maps_dict.keys()[i], type(self.signal_maps_dict.values()[i]))
            filename = self.signal_maps_dict.keys()[i].replace(":", "")
            # filename = "/home/murat/deep_ws/src/wifi_observation/maps/" + filename + ".txt"
            filename = "/home/murat/deep_ws/src/wifi_observation/maps/" + filename + ".tiff"
            rospy.loginfo("filename: %s", filename)
            data = np.array(self.signal_maps_dict.values()[i].data, dtype=np.uint8).reshape(self.map_response.map.info.height,self.map_response.map.info.width)
            # np.savetxt(filename, data, fmt="%d")
            cv2.imwrite(filename, data)
            cv2.imshow("Image window", data)
            cv2.waitKey(3)
            # im = Image.fromarray(data)
            # im.save(filename)

    def handle_signal_map(self, data):
        rospy.loginfo("Service request captured")
        if self.map_response is None:
            self.get_elevation_map()

        self.get_wifi_observation()
        # self.maintain_maps()

        return self.construct_response()

    def get_elevation_map(self,):
        rospy.wait_for_service('/static_map')
        getMap = rospy.ServiceProxy('/static_map', GetMap)
        try:
            self.map_response = getMap()
            rospy.loginfo("Map Load Time: %s", str(self.map_response.map.info.map_load_time))
            rospy.loginfo("Map Resolution: %f", self.map_response.map.info.resolution)
            rospy.loginfo("Map Size: %dx%d ", self.map_response.map.info.width, self.map_response.map.info.height)
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: %s" + str(exc))

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
                    str(self.obs_response.observations[i].ssid) + '\t' + str(self.pose.pose.pose.position.x) + '\t' + \
                    str(self.pose.pose.pose.position.y) + '\t' + str(self.pose.pose.covariance) + '\n'
                    text_file.write(line)
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: %s" + str(exc))

    def construct_response(self):
        maps = list()
        publishers = list()
        response = SignalMapArrayServiceResponse()
        response.success = True
        response.message = "Success!"
        for i in self.signal_maps_dict:
            publishers.append(rospy.Publisher(str('/signal_maps/'+ i.replace(":", "")), OccupancyGrid, queue_size=10, latch=True))
            maps.append(self.signal_maps_dict[i])

        ### TODO: Do I really seperate the loops? ###
        for i in range(len(publishers)):
            publishers[i].publish(maps[i])

        response.maps = maps
        return response

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

    def pose_to_grid_pixel(self):
        posx = int((self.pose.pose.pose.position.x-self.map_response.map.info.origin.position.x)/self.map_response.map.info.resolution)
        posy = int((self.pose.pose.pose.position.y-self.map_response.map.info.origin.position.y)/self.map_response.map.info.resolution)
        ## TODO: Raise an exception if resolution is 0
        return np.array([posx,posy])

    def gaussian1D(self, mu, std, size, num):
        x = np.linspace(1, size, num=num)
        return np.exp(-np.power((x-mu),2)/(2*np.power(std,2)))/(std*np.sqrt(np.pi))

    def gaussian2D(self, mean, std, size, res):
        fx = self.gaussian1D(mean[0], std[0], size[0], (size[0]/res[0]))
        fx = fx.reshape((size[0],1))
        fy = self.gaussian1D(mean[1], std[1], size[1], (size[1]/res[1]))
        fy = fy.reshape((size[1],1))
        gs = fx * fy.T
        gs = gs - np.amin(gs)
        gs = gs / np.amax(gs)
        return gs

if __name__ == "__main__":
    wifi_map = wifi_map()
