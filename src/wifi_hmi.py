#!/usr/bin/env python
from threading import Thread, Lock

import sys
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from wifi_observation.srv import SignalMapArrayService
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def callback(data):
    rospy.loginfo("Request captured, service is being called")
    rospy.wait_for_service('/signal_map')
    try:
        get_signal_map_array = rospy.ServiceProxy('/signal_map', SignalMapArrayService)
        resp = get_signal_map_array()
        print resp.success, resp.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def joy_callback(data):
    global sub
    sub.unregister()
    lock.acquire()
    if data.buttons[15] == 1:
        print "Signal Request", data.buttons[15]
        ## Observation Request
        rospy.loginfo("Request captured, service is being called")
        voice_feedback()
        rospy.wait_for_service('/signal_map')
        try:
            get_signal_map_array = rospy.ServiceProxy('/signal_map', SignalMapArrayService)
            resp = get_signal_map_array()
            # print resp.success, resp.message
            # else:
            #     voice_feedback("Error in service call")
        except rospy.ServiceException, e:
            # voice_feedback("Service exception")
            print "Service call failed: %s"%e
        finally:
            if(resp.success):
                voice_feedback()
            lock.release()
    elif data.buttons[13] == 1:
        print "Save Request", data.buttons[13]

        ## Map Save
        save_request = Empty()
        try:
            pub.publish(save_request)
            voice_feedback()
        except:
            rospy.loginfo("Exception caught")
        finally:
            voice_feedback()
            lock.release()
    else:
        lock.release()
    sub = rospy.Subscriber("/joy", Joy, joy_callback)

def voice_feedback():
    # s= soundhandle.playWave("say-beep.wav")
    # s3 = soundhandle.voiceSound("Testing the new A P I")
    soundhandle.play(SoundRequest.NEEDS_PLUGGING)
    # s = soundhandle.voiceSound(input)
    # s.play()
    # rospy.sleep(2)
    # s.stop()


if __name__ == "__main__":

    lock = Lock()
    rospy.init_node('wifi_hmi', anonymous=True)
    # rospy.Subscriber("/wifi_hmi", Empty, callback)
    sub = rospy.Subscriber("/joy", Joy, joy_callback)
    pub = rospy.Publisher("/save_signal_maps", Empty, queue_size=None)
    soundhandle = SoundClient()
    soundhandle.stopAll()
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     message = rospy.wait_for_message("/joy", Joy)
    #     rospy.loginfo("Data %s", message)
    #     rate.sleep()

    rospy.spin()
