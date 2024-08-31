#!/usr/bin/env python3

import rospy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import actionlib

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo


from spice_up_coordinator.action._SpiceUpBottlePick import SpiceUpBottlePick,SpiceUpBottlePickResponse
from spice_up_coordinator.srv._SpiceName import SpiceName, SpiceNameRequest
from spice_up_coordinator.srv._IDXAcquisition import IDXAcquisition,IDXAquisitionRequest

class spiceUpCoordinator:
    def __init__(self):
        rospy.init_node('spice_up_action_server')

        self.load_params()

        self.pick_up_pose = None
        self.place_pose = None

        # Possible grasping poses at 4 known locations in Kallax shelf TODO!!
        self.p0 = PoseStamped
        self.p1 = PoseStamped
        self.p2 = PoseStamped
        self.p3 = PoseStamped

        self.poses = {
            0: self.p0,
            1: self.p1,
            2: self.p2,
            3: self.p3
        }

        self.p_first_drop_off = PoseStamped
        self.p_second_drop_off = PoseStamped

        
        # Start action service
        self.action_server = actionlib.SimpleActionServer("spice_up_action_server", SpiceUpBottlePick, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()

        print("[spiceUpCoordinator] : Waiting for spice_name_server...")
        rospy.wait_for_service('spice_name_server')
        
        print("[spiceUpCoordinator] : Waiting for idx_finder_server...")
        rospy.wait_for_service('idx_finder_server')

        rospy.loginfo("[spiceUpCoordinator] : "+str("Initialized"))
        

    def execute_cb(self, goal):
        
        # publish info to the console for the user
        rospy.loginfo("[spiceUpCoordinator] : Received goal")
        
        response = SpiceUpBottlePickResponse()
        response.ee_pickup_target = PoseStamped
        response.ee_dropoff_target = self.p_first_drop_off
        response.debug = "FAIL"

        if goal.activation:       

            # Send request for color_profile to color_profile_server
            spice_name_request = SpiceNameRequest()
            spice_name_service_handle = rospy.ServiceProxy('spice_name_server', SpiceName)
            spice_name_service_response = spice_name_service_handle(spice_name_request)

            target_spice = spice_name_service_response.spice_name

            # Send request for target spice position idx to idx_finder_server
            idx_request = IDXAquisitionRequest()
            idx_request.target_spice = target_spice
            idx_acquisition_service_handle = rospy.ServiceProxy('idx_finder_server', IDXAcquisition)
            idx_acquisition_service_response = idx_acquisition_service_handle(idx_request)

            target_idx = idx_acquisition_service_response.idx

            response.ee_pickup_target = self.poses[target_idx]

            return response
        
if __name__ == '__main__':
    rospy.init_node('spice_up_action_server')
    server = spiceUpCoordinator()
    rospy.spin()
