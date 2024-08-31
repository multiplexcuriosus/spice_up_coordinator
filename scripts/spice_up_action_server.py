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

from spice_up_coordinator.msg import SpiceUpBottlePickAction,SpiceUpBottlePickResult
from spice_up_coordinator.srv._SpiceName import SpiceName, SpiceNameRequest
from spice_up_coordinator.srv._IDXAcquisition import IDXAcquisition,IDXAcquisitionRequest

class spiceUpCoordinator:
    def __init__(self):
        rospy.init_node('spice_up_action_server')

        self.load_params()
        
        # Start action service
        self.action_server = actionlib.SimpleActionServer("spice_up_action_server", SpiceUpBottlePickAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()

        print("[spiceUpCoordinator] : Waiting for spice_name_server...")
        rospy.wait_for_service('spice_name_server')
        
        print("[spiceUpCoordinator] : Waiting for idx_finder_server...")
        rospy.wait_for_service('idx_finder_server')

        print("[spiceUpCoordinator] : "+str("Initialized"))
        

    def execute_cb(self, goal): # Goal of type: SpiceUpBottlePickGoal
        
        # publish info to the console for the user
        print("[spiceUpCoordinator] : Received goal")
        
        result = SpiceUpBottlePickResult()
        result.ee_pickup_target = PoseStamped()
        result.ee_dropoff_target.header.stamp = rospy.Time.now()
        result.ee_dropoff_target = self.p_first_drop_off

        if goal.activation:       

            # Send request for color_profile to color_profile_server
            spice_name_request = SpiceNameRequest()
            spice_name_service_handle = rospy.ServiceProxy('spice_name_server', SpiceName)
            print("[spiceUpCoordinator] : Requesting target spice name")
            spice_name_service_response = spice_name_service_handle(spice_name_request)
            
            target_spice = spice_name_service_response.spice_name
            print("[spiceUpCoordinator] : Received target spice name: "+str(target_spice))

            # Send request for target spice position idx to idx_finder_server
            idx_request = IDXAcquisitionRequest()
            idx_request.target_spice = target_spice
            idx_acquisition_service_handle = rospy.ServiceProxy('idx_finder_server', IDXAcquisition)
            print("[spiceUpCoordinator] : Requesting target spice IDX")
            idx_acquisition_service_response = idx_acquisition_service_handle(idx_request)

            target_idx = idx_acquisition_service_response.idx
            print("[spiceUpCoordinator] : Received target spice IDX: "+str(target_idx))

            result.ee_pickup_target = self.poses[target_idx]

            self.action_server.set_succeeded(result)
    
    def load_params(self):
        self.pick_up_pose = None
        self.place_pose = None

        # Possible grasping poses at 4 known locations in Kallax shelf TODO!!
        self.p0 = PoseStamped()
        self.p0.header.stamp = rospy.Time.now()

        self.p1 = PoseStamped()
        self.p1.header.stamp = rospy.Time.now()

        self.p2 = PoseStamped()
        self.p2.header.stamp = rospy.Time.now()
        
        self.p3 = PoseStamped()
        self.p3.header.stamp = rospy.Time.now()

        self.poses = {
            0: self.p0,
            1: self.p1,
            2: self.p2,
            3: self.p3
        }

        self.p_first_drop_off = PoseStamped()
        self.p_first_drop_off.header.stamp = rospy.Time.now()

        self.p_second_drop_off = PoseStamped()
        self.p_second_drop_off.header.stamp = rospy.Time.now()
        
        
if __name__ == '__main__':
    rospy.init_node('spice_up_action_server')
    server = spiceUpCoordinator()
    rospy.spin()
