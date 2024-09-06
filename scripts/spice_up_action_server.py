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
from spice_up_coordinator.srv._ShelfPose import ShelfPose, ShelfPoseRequest,ShelfPoseResponse

class spiceUpCoordinator:
    def __init__(self):
        rospy.init_node('spice_up_action_server')

        self.load_params()

        # Set up callbacks
        color_img_sub = message_filters.Subscriber(self.color_topic_name,Image, queue_size=1)
        ts_col = message_filters.TimeSynchronizer([color_img_sub], 10)
        ts_col.registerCallback(self.synch_col_image_callback)
        self.last_image_color = None

        depth_img_sub = message_filters.Subscriber(self.depth_topic_name,Image, queue_size=1)
        ts_depth = message_filters.TimeSynchronizer([depth_img_sub], 10)
        ts_depth.registerCallback(self.synch_depth_image_callback)
        self.last_image_depth = None


        
        # Start action service
        self.action_server = actionlib.SimpleActionServer("spice_up_action_server", SpiceUpBottlePickAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_server.start()


        print("[spiceUpCoordinator] : Waiting for spice_name_server...")
        rospy.wait_for_service('spice_name_server')
        print("[spiceUpCoordinator] : spice_name_server found")

        print("[spiceUpCoordinator] : Waiting for idx_finder_server...")
        rospy.wait_for_service('idx_finder_server')
        print("[spiceUpCoordinator] : idx_finder_server found")

        print("[spiceUpCoordinator] : Waiting for pose_est_server...")
        rospy.wait_for_service('pose_est_server')
        print("[spiceUpCoordinator] : pose_est_server found")

        print("[spiceUpCoordinator] : "+str("Initialized"))
        

    def synch_col_image_callback(self, color_msg):
        try:
            cv_color_img = self._bridge.imgmsg_to_cv2(color_msg,desired_encoding="bgr8").copy()
            self.last_image_color = cv_color_img

        except CvBridgeError as e:
            print(e)

    
    def synch_depth_image_callback(self, depth_msg):
        try:
            cv_depth_img = self._bridge.imgmsg_to_cv2(depth_msg,desired_encoding = "passthrough").astype(np.uint16).copy() / 1000.0
            cv_depth_img[(cv_depth_img < 0.1)] = 0
            self.last_image_depth = cv_depth_img

        except CvBridgeError as e:
            print(e)
    

    def execute_cb(self, goal): # Goal of type: SpiceUpBottlePickGoal
        
        # publish info to the console for the user
        print("[spiceUpCoordinator] : Received goal")
        
        result = SpiceUpBottlePickResult()
        result.ee_pickup_target = PoseStamped()

        result.ee_dropoff_target.header.stamp = rospy.Time.now()
        result.ee_dropoff_target = self.p_first_drop_off

        if goal.activation:       
            
            # Send request for shelf pose to pose_est_server
            pose_request = ShelfPoseRequest()
            #color = cv2.imread("/home/jau/ros/catkin_ws/src/idx_finder/scripts/last_voc0.png")  
            #depth = cv2.imread("/home/jau/ros/catkin_ws/src/idx_finder/scripts/last_voc1.png")
            cv_color = self.last_image_color
            cv_depth = self.last_image_depth
            color_msg = self.cv2_to_ros(cv_color)
            depth_msg = self.cv2_to_ros(np.array(cv_depth,dtype="uint8"))
            
            pose_request.color_frame = color_msg
            pose_request.depth_frame = depth_msg
            pose_service_handle = rospy.ServiceProxy('pose_est_server', ShelfPose)
            print("[spiceUpCoordinator] : Requesting shelf pose")
            pose_service_response = pose_service_handle(pose_request)

            T_ce = pose_service_response.T_ce
            mask_msg = pose_service_response.mask
            print("[spiceUpCoordinator] : Received shelf pose: "+str(T_ce))

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
            #print("idx request mask shape: "+str(mask.shape))
            #print("idx request color shape: "+str(color.shape))

            idx_request.mask = mask_msg
            idx_request.color_frame = color_msg
            idx_acquisition_service_handle = rospy.ServiceProxy('idx_finder_server', IDXAcquisition)
            print("[spiceUpCoordinator] : Requesting target spice IDX")
            idx_acquisition_service_response = idx_acquisition_service_handle(idx_request)

            target_idx = idx_acquisition_service_response.idx
            print("[spiceUpCoordinator] : Received target spice IDX: "+str(target_idx))

            result.ee_pickup_target = self.poses[target_idx]

            self.action_server.set_succeeded(result)
    
    def ros_to_cv2(self, frame: Image, desired_encoding="bgr8"):
        return self._bridge.imgmsg_to_cv2(frame, desired_encoding=desired_encoding)

    def cv2_to_ros(self, frame: np.ndarray):
        return self._bridge.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), encoding="rgb8")

    def load_params(self):

        self._bridge = CvBridge()

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

        self.color_topic_name = "/camera/color/image_raw"
        self.depth_topic_name = "/camera/aligned_depth_to_color/image_raw"
        
        
if __name__ == '__main__':
    rospy.init_node('spice_up_action_server')
    server = spiceUpCoordinator()
    rospy.spin()
