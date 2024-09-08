#!/usr/bin/env python3

import rospy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import actionlib

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray,Bool
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool,SetBoolRequest

from pose_processor import poseProcessor

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
        
        depth_img_sub = message_filters.Subscriber(self.depth_topic_name,Image, queue_size=1)
        ts_depth = message_filters.TimeSynchronizer([depth_img_sub], 10)
        ts_depth.registerCallback(self.synch_depth_image_callback)
        
        # Setup publishers
        self.shutdown_pub = rospy.Publisher("/shutdown_spice_up",Bool)
        self.pose_debug_pub = rospy.Publisher("/debug_pose", Image, queue_size=1)

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
        


    def get_intrinsics(self):
        intrinsics_topic = "/camera/color/camera_info"
        try:
            data = rospy.wait_for_message(intrinsics_topic, CameraInfo, timeout=10.0)
            K = np.array(data.K).reshape(3, 3).astype(np.float64)
            return K
        except rospy.ROSException:
            rospy.logwarn(f"[PoseDetectorNode]: Failed to get intrinsics from topic '{intrinsics_topic}', retrying...")
            return self.get_intrinsics()

    def synch_col_image_callback(self, color_msg):
        try:
            cv_color_img = self._bridge.imgmsg_to_cv2(color_msg,desired_encoding="bgr8")
            self.last_image_color = cv_color_img
            self.last_image_color_msg = color_msg

        except CvBridgeError as e:
            print(e)

    
    def synch_depth_image_callback(self, depth_msg):
        try:
            cv_depth_img = self._bridge.imgmsg_to_cv2(depth_msg,desired_encoding = "passthrough").astype(np.uint16).copy() / 1000.0
            cv_depth_img[(cv_depth_img < 0.1)] = 0
            self.last_image_depth = cv_depth_img
            self.last_image_depth_msg = depth_msg

        except CvBridgeError as e:
            print(e)
    

    def execute_cb(self, goal): # Goal of type: SpiceUpBottlePickGoal
        
        # publish info to the console for the user
        print("[spiceUpCoordinator] : Received action goal")
        
        result = SpiceUpBottlePickResult()
        result.ee_pickup_target = PoseStamped()
        result.ee_dropoff_target = PoseStamped()

        if goal.activation:       
            if self.drop_off_index == 0:
                # Send request for shelf pose to pose_est_server
                pose_request = ShelfPoseRequest()
                cv_color = self.last_image_color
                cv_depth = self.last_image_depth
                self.color_msg = self.last_image_color_msg
                #depth_msg = self.cv2_to_ros(np.array(cv_depth,dtype="uint8"))
                depth_msg = self.last_image_depth_msg

                pose_request.color_frame = self.color_msg
                pose_request.depth_frame = depth_msg
                pose_service_handle = rospy.ServiceProxy('pose_est_server', ShelfPose)
                print("[spiceUpCoordinator] : Requesting shelf pose")
                pose_service_response = pose_service_handle(pose_request)
                T_ce_msg = pose_service_response.T_ce
                self.mask_msg = pose_service_response.mask
                self.mask_has_five_contours = pose_service_response.has_five_contours
                print("[spiceUpCoordinator] : Received shelf pose: "+str(T_ce_msg))
                self.pp = poseProcessor(T_ce_msg,self.K,cv_color)

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
            idx_request.mask = self.mask_msg
            idx_request.color_frame = self.color_msg
            idx_request.has_five_contours = self.mask_has_five_contours
            idx_acquisition_service_handle = rospy.ServiceProxy('idx_finder_server', IDXAcquisition)
            print("[spiceUpCoordinator] : Requesting target spice IDX")
            idx_acquisition_service_response = idx_acquisition_service_handle(idx_request)
            target_idx = idx_acquisition_service_response.idx
            print("[spiceUpCoordinator] : Received target spice IDX: "+str(target_idx))
            
            # Visualization
            pose_visualized_msg = self.cv2_to_ros(self.pp.get_specific_viz(target_idx,self.drop_off_index))
            #pose_visualized_msg = self.cv2_to_ros(self.pp.pose_visualized_img_all)
            
            #pose_visualized_msg.header.stamp = rospy.Time.now()
            self.pose_debug_pub.publish(pose_visualized_msg)

            # Fill result
            result.ee_pickup_target = self.pp.grasp_msg_dict[target_idx]
            result.ee_dropoff_target = self.pp.drop_off_msg_dict[self.drop_off_index]
            self.drop_off_index += 1 # Next time use other drop off location
            self.action_server.set_succeeded(result)

            # Shutdown
            if self.drop_off_index == 2:
                print("[spiceUpCoordinator] : Sending shutdown request")
                shutdown_msg = Bool()
                shutdown_msg.data = True
                self.shutdown_pub.publish(shutdown_msg)

                print("[spiceUpCoordinator] : Shutting down")
                rospy.signal_shutdown("Job done")
        
    def ros_to_cv2(self, frame: Image, desired_encoding="bgr8"):
        return self._bridge.imgmsg_to_cv2(frame, desired_encoding=desired_encoding)

    def cv2_to_ros(self, frame: np.ndarray):
        return self._bridge.cv2_to_imgmsg(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), encoding="rgb8")

    def load_params(self):
        
        self.pp = None # PoseProcessor instance

        self.K = self.get_intrinsics()

        self.drop_off_index = 0

        self.mask_msg = None
        self.color_msg = None

        self._bridge = CvBridge()

        self.last_image_color = None
        self.last_image_color_msg = None

        self.last_image_depth = None
        self.last_image_depth_msg = None



        self.color_topic_name = "/camera/color/image_raw"
        self.depth_topic_name = "/camera/aligned_depth_to_color/image_raw"
        
        
if __name__ == '__main__':
    rospy.init_node('spice_up_action_server')
    server = spiceUpCoordinator()
    rospy.spin()
