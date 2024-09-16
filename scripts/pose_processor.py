import numpy as np
import rospy
import trimesh
import cv2
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_matrix

class poseProcessor:
    def __init__(self,T_ce_msg,K,color_frame) -> None:
        
        self.K = K
        self.T_ce = self.read_pose_msg(T_ce_msg)

        self.simple_grasp = True
        self.correct_rot = True


        # Get mesh props
        mesh_file_path = "/home/carmen/ros/catkin_ws/src/spice_up_coordinator/data/kallax_with_backplate_meters.obj"
        self.mesh, self.mesh_props = self.load_mesh(mesh_file_path)
        self.bbox = self.mesh_props["bbox"]
        self.extents = self.mesh_props["extents"]

        # Get shel h,w,d
        self.shelf_depth = self.extents[0]
        self.shelf_height = self.extents[1] # TODO: CHECK IF STILL TRUE FOR SMALLER KALLAX !! 
        self.shelf_width = self.extents[2] # TODO: CHECK IF STILL TRUE FOR SMALLER KALLAX !! 
        
        # Create T_es -> T_cs for viz (not original T_cs, but the one rotationally aligned with E-frame)
        T_es = np.zeros((4,4))
        T_es[0:3,0:3] = np.identity(3)
        T_es[0:3,3] = np.array([self.shelf_depth/2,self.shelf_height/2,self.shelf_width/2])
        T_es[3,3] = 1

        self.T_cs = self.T_ce @ T_es

        self.T_cdo0,self.T_cdo1 = self.get_drop_off_poses()
        self.T_cg0,self.T_cg1,self.T_cg2,self.T_cg3, = self.get_grasp_poses()


        # Create pose msgs
        T_cg0_msg = self.create_pose_msg(self.T_cg0)
        T_cg1_msg = self.create_pose_msg(self.T_cg1)
        T_cg2_msg = self.create_pose_msg(self.T_cg2)
        T_cg3_msg = self.create_pose_msg(self.T_cg3)
        
        T_cdo0_msg = self.create_pose_msg(self.T_cdo0)
        T_cdo1_msg = self.create_pose_msg(self.T_cdo1)

        self.grasp_dict = {
            0: self.T_cg0,
            1: self.T_cg1,
            2: self.T_cg2,
            3: self.T_cg3
        }

        self.grasp_msg_dict = {
            0: T_cg0_msg,
            1: T_cg1_msg,
            2: T_cg2_msg,
            3: T_cg3_msg
        }

        self.drop_off_msg_dict = {
            0: T_cdo0_msg,
            1: T_cdo1_msg
        }

        # Visualizes
        self.color_frame = color_frame
        self.pose_visualized_img_all = self.get_viz_img_all(color_frame,self.T_cs,self.bbox,self.T_cg0,self.T_cg1,self.T_cg2,self.T_cg3,self.T_cdo0,self.T_cdo1)

    def get_specific_viz(self,gidx,didx):
        return self.get_viz_img_specific(self.color_frame,self.T_cs,self.bbox,self.T_cg0,self.T_cg1,self.T_cg2,self.T_cg3,self.T_cdo0,self.T_cdo1,gidx,didx)

    def read_pose_msg(self,pose_msg):

        T = np.zeros((4,4))
        T[0,3] = pose_msg.pose.position.x
        T[1,3] = pose_msg.pose.position.y
        T[2,3] = pose_msg.pose.position.z
        
        quat = [0,0,0,0]
        quat[0] = pose_msg.pose.orientation.x 
        quat[1] = pose_msg.pose.orientation.y
        quat[2] = pose_msg.pose.orientation.z 
        quat[3] = pose_msg.pose.orientation.w

        T[0:3,0:3] = quaternion_matrix(quat)[0:3,0:3] #https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py line 1174
        T[3,3] = 1

        return T

    def create_pose_msg(self,T):
        pose_mat = T.reshape(4, 4)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "dynaarm_REALSENSE_color_optical_frame"
        pose_msg.pose.position.x = pose_mat[0, 3]
        pose_msg.pose.position.y = pose_mat[1, 3]
        pose_msg.pose.position.z = pose_mat[2, 3]
        quat = Rotation.from_matrix(pose_mat[:3, :3]).as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        return pose_msg
    
    def load_mesh(self, mesh_file):
        mesh = trimesh.load(mesh_file, force="mesh")
        mesh_props = dict()
        to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
        bbox = np.stack([-extents / 2, extents / 2], axis=0).reshape(2, 3)
        mesh_props["to_origin"] = to_origin
        mesh_props["bbox"] = bbox
        mesh_props["extents"] = extents
        return mesh, mesh_props

    def get_drop_off_poses(self):

        T_ce = self.T_ce
        
        #Create drop off pose DO0,DO1 ->TODO: drop off position bottom part of bottle or com?
        
        # Create rotation matrix from C frame to standard frame used in alma simulator (?)
        R_esim = np.array([[0,0,1],
                         [0,-1,0],
                         [1,0,0]])

        R_ce = T_ce[0:3,0:3]
        R_do = R_ce @ R_esim


        DO0_E = np.array([self.shelf_depth/2,self.shelf_width*0.75,self.shelf_height,1.0])
        DO1_E = np.array([self.shelf_depth/2,self.shelf_width*0.25,self.shelf_height,1.0])
        
        T_cdo0 = np.zeros((4,4))
        T_cdo0[0:3,0:3] = R_do
        T_cdo0[:,3] = T_ce @ DO0_E
        T_cdo0[3,3] = 1
    
        T_cdo1 = np.zeros((4,4))
        T_cdo1[0:3,0:3] = R_do
        T_cdo1[:,3] = T_ce @ DO1_E
        T_cdo1[3,3] = 1

        return T_cdo0,T_cdo1
    
    def compute_grasp_pose(self,T_em,T_bc,T_ec):
        T_cb = np.linalg.inv(T_bc)
        T_eb = T_ec @ T_cb
        #print("T_eb: "+str(T_eb))

        M_E = T_em[0:3,3]
        B_E = T_eb[0:3,3]
        BM_r_E = M_E - B_E
        BM_r_E_unit = BM_r_E / np.linalg.norm(BM_r_E)

        gamma_E = np.array([B_E[0],B_E[1],M_E[2]]) # point on height of grasp positions below frame (used to make grasp tf orthonormal)
        gammaM_r_E = M_E - gamma_E
        gammaM_r_E_unit = gammaM_r_E / np.linalg.norm(gammaM_r_E)
        normal_to_gammaG_in_ez_plane_E = np.array([-gammaM_r_E_unit[1],gammaM_r_E_unit[0],0])
        
        G_zaxis_E = BM_r_E_unit
        G_yaxis_E = normal_to_gammaG_in_ez_plane_E
        G_xaxis_E = np.cross(G_zaxis_E,G_yaxis_E)

        R_eg = np.array([G_xaxis_E,G_yaxis_E,G_zaxis_E])

        T_eg = np.zeros((4,4))
        T_eg[0:3,0:3] = R_eg
        T_eg[0:3,3] = M_E
        T_eg[3,3] = 1

        if False:
            print("M_e: "+str(M_E))
            print("B_E: "+str(B_E))
            print("BM_r_E: "+str(BM_r_E))
            print("gamma_E: "+str(gamma_E))
            print("normal_to_gammaG_in_ez_plane_E: "+str(normal_to_gammaG_in_ez_plane_E))




        return T_eg

    def get_grasp_poses(self):

        T_ce = self.T_ce
        T_ec = np.linalg.inv(T_ce)

        # CONSTRUCT GRASPING POSES in C-frame
        EP0_r_E = np.array([0.040,0.285,0.392,1.0])
        EP1_r_E = np.array([0.040,0.125,0.392,1.0])
        EP2_r_E = np.array([0.035,0.280,0.042,1.0])
        EP3_r_E = np.array([0.035,0.130,0.042,1.0])
        z_off = 0.03
        EM0_r_E = EP0_r_E + np.array([0,0,z_off,0.0])
        EM1_r_E = EP1_r_E + np.array([0,0,z_off,0.0])
        EM2_r_E = EP2_r_E + np.array([0,0,z_off,0.0])
        EM3_r_E = EP3_r_E + np.array([0,0,z_off,0.0])

        T_cb = np.array([[1,0,0,0], # fake base transform --> TODO Get from tf pub?
                         [0,1,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])
        T_bc = np.linalg.inv(T_cb)

        # Construct transforms from E t0 m0,m1,m2,m3 (bottles middle points))
        T_em0 = np.zeros((4,4))
        T_em0[0:3,0:3] = np.identity(3)
        T_em0[0:4,3] = EM0_r_E
        T_em0[3,3] = 1

        T_em1 = np.zeros((4,4))
        T_em1[0:3,0:3] = np.identity(3)
        T_em1[0:4,3] = EM1_r_E
        T_em1[3,3] = 1

        T_em2 = np.zeros((4,4))
        T_em2[0:3,0:3] = np.identity(3)
        T_em2[0:4,3] = EM2_r_E
        T_em2[3,3] = 1

        T_em3 = np.zeros((4,4))
        T_em3[0:3,0:3] = np.identity(3)
        T_em3[0:4,3] = EM3_r_E
        T_em3[3,3] = 1

        # Construct pick up grasp poses
        if self.simple_grasp:
            T_eg0 = T_em0
            T_eg1 = T_em1
            T_eg2 = T_em2
            T_eg3 = T_em3
        else:     
            T_eg0 = self.compute_grasp_pose(T_em0,T_bc,T_ec)
            T_eg1 = self.compute_grasp_pose(T_em1,T_bc,T_ec)
            T_eg2 = self.compute_grasp_pose(T_em2,T_bc,T_ec)
            T_eg3 = self.compute_grasp_pose(T_em3,T_bc,T_ec)

        # Bring into camera frame
        T_cg0 = T_ce @ T_eg0  
        T_cg1 = T_ce @ T_eg1  
        T_cg2 = T_ce @ T_eg2  
        T_cg3 = T_ce @ T_eg3  

        if self.correct_rot:
            # Construct transforms from C to GRASP_i
            T_gsim = np.array([[0, 0, 1,0],
                               [0,-1, 0,0],
                               [1, 0, 0,0],
                               [0, 0, 0,1]])
            
            T_cg0 = T_cg0 @ T_gsim 
            T_cg1 = T_cg1 @ T_gsim 
            T_cg2 = T_cg2 @ T_gsim 
            T_cg3 = T_cg3 @ T_gsim 

        return T_cg0,T_cg1,T_cg2,T_cg3
    
    def project_3d_to_2d(self,pt,K,ob_in_cam):
        pt = pt.reshape(4,1)
        projected = K @ ((ob_in_cam@pt)[:3,:])
        projected = projected.reshape(-1)
        projected = projected/projected[2]
        return projected.reshape(-1)[:2].round().astype(int)

    def draw_posed_3d_box(self,K, img, ob_in_cam, bbox, line_color=(0,255,0), linewidth=2):
        '''Revised from 6pack dataset/inference_dataset_nocs.py::projection
        @bbox: (2,3) min/max
        @line_color: RGB
        '''
        min_xyz = bbox.min(axis=0)
        xmin, ymin, zmin = min_xyz
        max_xyz = bbox.max(axis=0)
        xmax, ymax, zmax = max_xyz

        for y in [ymin,ymax]:
            for z in [zmin,zmax]:
                start = np.array([xmin,y,z])
                end = start+np.array([xmax-xmin,0,0])
                img = self.draw_line3d(start,end,img,ob_in_cam,K,line_color,linewidth)

        for x in [xmin,xmax]:
            for z in [zmin,zmax]:
                start = np.array([x,ymin,z])
                end = start+np.array([0,ymax-ymin,0])
                img = self.draw_line3d(start,end,img,ob_in_cam,K,line_color,linewidth)

        for x in [xmin,xmax]:
            for y in [ymin,ymax]:
                start = np.array([x,y,zmin])
                end = start+np.array([0,0,zmax-zmin])
                img = self.draw_line3d(start,end,img,ob_in_cam,K,line_color,linewidth)

        return img

    def draw_xyz_axis(self,color, ob_in_cam, scale=0.1, K=np.eye(3), thickness=3, transparency=0,is_input_rgb=False):
        if is_input_rgb:
            color = cv2.cvtColor(color,cv2.COLOR_RGB2BGR)
        xx = np.array([1,0,0,1]).astype(float)
        yy = np.array([0,1,0,1]).astype(float)
        zz = np.array([0,0,1,1]).astype(float)
        xx[:3] = xx[:3]*scale
        yy[:3] = yy[:3]*scale
        zz[:3] = zz[:3]*scale
        origin = tuple(self.project_3d_to_2d(np.array([0,0,0,1]), K, ob_in_cam))
        xx = tuple(self.project_3d_to_2d(xx, K, ob_in_cam))
        yy = tuple(self.project_3d_to_2d(yy, K, ob_in_cam))
        zz = tuple(self.project_3d_to_2d(zz, K, ob_in_cam))
        line_type = cv2.LINE_AA
        arrow_len = 0
        tmp = color.copy()
        tmp1 = tmp.copy()
        tmp1 = cv2.arrowedLine(tmp1, origin, xx, color=(255,0,0), thickness=thickness,line_type=line_type, tipLength=arrow_len)
        mask = np.linalg.norm(tmp1-tmp, axis=-1)>0
        tmp[mask] = tmp[mask]*transparency + tmp1[mask]*(1-transparency)
        tmp1 = tmp.copy()
        tmp1 = cv2.arrowedLine(tmp1, origin, yy, color=(0,255,0), thickness=thickness,line_type=line_type, tipLength=arrow_len)
        mask = np.linalg.norm(tmp1-tmp, axis=-1)>0
        tmp[mask] = tmp[mask]*transparency + tmp1[mask]*(1-transparency)
        tmp1 = tmp.copy()
        tmp1 = cv2.arrowedLine(tmp1, origin, zz, color=(0,0,255), thickness=thickness,line_type=line_type, tipLength=arrow_len)
        mask = np.linalg.norm(tmp1-tmp, axis=-1)>0
        tmp[mask] = tmp[mask]*transparency + tmp1[mask]*(1-transparency)
        tmp = tmp.astype(np.uint8)
        if is_input_rgb:
            tmp = cv2.cvtColor(tmp,cv2.COLOR_BGR2RGB)
        return tmp

    def to_homo(self,pts):
        '''
        @pts: (N,3 or 2) will homogeneliaze the last dimension
        '''
        assert len(pts.shape)==2, f'pts.shape: {pts.shape}'
        homo = np.concatenate((pts, np.ones((pts.shape[0],1))),axis=-1)
        return homo

    def draw_line3d(self,start,end,img,ob_in_cam,K,line_color,linewidth):
        pts = np.stack((start,end),axis=0).reshape(-1,3)
        pts = (ob_in_cam@self.to_homo(pts).T).T[:,:3]   #(2,3)
        projected = (K@pts.T).T
        uv = np.round(projected[:,:2]/projected[:,2].reshape(-1,1)).astype(int)   #(2,2)
        img = cv2.line(img, uv[0].tolist(), uv[1].tolist(), color=line_color, thickness=linewidth, lineType=cv2.LINE_AA)
        return img

    def get_viz_img_all(self,color,T_cs,bbox,T_cg0,T_cg1,T_cg2,T_cg3,T_cdo0,T_cdo1):
        
        # Draw bbox and T_CA coord axes
        pose_visualized = self.draw_posed_3d_box(self.K, img=color, ob_in_cam=T_cs, bbox=bbox) # 
        pose_visualized = self.draw_xyz_axis(color,ob_in_cam=T_cs,scale=0.1,K=self.K,thickness=3, transparency=0,is_input_rgb=True) 
        
        # Draw grasp poses 
        pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cg0,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cg1,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cg2,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cg3,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)

        # Draw drop off poses
        pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cdo0,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cdo1,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)

        return pose_visualized

    def get_viz_img_specific(self,color,T_cs,bbox,T_cg0,T_cg1,T_cg2,T_cg3,T_cdo0,T_cdo1,target_spice_loc_idx,target_dropoff_idx):
        
        # Draw bbox and T_CA coord axes
        pose_visualized = self.draw_posed_3d_box(self.K, img=color, ob_in_cam=T_cs, bbox=bbox) # 
        #pose_visualized = self.draw_xyz_axis(color,ob_in_cam=T_cs,scale=0.1,K=self.K,thickness=3, transparency=0,is_input_rgb=True) 
        
        # Draw grasp poses 
        if target_spice_loc_idx == 0:
            pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cg0,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        elif target_spice_loc_idx == 1:
            pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cg1,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        elif target_spice_loc_idx == 2:
            pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cg2,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        elif target_spice_loc_idx == 3:
            pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cg3,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        else:
            print("ERROR: invalid target_spice_loc_idx")
            return None

        # Draw drop off poses
        if target_dropoff_idx == 0:
            pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cdo0,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        elif target_dropoff_idx == 1:
            pose_visualized = self.draw_xyz_axis(pose_visualized,ob_in_cam=T_cdo1,scale=0.05,K=self.K,thickness=2,transparency=0,is_input_rgb=True)
        else:
            print("ERROR: invalid target_dropoff_idx")
            return None

        return pose_visualized

