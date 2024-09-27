# The spice_up_coordinator node
This ReadMe is structured into:
* **Installation** (spice_up_coordinator + idx_finder)
* **Configuration** (spice_up_coordinator + idx_finder)
* **Setup** (spice_up_coordinator + idx_finder)
* **Launch** (spice_up_coordinator + idx_finder)
* **Testing** 
* **Overview** (SpiceUp Ros Network)
* **PoseProcessor**

## Installation

### ROS & catkin_ws setup on jetson
* Go to [here](https://bitbucket.org/leggedrobotics/anymal_rsl/wiki/Simulation%20PC%20and%20OPC%20Setup%20-%20Release%20%2223.04%22) and do: Setup release: Steps 1-2.5.2 (not sure if necessary)
* Go to [here](https://bitbucket.org/leggedrobotics/alma_rsl/src/main/) and do everything up to step 3.1 (for step 3.1, use sim) (not sure if necessary).  

### Jetson
1. Clone the spice_up_coordinator and idx_finder package into the jetson catkin_ws/src:
```
git clone https://github.com/multiplexcuriosus/spice_up_coordinator.git
git clone https://github.com/multiplexcuriosus/idx_finder.git
```
2. Build them:
```
catkin build spice_up_coordinator 
catkin build idx_finder
```
3. Create venv & install requirements
```
python -m venv /path/to/new/virtual/environment
source /path/to/new/virtual/environment/bin/activate
cd ~/ros/catkin_ws/src/spice_up_coordinator/
pip install -r requirements.txt
```


## Configuration
### Params  
In `idx_finder/config/index_finder.yaml`: Set `index_finder/HOME`  
In `foundation_pose_ros/config/pose_detector.yaml`: 
* Set `spice_up_coordinator/mesh_file`
* Set `spice_up_coordinator/username`

## Setup

### PYTHONPATH 
In order for the submodules of the idx_finder and the spice_up_coordinator to be available when launch the nodes, the following has to be done:
```
export PYTHONPATH=$PYTHONPATH:/home/<username>/ros/catkin_ws/src/idx_finder/scripts/  
export PYTHONPATH=$PYTHONPATH:/home/<username>/ros/catkin_ws/src/spice_up_coordinator/scripts/  
```
### Venv
For the packages installed in the above mentioned venv, the venv must be added to the PYTHONPATH with:  
```
export PYTHONPATH=$PYTHONPATH:/home/<username>/spice_up/lib/python3.8/site-packages/
```
To reset the PYTHONPATH to what it was prior to this operation run:
```
export PYTHONPATH=/home/<username>/ros/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
```

## Launch 
There is only one launch file: `all.launch`.   
It launches the following parameter files:  
* `index_finder.yaml`
* `spice_up_coordinator.yaml`  <br />

And the following nodes:  
* `idx_finder`
* `spice_up_coordinator`  <br />

Specifically, the following services are launched:  
* `index_finder_server` 
* `spice_up_action_server`   

Launch with:
```
roslaunch foundation_pose_ros all.launch
```

### Quick start: Existing venv on Carmen's account
All packages required to run launch the `idx_finder` & `spice_up_coordinator` nodes with the `all.launch` launch-file are either already installed on the jetson or are contained in the `spice_up`-venv on the jetson (carmen's account). To source it run:
```
source /home/<username>/spice_up/bin/activate  
```

The `spice_up`-venv contains the following packages:
* trimesh
* easyocr

## Testing
The whole pipeline (as described by steps 1-8 under Information flow) can be tested by running:
```
rosrun spice_up_coordinator spice_up_action_client.py
```
* This will send an actionGoal to the spice_up_coordinator node
* This requires color-imgs,depth-imgs and intrinsics-infos to be published in the ros-network. Ideally on the same machine as the spice_up_coordinator is running, in order to keep the streams from congesting the network.

## SpiceUp Ros Network Overview
The spice_up_coordinator is the interface between the spiceUpAutomationModule and the following nodes:
* `idx_finder`
* `foundation_pose_ros`
* `spice_selection_gui`

Additionally, the spice_up_coordinator contains the poseProcessor module, which creates the E-Frame (defined below) and generates the candidate grasp and drop off poses.
## Information flow
This diagram depicts the relation between the spice_up_coordinator and the mentioned nodes.
![spice_up_nodes](https://github.com/user-attachments/assets/94ca1baa-e273-4804-a574-ece3452ac3f9)
The numbers indicate the sequence of the events and the colors the nodes which are either requesting or responding.
The activity indicated at one of an arrow always stems from the node at the other end of the error. 

## PoseProcessor
### E-Frame
There are four possible orientations of the CAD model of the wide shelf which are equally likely to be found by foundationpose's pose estimate.
For the narrow shelf it will be two possible rotations. 

In order to have a reliable reference coordinate frame, the following is done.
<img src="https://github.com/user-attachments/assets/d405b2dc-6dc7-411d-877c-1d651fd5fcca" width=50% height=50%>

1. From the returned pose estimate, all 3D corner positions are identified.
2. Based on the C-frame x-axis, all corners are separated into a left and a right group (blue and red in the image above)
3. For the L-group, the closest corner to the camera is defined as H-corner, the farthest the J-corner. Analogously for the right group with F and G
4. For the L-group A is defined as the corner in {A,D} which is closest to H and D as the other one from {A,D}. Analogously for the R-group with E & I. Now all corners from the the initial pose estimate are well defined.
5. The E-frame is defined as follows:
* X-Axis: unit_vector_EF
* Y-Axis: unit_vector_ED
* Z-Axis: unit_vector_EG

 ### Aligning grasp poses with B-frame
 Currently the rotation of the four graspposes is the same as in E-frame. 
 This is not ideal, it would be better if the z-axis of the grasp pose was colinear with the vector connecting the anymal base-frame and the G-frame.
 
 **Note: In the poseprocessor, there is an additional frame called M-frame**. More specifically, there are four M-frames (M0,M1,M2,M3), one for each bottle.
 The M_i - pose corresponds to the center of the contact cicle between each spice bottles bottom plane and the shelf. I.e the center point on which each bottle stands. The rotation of all M_i frames is the same as the E-frame.

 I propose the following solution to achieve the alignment:
![sa_slide_extraction-9](https://github.com/user-attachments/assets/8a267513-53c5-4066-97de-b190edb70b41)
Legend:
* C_E: camera position in E-frame
* B_E: anymal base-frame position in E-frame
* G_E: grasp-pose position in E-frame
* gamma_E: point directly below B_E on height of grasp position,i.e gamme_E = [B_E.x,B_E.y,G_E.z]

Algorithm:
1. Construct gamma_E as specified above
2. Let BG_r_E = G_E - B_E
3. Let BG_r_E_unit = normalize(BG_r_E)
4. Let gammaG_r_E = G_E - gamma_E
5. Let gammaG_r_E_unit = normalize(gammaG_r_E)
6. Consruct n such that it is normal to gammaG_r_E_unit,normalized and parallel to the floor (E.x,E.y plane), by letting n = [-gammaG_r_E_unit.y,gammaG_r_E_unit.x,0]
7. Let zeta = BG_r_E_unit x n
8. The rotation of the grasp pose G is now G_rot = [BG_r_E,n,zeta]

This algorithm is implemented in the poseProcessor under `compute_grasp_pose_aligned_with_base_frame` but was not yet tested since the grasp poses it produces look wrong. 

Note that the resulting grasp pose orientation is not yet aligned with the one from the gripper in the simulation.
 
