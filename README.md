# The spice_up_coordinator node
## Overview
The spice_up_coordinator is the interface between the spiceUpAutomationModule and the following nodes:
* `idx_finder`
* `foundation_pose_ros`
* `spice_selection_gui`

Additionally, the spice_up_coordinator contains the poseProcessor module, which creates the E-Frame (defined below) and generates the candidate grasp and drop off poses.
## Information flow
This diagram depicts the relation between the spice_up_coordinator and the mentioned nodes.
![spice_up_nodes](https://github.com/user-attachments/assets/94ca1baa-e273-4804-a574-ece3452ac3f9)
The numbers indicate the sequence of the events and the colors the nodes which are either requesting or responding.

## Launch files
There is only one launch file: `all.launch`.   
It launches the following parameter files:  
* `index_finder.yaml`
* `spice_up_coordinator.yaml`  <br />

And the following nodes:  
* `index_finder`
* `spice_up_coordination`  <br />

Specifically, the following services are launched:  
* `idx_finder_server` 
* `spice_up_action_server` 

## Installation
### Jetson
1. Git clone the following repos into the jetson catkin_ws/src:
* spice_up_coordinator (this repo): `git clone https://github.com/multiplexcuriosus/spice_up_coordinator.git`
* idx_finder: `git clone https://github.com/multiplexcuriosus/idx_finder.git`
2. catkin build `spice_up_coordinator` and `idx_finder`
3. VENV
* Create a new venv with `python -m venv /path/to/new/virtual/environment`.
* Source it with `source /path/to/new/virtual/environment/bin/activate `
* Install requirements file with `pip install requirements.txt`
### OPC
1. Git clone the following repos into the opc catkin_ws/src:
* foundation_pose_ros: `git clone https://github.com/multiplexcuriosus/foundationpose_ros.git`
* spice_selection_gui: `git clone https://github.com/multiplexcuriosus/spice_selection_gui.git`
2. Go to [here](https://bitbucket.org/leggedrobotics/anymal_rsl/wiki/Simulation%20PC%20and%20OPC%20Setup%20-%20Release%20%2223.04%22) and do the steps:
  * Setup release: Steps 1-2.5.2 (not sure if necessary)
3. Go to [here]() and do everything up to step 3.1 (for step 3.1, use sim)
  
  Follow the instructions [here](https://github.com/leggedrobotics/foundation_pose_ros?tab=readme-ov-filea) to setup the leggedrobotics foundationpose ros wrapper.
3. 


## Setup
### Pythonpath + source venv
Doing `roslaunch spice_up_coordinator all.launch` on the jetson results in an error saying that certain sub-modules (e.g poseProcessor.py) cannot be found. To fix this do the following on the jetson:
```
export PYTHONPATH=$PYTHONPATH:/home/<username>/ros/catkin_ws/src/idx_finder/scripts/  
export PYTHONPATH=$PYTHONPATH:/home/<username>/ros/catkin_ws/src/spice_up_coordinator/scripts/  
export PYTHONPATH=$PYTHONPATH:/home/<username>/spice_up/lib/python3.8/site-packages/

```
To reset the PYTHONPATH to what it was prior to this operation run:
```
export PYTHONPATH=/home/jau/ros/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
```

All packages required to run launch the `idx_finder` & `spice_up_coordinator` nodes with the `all.launch` launch-file are either already installed or are contained in the `spice_up`-venv on the jetson. To source it run:
```
source /home/jau/spice_up/bin/activate  
```
The `spice_up`-venv contains the following packages:
* trimesh
* easyocr

## PoseProcessor
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

