# The spice_up_coordinator node
## Overview
The spice_up_coordinator is the interface between the spiceUpAutomationModule and the following nodes:
* `idx_finder`
* `foundation_pose_ros`
* `spice_selectioin_gui`
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

## Terminal setup
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

