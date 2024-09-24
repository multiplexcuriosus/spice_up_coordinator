# The spice_up_coordinator node
## Overview
The spice_up_coordinator is the interface between the spiceUpAutomationModule and the following nodes:
* `idx_finder`
* `foundation_pose_ros`
* `spice_selectioin_gui`
## Control flow
![alt text](https://github.com/multiplexcuriosus/spice_up_coordinator/ReadMe_imgs/master/spice_up_nodes.png?raw=true)


## Terminal setup
### Pythonpath + source venv
Doing `roslaunch spice_up_coordinator all.launch` on the jetson results in an error saying that certain sub-modules (e.g poseProcessor.py) cannot be found. To fix this do the following on the jetson:
```
export PYTHONPATH=$PYTHONPATH:/home/jau/ros/catkin_ws/src/idx_finder/scripts/  
export PYTHONPATH=$PYTHONPATH:/home/jau/ros/catkin_ws/src/spice_up_coordinator/scripts/  
export PYTHONPATH=$PYTHONPATH:/home/jau/spice_up/lib/python3.8/site-packages/

```
To reset the PYTHONPATH to what it was prior to this operation run:
```
export PYTHONPATH=/home/jau/ros/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
```

All packages required to run launch the `idx_finder` & `spice_up_coordinator` nodes with the `all.launch` launch-file are either already installed or are contained in the `spice_up`-venv on the jetson. To source it run:
```
source /home/jau/spice_up/bin/activate  
```

