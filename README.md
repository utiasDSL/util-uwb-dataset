# dsl__projects__uwbDataset
The git repository for UWB dataset collection and processing.

## Procedure
1. Build related ROS messages:
```
$cd ros_ws/src
$catkin_init_workspace
$cd ..
$catkin_make
$source devel/setup.bash
```

2. Run script to convert sd log to rosbag: 
```
cd scripts
python3 log_to_bag.py ../sd-card-data/UWB-eventlog00
```
