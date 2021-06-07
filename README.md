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
cd 3_scripts
python3 log_to_bag.py ../sd-card-data/UWB-eventlog00
```

3. Run the script to visualize the survey results
```
cd 3_script/survey
python3 anchor_survey.py ../..2_data/survey-results/anchor_0425.txt
```

## Visualize the static signal testing experiments
1. Run the script to visualize the one particular experiments
```
cd 3_scripts/data-process/static-data-process
python3 visual_static_signal.py -i ../../../2_data/rosbag/static-signal-testing/(select the rosbag you wish to visualize)
```

2. Run the script to visualize the LOS line signal testing
```
cd 3_scripts/data-process/static-data-process
python3 visual_los_lineTest.py -i /los_testing
```

3. Run the script to visualize the LOS circle signal testing
```
cd 3_scripts/data-process/static-data-process
python3 visual_los_circleTest.py -i /los_testing
```

## Visualize the data collected during flights
1. Run the script to visualize data with UWB TDOA2 mode
```
cd 3_scripts/data-process/flying-data-process
python3 sensor_visual_tdoa2.py -i ../../survey/anchor_0425.npz ../../../2_data/rosbag/flying-data/(select the rosbag you wish to visualize)
```

2. Run the script to visualize data with UWB TDOA3 mode
```
cd 3_scripts/data-process/flying-data-process
python3 sensor_visual_tdoa3.py -i ../../survey/anchor_0425.npz ../../../2_data/rosbag/flying-data/(select the rosbag you wish to visualize)
```

4. Run the script to visualize UWB bias
```
cd 3_scripts/data-process/flying-data-process
python3 visual_uwb_bias.py -i ../../survey/anchor_0425.npz ../../../2_data/rosbag/flying-data/(select the rosbag you wish to visualize)
```

4. Run the script to visualize the position of obstalces
```
cd 3_scripts/data-process/flying-data-process
python3 visual_obs.py -i ../../survey/anchor_0426.npz
```