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

## Data parsing scripts for flight dataset
2. Convert sdcard binary data to rosbag:
```
$(source the ros workspace)
$cd dataset/scripts/flight-data/sdcard_scripts
$python3 log_to_bag.py (sdcard_data)

e.g.
$python3 log_to_bag.py ../../../flight-dataset/binary-data/const1/const1-log1
```

3. Visualize the survey results:
```
$cd dataset/scripts/survey
$python3 anchor_survey.py (anchor_survey_txt_files)

e.g.
$python3 anchor_survey.py ../../flight-dataset/survey-results/anchor_const1.txt
```

4. Visualize UWB measurements:
```
$cd dataset/scripts/flight-data
$python3 visual_tdoa2.py -i (anchor_survey_npz) (tdoa2_rosbag_data)
$python3 visual_tdoa3.py -i (anchor_survey_npz) (tdoa3_rosbag_data)

e.g.
$python3 visual_tdoa2.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log1.bag 
$python3 visual_tdoa3.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log7.bag 

For TDOA3, the anchor pair of the visualized UWB measurement is set in the script (visual_tdoa3.py).
```

5. Visualize UWB measurement bias:
```
$cd dataset/scripts/flight-data
$python3 visual_bias.py -i (anchor_survey_npz) (tdoa_rosbag_data)

e.g.
$python3 visual_bias.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log1.bag

The anchor pair of the visualized UWB measurement is set in the script (visual_bias.py)
```

6. Visualize trajectory and obstacle positions during manual data collections
```
$cd dataset/scripts/flight-data
$python3 visual_TrajObs.py -i ../survey/anchor_const3.npz (roabag_data)

e.g. 
$python3 visual_TrajObs.py -i ../survey/anchor_const3.npz ../../rosbag/flight-data/rosbag-const3/const3-tdoa2-obs-log1.bag 
```

7. Error-State Kalman Filter Estimation
```
$(source the ros workspace)
$cd dataset/scripts/estimation
$python3 eskf.py -i (num_of_const) (rosbag_data)

e.g.
$python3 eskf.py -i 1 ../../rosbag/flight-data/rosbag-const1/const1-log1.bag
```

## Data parsing scripts for static dataset

8. Visualize LOS static data
```
$cd dataset/scripts/static-data
$python3 los_visual.py -i (los_data_folder)

e.g.
$python3 los_visual.py -i ../../static-dataset/los/distTest/distT1
```

9. Visualize NLOS static data
```
$cd dataset/scripts/static-data
$python3 nlos_visual.py -i (nlos_data_folder)

e.g.
$python3 nlos_visual.py -i ../../static-dataset/nlos/anTag/metal/data1
```
