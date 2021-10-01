<img src="files/readme_images/fig1.png" alt="start-img" width="1000">

---
## Summary
The UTIAS ultra-wideband (UWB) time-difference-of-arrival (TDOA) consists of low-level signal information from static experiments and UWB TDOA measurements and additional onboard sensor data from flight experiments on a quadrotor. We hope this dataset can help researchers develop and compare reliable estimation methods for emerging UWB TDOA-based indoor localization technology. 

---
## Static Dataset

<img src="files/readme_images/static-los.png" alt="static setup" width="1000"> 


![image](files/readme_images/static-nlos.png){: .img-left}

The UTIAS ultra-wideband (UWB) time-difference-of-arrival (TDOA) consists of low-level signal information from static experiments and UWB TDOA measurements and additional onboard sensor data from flight experiments on a quadrotor. We hope this dataset can help researchers develop and compare reliable estimation methods for emerging UWB TDOA-based indoor localization technology. 

---
## Procedure
Step 1. Build ROS messages:
```
$ cd ros_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

NOTE: remember to [source both your ROS environment and workspace.](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Managing_Your_Environment)

---
## Data parsing scripts for flight dataset
Step 2. Convert SD card binary data to `rosbag`:
```
$ cd dataset/scripts/flight-data/sdcard_scripts
$ python3 log_to_bag.py [SD_CARD_BINARY_DATA]                               # e.g. python3 log_to_bag.py ../../../flight-dataset/binary-data/const1/const1-log1
```

---
Step 3. Visualize the survey results:
```
$ cd dataset/scripts/survey
$ python3 anchor_survey.py [SURVEY_RESULT_TXT]                              # e.g. python3 anchor_survey.py ../../flight-dataset/survey-results/anchor_const1.txt
```

---
Step 4. Visualize UWB measurements:
```
$ cd dataset/scripts/flight-data
$ python3 visual_tdoa2.py -i [ANCHOR_SURVEY_NPZ] [TDOA2_ROSBAG_DATA]        # e.g. python3 visual_tdoa2.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log1.bag 
$ python3 visual_tdoa3.py -i [ANCHOR_SURVEY_NPZ] [TDOA3_ROSBAG_DATA]        # e.g. python3 visual_tdoa3.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log7.bag 
```
For TDOA3, the anchor pair of the visualized UWB measurement is set in the script `visual_tdoa3.py`.

---
Step 5. Visualize UWB measurement bias:
```
$ cd dataset/scripts/flight-data
$ python3 visual_bias.py -i [ANCHOR_SURVEY_NPZ] [TDOA_ROSBAG_DATA]          # e.g. python3 visual_bias.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log1.bag
```
The anchor pair of the visualized UWB measurement is set in the script `visual_bias.py`

---
Step 6. Visualize trajectory and obstacle positions during manual data collections
```
$ cd dataset/scripts/flight-data
$ python3 visual_TrajObs.py -i ../survey/anchor_const3.npz [ROSBAG_DATA]    # e.g. python3 visual_TrajObs.py -i ../survey/anchor_const3.npz ../../rosbag/flight-data/rosbag-const3/const3-tdoa2-obs-log1.bag 
```

---
Step 7. Error-State Kalman Filter Estimation
```
$ cd dataset/scripts/estimation
$ python3 eskf.py -i [ANCHOR_SURVEY_NPZ] [ROSBAG_DATA]                      # e.g. python3 eskf.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log1.bag
```

---
## Data parsing scripts for static dataset

Step 8. Visualize LOS static data
```
$ cd dataset/scripts/static-data
$ python3 los_visual.py -i [LOS_DATA_FOLDER]                                # e.g. python3 los_visual.py -i ../../static-dataset/los/distTest/distT1
```

---
Step 9. Visualize NLOS static data
```
$ cd dataset/scripts/static-data
$ python3 nlos_visual.py -i [NLOS_DATA_FOLDER]                              # e.g. python3 nlos_visual.py -i ../../static-dataset/nlos/anTag/metal/data1
```

-----
> University of Toronto's [Dynamic Systems Lab](https://github.com/utiasDSL) / [Vector Institute](https://github.com/VectorInstitute)
