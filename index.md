<img src="files/images/start-fig.png" alt="start-img" width="1000">

---
## Summary
The UTIAS ultra-wideband (UWB) time-difference-of-arrival (TDOA) consists of low-level signal information from static experiments and UWB TDOA measurements and additional onboard sensor data from flight experiments on a quadrotor. The [Loco Positioning System (LPS)](https://www.bitcraze.io/documentation/system/positioning/loco-positioning-system/) from [Bitcraze](https://www.bitcraze.io/) based on DWM1000 UWB modules is used to create this dataset. We hope this dataset can help researchers develop and compare reliable estimation methods for emerging UWB TDOA-based indoor localization technology. 

---
## Static Dataset
For the static experiments, we collected UWB TDOA measurements under various line-of-sight (LOS) and non-line-of-sight (NLOS) conditions. Two UWB anchors and one [Crazyflie nano-quadrotor](https://www.bitcraze.io/products/old-products/crazyflie-2-0/) equipped with an UWB tag are placed on wooden structures. A millimeter-level accurate Vicon motion capture system measures the poses of the tag and the anchors for groundtruth data.

### Line-of-sight (LOS) experiments
<!-- word -- left, fig -- right -->
<div style="clear: both;">
  <div style="float: right; margin-left 3em;">
    <img src="files/images/static-los.png" alt="" width="400">
  </div>
  <div>
    <p>In LOS conditions, we collected data from two tests: (1) the LOS distance test and (2) the LOS angle test. The positions of the tag and anchor 2 are fixed throughout the LOS data collection process. In LOS distance test, we change the distance d1 from 0.5 meters to 6.5 meter with an interval of 0.5 meter.In LOS angle dataset, we change the angle from 180 degree to 15 degree with an interval of 15 degree. </p>
  </div>
</div>

### Non-line-of-sight (NLOS) experiments

<div style="clear: both;">
  <div style="float: right; margin-left 3em;">
    <img src="files/images/static-nlos.png" alt="" width="400">
  </div>
  <div>
    <p>During the NLOS tests, we fixed the positions of the tag and two anchors and placed different obstacles to block the line-of-sight of TDOA measurements. To reflect the comprehensive performance of UWB NLOS measurements, we selected six obstacles of different type of materials commonly used in indoor settings, including cardboard, metal, wood, plastic, and foam. We conducted NLOS experiments under (i) NLOS conditions between anchor 1 and the tag and (ii) NLOS condition between anchor 1 and anchor 2. Considering the different radio reflection and diffraction effects with one obstacle under different orientations, we collect six sub-datasets for each NLOS condition with different orientations of the obstacle. One LOS data is collected for comparison.
    </p>
  </div>
</div>
 
### Static dataset format
<div style="clear: both;">
  <div style="float: right; margin-left 3em;">
    <img src="files/images/static-data-format.png" alt="" width="400">
  </div>
  <div>
    <p>In each sub-dataset, we provide a csv file containing the collected data and a txt file containing the poses of the tag and two anchors in one folder. For NLOS tests, the positions of the four markers on the obstacles are also included in the txt file. The format of the csv file and brief descriptions of each value are summarized in table on the right. Detailed information can be found in the dataset paper.
    </p>
    <p>&nbsp;</p>   <!-- line break -->
    <p>&nbsp;</p>
    <p>&nbsp;</p>
    <p>&nbsp;</p>
    <p>&nbsp;</p>
    <p>&nbsp;</p>
    <p>&nbsp;</p>
  </div>
</div>

---
## Flight Dataset
For the flight experiments, we collected the raw UWB meaurements, gyroscope, accelerometer, optical flow, time-of-flight (ToF) laser-ranging, barometer, and the Vicon pose measurements (sent from the ground station) on-board a customized quadrotor platform.

### Flight arena and experimental setup
<div style="clear: both;">
  <div style="float: right; margin-left 3em;">
    <img src="files/images/flight-setup.png" alt="" width="400">
  </div>
  <div>
    <p>The UWB TDOA flight dataset is produced in a  7.0 m × 8.0 m × 3.5 m indoor flight arena equipped with a motion capture system of 10 <a href="https://www.vicon.com/hardware/cameras/vantage/">Vicon Vantage+ cameras</a>. Printed Apriltags are attached to the soft mattresses to provide visual features for optical flow. For each sub-dataset, eight UWB anchors were pre-installed in the flight arena referred to as a constellation. Three different UWB constellations are used for data collection. The position and orientation of each anchor were surveyed using a mm-level accurate <a href="https://leica-geosystems.com/products/total-stations/">Leica total station</a> for reproducibility.
    </p> 
    <p>We refer to the Vicon frame (see the right figure) as the inertial frame. To align the Leica total station frame and the inertial frame, we use the total station to survey six Vicon reflective markers with known positions in inertial frame and computethe transformation matrix through point cloud alignment. The average reprojection root-mean-squared error (RMSE) of the six reflective markers is around 1.12 mm.
    </p>
  </div>
</div>

### Quadrotor platform
<div style="clear: both;">
  <div style="float: right; margin-left 3em;">
    <img src="files/images/drone-setup.png" alt="" width="400">
  </div>
  <div>
    <p>We built a customized quadrotor based on the <a href="https://store.bitcraze.io/products/crazyflie-bolt/">Crazyflie Bolt</a> flight controller with an inertial measurement unit (IMU) and  attached commercially available extension boards (so-called decks) from Bitcraze for data collection. The LPS UWB tag is mounted vertically on the top to receive UWB TDOA measurements. A flow deck attached at the bottom provides optical flow measurements and a laser-based ToF sensor provides the local altitude information. The accelerometer and gyroscope data is obtained from the  onboard IMU. A micro SD card deck logs the raw sensor data received by the flight control board with high-precision microsecond timestamps. The customized quadrotor communicates with a ground station computer over a 2.4 GHz USB radio dongle <a href="https://www.bitcraze.io/products/crazyradio-pa/">(Crazyradio PA)</a> for high-level interaction. In terms of software, we use the <a href="https://github.com/USC-ACTLab/crazyswarm/">Crazyswarm</a> package (Preiss et al. (2017)) to send high-level commands and the pose of the quadrotor measured by the motion capture system  to the quadrotor. </p>
  </div>
</div>

### Time synchronization, latency, and calibration
Onboard the quadrotor, the raw UWB measurements, gyroscope, accelerometer, optical flow, ToF laser-ranging, barometer, and the Vicon pose measurements (sent from the ground station) are recorded as [event streams](https://www.bitcraze.io/2021/03/event-based-data-logging/). The Vicon pose measurements logged onboard are treated as the ground truth data.  Each datapoint is timestamped with the onboard microsecond timer and the resulting time series is written to the micro SD card as a binary file. Python scripts are provided to parse and analyze the binary data. In terms of the latency from the ground station software to the onboard firmware, we manually flicked the quadrotor and compared the offset between the acceleration and Vicon measurement spikes. The latency is tested to be around 22 ms. As the length of each sub-dataset is around 120 seconds, we ignore the onboard clock drift.

We refer to the offset between the center of a sensor and the vehicle center as sensor extrinsic parameters. The IMU is assumed to be aligned with the vehicle center. We provide the manually measured translation vectors from the center of the vehicle to onboard sensors (UWB tag and flow deck) in the dataset paper and the data parsing scripts.

### Localization performance
<div style="clear: both;">
  <div style="float: right; margin-left 3em;">
    <img src="files/images/eskf.png" alt="" width="420">
  </div>
  <div>
    <p>In the flight dataset, we provide the UWB measurements under centralized TDOA mode (TDOA2) and decentralized TDOA mode (TDOA3). One centralized TDOA measurement and the Vicon ground truth are shown on the right as an example. The flight dataset can be used to assess the UWB TDOA-based localization performance using low-cost DWM1000 UWB modules. We provide an error-state Kalman filter implementation for localization and the performance is demonstrated on the bottom right. Users are encouraged to design new algorithms to cope with the UWB measurement errors and noise for accurate indoor localizaiton.</p>
    <p>&nbsp;</p>
  </div>
</div>

### Flight dataset format
For each UWB constellation, we provide the raw Leica total station survey results and computed anchor poses in txt files. In each sub-dataset, we provide the timestamped TDOA, accelerometer, gyroscope, optical flow, ToF laser-ranger, and the barometer measurements and the ground truth measurements of the quadrotor’s pose in a csv file. The data format is shown in the following table. We provide both Matlab and Python scripts to parse the data.
<img src="files/images/flight-data-format.png" alt="" width="800">


## User Instructions
We provide the instructions for running the Python scripts. For the corresponding Matlab scripts, please change the path for the data on top of the scripts for usage.
### Procedure
---
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
### Data parsing scripts for flight dataset
Step 2. Convert SD card binary data to `rosbag`:
```
$ cd dataset/scripts/flight-data/sdcard_scripts
$ python3 log_to_bag.py [SD_CARD_BINARY_DATA]                               
# e.g. python3 log_to_bag.py ../../../flight-dataset/binary-data/const1/const1-log1
```

---
Step 3. Convert the survey results to the inertial frame:
```
$ cd dataset/scripts/survey
$ python3 anchor_survey.py [SURVEY_RESULT_TXT]                              
# e.g. python3 anchor_survey.py ../../flight-dataset/survey-results/anchor_const1.txt
```
We also provide the converted survey results as npz and txt files.

---
Step 4. Visualize UWB measurements:
```
$ cd dataset/scripts/flight-data
$ python3 visual_tdoa2.py -i [ANCHOR_SURVEY_NPZ] [TDOA2_ROSBAG_DATA]        
# e.g. python3 visual_tdoa2.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log1.bag 

$ python3 visual_tdoa3.py -i [ANCHOR_SURVEY_NPZ] [TDOA3_ROSBAG_DATA]        
# e.g. python3 visual_tdoa3.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log7.bag 
```
For TDOA3, the anchor pair of the visualized UWB measurement is set in the script `visual_tdoa3.py`.

---
Step 5. Visualize UWB measurement bias:
```
$ cd dataset/scripts/flight-data
$ python3 visual_bias.py -i [ANCHOR_SURVEY_NPZ] [TDOA_ROSBAG_DATA]          
# e.g. python3 visual_bias.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log1.bag
```
The anchor pair of the visualized UWB measurement is set in the script `visual_bias.py`

---
Step 6. Visualize trajectory and obstacle positions during manual data collections
```
$ cd dataset/scripts/flight-data
$ python3 visual_TrajObs.py -i ../survey/anchor_const3.npz [ROSBAG_DATA]    
# e.g. python3 visual_TrajObs.py -i ../survey/anchor_const3.npz ../../rosbag/flight-data/rosbag-const3/const3-tdoa2-obs-log1.bag 
```

---
Step 7. Error-State Kalman Filter Estimation
```
$ cd dataset/scripts/estimation
$ python3 eskf.py -i [ANCHOR_SURVEY_NPZ] [ROSBAG_DATA]                      
# e.g. python3 eskf.py -i ../survey/anchor_const1.npz ../../rosbag/flight-data/rosbag-const1/const1-log1.bag
```

---
### Data parsing scripts for static dataset

Step 8. Visualize LOS static data
```
$ cd dataset/scripts/static-data
$ python3 los_visual.py -i [LOS_DATA_FOLDER]                                
# e.g. python3 los_visual.py -i ../../static-dataset/los/distTest/distT1
```

---
Step 9. Visualize NLOS static data
```
$ cd dataset/scripts/static-data
$ python3 nlos_visual.py -i [NLOS_DATA_FOLDER]                              
# e.g. python3 nlos_visual.py -i ../../static-dataset/nlos/anTag/metal/data1
```

## Credits

This dataset was the work of [Wenda Zhao](https://williamwenda.github.io/), [Abhishek Goudar](https://www.linkedin.com/in/abhishek-goudar-47b46090/), and [Angela P. Schoellig](https://www.dynsyslab.org/prof-angela-schoellig/). If you use the data provided by this website in your own work, please use the following citation:
```
@INPROCEEDINGS{zhao2021uwbData,
      title={The UTIAS ultra-wideband time-difference-of-arrival dataset for indoor localization}, 
      author={Wenda Zhao and Abhishek Goudar and Angela P. Schoellig},
      booktitle={2021 International Journal of Robotics Research (IJRR)},
      year={2021},
      volume={},
      number={},
      pages={},
      doi={}
}
```
-----
> University of Toronto's [Dynamic Systems Lab](https://github.com/utiasDSL) / [Vector Institute](https://github.com/VectorInstitute)/ [UofT Robotics Institute](https://robotics.utoronto.ca/)
