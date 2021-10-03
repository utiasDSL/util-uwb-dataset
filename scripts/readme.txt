User Instructions
We provide the instructions for running the Python scripts. For the corresponding Matlab scripts, please change the path for the data on top of the scripts for usage.

Access data
Clone the Git repository, download the latest release of the dataset, and decompose the zip file into the base folder.

ROS workspace
Step 1. Build ROS messages:

$cd ros_ws/src
$catkin_init_workspace
$cd ..
$catkin_make
$source devel/setup.bash
NOTE: remember to source both your ROS environment and workspace.

Data parsing scripts for flight dataset
Step 2. Convert SD card binary data to rosbag:

$ cd scripts/flight-data/sdcard_scripts
$ python3 log_to_bag.py [SD_CARD_BINARY_DATA]    
                           
# e.g. python3 log_to_bag.py ../../../dataset/flight-dataset/binary-data/const1/const1-log1

NOTE: we provide the converted rosbag data in the folder: “dataset/flight-dataset/rosbag-data/”.

Step 3. Convert the survey results to the inertial frame:

$ cd scripts/survey
$ python3 anchor_survey.py [SURVEY_RESULT_TXT]  
                            
# e.g. python3 anchor_survey.py ../../dataset/flight-dataset/survey-results/raw-data/anchor_const1.txt

NOTE, we provide the converted survey results (npz and txt files) in the folder: “dataset/flight-dataset/survey-results/”.

Step 4. Visualize UWB measurements:

$ cd scripts/flight-dataset
$ python3 visual_tdoa2.py -i [ANCHOR_SURVEY_NPZ] [TDOA2_ROSBAG_DATA]        
# e.g. python3 visual_tdoa2.py -i ../../dataset/flight-dataset/survey-results/anchor_const1.npz ../../dataset/flight-dataset/rosbag-data/const1/const1-log1.bag 

$ python3 visual_tdoa3.py -i [ANCHOR_SURVEY_NPZ] [TDOA3_ROSBAG_DATA]        
# e.g. python3 visual_tdoa3.py -i ../../dataset/flight-dataset/survey-results/anchor_const1.npz ../../dataset/flight-dataset/rosbag-data/const1/const1-log7.bag 

For TDOA3, the anchor pair of the visualized UWB measurement is set in the script visual_tdoa3.py.


Step 5. Visualize UWB measurement bias:

$ cd scripts/flight-dataset
$ python3 visual_bias.py -i [ANCHOR_SURVEY_NPZ] [TDOA_ROSBAG_DATA]        
  
# e.g. python3 visual_bias.py -i ../../dataset/flight-dataset/survey-results/anchor_const1.npz ../../dataset/flight-dataset/rosbag-data/const1/const1-log1.bag

The anchor pair of the visualized UWB measurement is set in the script visual_bias.py


Step 6. Visualize the trajectory and obstacle positions of manual data collections
$ cd scripts/flight-dataset
$ python3 visual_TrajObs.py -i [ANCHOR_SURVEY_NPZ] [ROSBAG_DATA]    

# e.g. python3 visual_TrajObs.py -i ../../dataset/flight-dataset/survey-results/anchor_const3.npz ../../dataset/flight-dataset/rosbag-data/const3/const3-tdoa2-obs-log1.bag 

NOTE: manual data collections at the presence of obstacles were conducted in const. 3.


Step 7. Error-State Kalman Filter Estimation
$ cd scripts/estimation
$ python3 eskf.py -i [ANCHOR_SURVEY_NPZ] [ROSBAG_DATA]  
                    
# e.g. python3 eskf.py -i ../../dataset/flight-dataset/survey-results/anchor_const1.npz ../../dataset/flight-dataset/rosbag-data/const1/const1-log1.bag


Data parsing scripts for static dataset
Step 8. Visualize LOS static data
$ cd scripts/static-data
$ python3 los_visual.py -i [LOS_DATA_FOLDER]  
                              
# e.g. python3 los_visual.py -i ../../dataset/static-dataset/los/distTest/distT1


Step 9. Visualize NLOS static data
$ cd scripts/static-data
$ python3 nlos_visual.py -i [NLOS_DATA_FOLDER]          
                    
# e.g. python3 nlos_visual.py -i ../../dataset/static-dataset/nlos/anTag/metal/data1











