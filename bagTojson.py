import rosbag
import os
import sys
import numpy as np
import json

bag_path = './trial6/nlos-toda2-trial6_traj_3-obs.bag'
output_path_1 = 'obs_traj_t6/tdoa2_traj3.json'
output_path_2 = 'obs_traj_t6/drone_toda2_traj3.json'
def savetojson(path,array):
    print("Length of the array is: ",len(array))
    with open(path,'w') as f:
        json.dump(array,f)
    print("Saved Success!")
    return True

def clean(list):
    new = []
    for i in list:
        new.append(float("{:.5f}".format(i/1000)))
    return new

# access rosbag
ros_bag = bag_path
bag = rosbag.Bag(ros_bag)
# bag_file = os.path.split(sys.argv[-1])[1]
# print("visualizing rosbag: " + str(bag_file) + "\n")

obs_pose = []
drone_pose = []
for topic, msg, t in bag.read_messages(['/vicon/markers','/vicon/cf7/cf7']):
    drone = []
    obs = [[], [], [], []]
    if(topic == "/vicon/markers"):
        ### obs1,obs2,obs3,obs4
        for i in range(len(msg.markers)):
            if(msg.markers[i].marker_name == 'metal_obs1'):
                obs[0] += clean([msg.markers[i].translation.x,msg.markers[i].translation.y,msg.markers[i].translation.z])
            if (msg.markers[i].marker_name == 'metal_obs2'):
                obs[1] += clean([msg.markers[i].translation.x, msg.markers[i].translation.y, msg.markers[i].translation.z])
            if (msg.markers[i].marker_name == 'metal_obs3'):
                obs[2] += clean([msg.markers[i].translation.x, msg.markers[i].translation.y,msg.markers[i].translation.z])
            if(msg.markers[i].marker_name == 'metal_obs4'):
                obs[3] += clean([msg.markers[i].translation.x, msg.markers[i].translation.y, msg.markers[i].translation.z])
        obs_pose += [obs]
    if(topic == "/vicon/cf7/cf7"):
        drone += [float("{:.5f}".format(msg.transform.translation.x)),float("{:.5f}".format(msg.transform.translation.y)),float("{:.5f}".format(msg.transform.translation.z))]
        drone_pose += [drone]


savetojson(output_path_1,obs_pose)
savetojson(output_path_2,drone_pose)
# print(len(obs_pose))
# print(len(drone_pose))
# print(obs_pose[233])
# print(drone_pose[233])
