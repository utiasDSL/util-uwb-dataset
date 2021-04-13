# -*- coding: utf-8 -*-

import cfusdlog
import argparse
import numpy as np
import functools
import rosbag
import rospy

def parseGyroData(data, start_time, bag_file):
    from cf_msgs.msg import Gyro
    for (t, x, y, z) in zip(data['timestamp'],
        data['gyro.x'], data['gyro.y'], data['gyro.z']):
        stamp = (t - start_time)/1000;
        msg = Gyro()
        msg.header.frame_id = "imu"
        msg.header.stamp = rospy.Time(stamp);
        msg.x = x;
        msg.y = y;
        msg.z = z;
        bag_file.write("/gyro_data", msg, rospy.Time(stamp));

def parseAccelData(data, start_time, bag_file):
    from cf_msgs.msg import Accel
    for (t, x, y, z) in zip(data['timestamp'], 
        data['acc.x'], data['acc.y'], data['acc.z']):
        stamp = (t - start_time)/1000;
        msg = Accel()
        msg.header.frame_id = "imu"
        msg.header.stamp = rospy.Time(stamp);
        msg.x = x;
        msg.y = y;
        msg.z = z;
        bag_file.write("/accel_data", msg, rospy.Time(stamp));

def parsePosedata(data, start_time, bag_file):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    for (t, x, y, z, qx, qy, qz, qw) in zip(data['timestamp'],
        data['locSrv.x'], data['locSrv.y'], data['locSrv.z'],
        data['locSrv.qx'], data['locSrv.qy'], data['locSrv.qz'],
        data['locSrv.qw']):
        # print("t:[%.2f] x:[%.2f] y:[%.2f] z:[%.2f] qx:[%.2f] qy:[%.2f] qz:[%.2f] qw:[%.2f]"%(
        #     (t - start_time)/1000, x, y, z, qx, qy, qz, qw));
        stamp = (t - start_time)/1000;
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "crazyflie"
        msg.header.stamp = rospy.Time(stamp);
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = z;
        msg.pose.pose.orientation.x = qx;
        msg.pose.pose.orientation.y = qy;
        msg.pose.pose.orientation.z = qz;
        msg.pose.pose.orientation.w = qw;
        bag_file.write("/pose_data", msg, rospy.Time(stamp));

def parseTdoaData(data, start_time, bag_file):
    from cf_msgs.msg import Tdoa
    for (t, idA, idB, diff) in zip(data['timestamp'], 
        data['idA'], data['idB'], data["distanceDiff"]):
        stamp = (t - start_time)/1000;
        msg = Tdoa()
        msg.header.frame_id = "uwb"
        msg.header.stamp = rospy.Time(stamp);
        msg.idA = idA;
        msg.idB = idB;
        msg.data = diff;
        bag_file.write("/tdoa_data", msg, rospy.Time(stamp));

def parseTofData(data, start_time, bag_file):
    from cf_msgs.msg import Tof
    for (t, zrange) in zip(data['timestamp'], data["range.zrange"]):
        stamp = (t - start_time)/1000;
        msg = Tof()
        msg.header.frame_id = "tof"
        msg.header.stamp = rospy.Time(stamp);
        msg.zrange = zrange*0.001; # mm to m
        bag_file.write("/tof_data", msg, rospy.Time(stamp));

def parseBaroData(data, start_time, bag_file):
    from cf_msgs.msg import Baro
    for (t, asl) in zip(data['timestamp'], data["baro.asl"]):
        stamp = (t - start_time)/1000;
        msg = Baro()
        msg.header.frame_id = "baro"
        msg.header.stamp = rospy.Time(stamp);
        msg.asl = asl;
        bag_file.write("/baro_data", msg, rospy.Time(stamp));

def parseFlowData(data, start_time, bag_file):
    from cf_msgs.msg import Flow
    for (t, dx, dy) in zip(data['timestamp'], data["motion.deltaX"], data["motion.deltaY"]):
        stamp = (t - start_time)/1000;
        msg = Flow()
        msg.header.frame_id = "flow"
        msg.header.stamp = rospy.Time(stamp);
        msg.deltaX = dx;
        msg.deltaY = dy;
        bag_file.write("/flow_data", msg, rospy.Time(stamp));

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_usd")
    args = parser.parse_args()
    # decode binary log data
    data_usd = cfusdlog.decode(args.file_usd)

    import os
    bag_file_path = os.path.abspath(args.file_usd + ".bag");
    print("Writing data to bag file:%s"%bag_file_path)
    bag_file = rosbag.Bag(bag_file_path, "w");
    # find start time
    start_time = None
    for k, (event_name, data) in enumerate(data_usd.items()):
        if start_time is None:
            start_time = data['timestamp'][0]
        else:
            start_time = min(start_time, data['timestamp'][0])

    for k, (event_name, data) in enumerate(data_usd.items()):
        print(k, event_name)
        if event_name == "estPose":
            parsePosedata(data, start_time, bag_file)
        elif event_name == "estGyroscope":
            parseGyroData(data, start_time, bag_file)
        elif event_name == "estAcceleration":
            parseAccelData(data, start_time, bag_file)
        elif event_name == "estTDOA":
            parseTdoaData(data, start_time, bag_file)
        elif event_name == "estTOF":
            parseTofData(data, start_time, bag_file)
        elif event_name == "estBarometer":
            parseBaroData(data, start_time, bag_file)
        elif event_name == "estFlow":
            parseFlowData(data, start_time, bag_file)

    bag_file.close()