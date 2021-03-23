import numpy as np
import rosbag
from uwb_msgs.msg import Range
from geometry_msgs.msg import TransformStamped
from itertools import permutations
import matplotlib.pyplot as plt
from scipy.stats import norm

UWB_TOPIC = "/uwb_master_node/uwb_range_data"
# 
ANCHORS = ["anchor_1", "anchor_2", "anchor_5"]

def vicon_topic(anchor_id):
    return "/vicon/dsl_anchor%d/dsl_anchor%d"%(anchor_id, anchor_id)

def parse_transform_stamped(msg):
    return [msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z]

def get_vicon_anchor_dist(bag_file):
    anchor_topics = {}
    anchor_pos_arr = {}
    for anchor in ANCHORS:
        anchor_topics[vicon_topic(int(anchor.split("_")[1]))] = anchor
        anchor_pos_arr[anchor] = []
    
    for topic, msg, t in rosbag.Bag(bag_file).read_messages(topics=anchor_topics):
        anchor_pos_arr[anchor_topics[topic]].append(parse_transform_stamped(msg))
    
    anchor_pos = {}
    for anc, data in anchor_pos_arr.items():
        anchor_pos[anc] = np.mean(data, axis=0)
    
    return anchor_pos

def get_range_data(bag_file, src_dest):
    anchor_pos = get_vicon_anchor_dist(path + "/" + bag_file)
    gt_rng_data = {}
    for src, dests in src_dest.items():
        for dest in dests:
            rng = np.linalg.norm(anchor_pos[src]-anchor_pos[dest])
            print("Vicon %s to %s: %f m"%(src, dest, rng))
            gt_rng_data[src+"-"+dest] = rng
    
    uwb_data = {}
    err_data = {}
    for topic, msg, t in rosbag.Bag(path + "/" + bag_file).read_messages(topics=UWB_TOPIC):
        src_dest = msg.mobile + "-" + msg.anchors[0]
        if src_dest in uwb_data:
            uwb_data[src_dest].append(msg.data[0])
            err_data[src_dest].append(msg.data[0] - gt_rng_data[src_dest])
        else:
            uwb_data[src_dest] = []
            err_data[src_dest] = []
    
    for src_dest, rng in uwb_data.items():
        [src, dest] = src_dest.split('-')
        print("UWB %s to %s: %f m"%(src, dest, np.mean(rng)))
    
    # generate error plot
    for src_dest, data in err_data.items():
        fig = plt.figure()
        n, bins, rectangles = plt.hist(data, 200, density=True)
        fig.canvas.draw()
        mu = np.mean(data)
        std = np.std(data) 
        p = norm.pdf(bins, mu, std)
        plt.plot(bins, p, 'k', linewidth=2)
        plt.title("%s: mean:%.2f (m) std:%.2f (m)"%(src_dest, mu, std))
        plt.ylabel("Range error (m)")
        plt.xlim([-0.35, 0.35])
        plt.grid()
        plt.savefig(path + "/" + src_dest + ".png", format="png")


if __name__ == "__main__":
    path = "/home/abhi/data/anchor_calibration"
    #
    bag_file = "anc1_anc2anc5.bag"
    src_dest = {"anchor_1" : ["anchor_2", "anchor_5"]}
    print("Processing bag:%s"%(path + "/" + bag_file))
    get_range_data(bag_file, src_dest)
    #
    bag_file = "anc2_anc1anc5.bag"
    src_dest = {"anchor_2" : ["anchor_1", "anchor_5"]}
    print("Processing bag:%s"%(path + "/" + bag_file))
    get_range_data(bag_file, src_dest)
    #
    bag_file = "anc5_anc1anc2.bag"
    src_dest = {"anchor_5" : ["anchor_1", "anchor_2"]}
    print("Processing bag:%s"%(path + "/" + bag_file))
    get_range_data(bag_file, src_dest)
    



