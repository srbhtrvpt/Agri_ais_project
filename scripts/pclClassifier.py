#!/usr/bin/env python


import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm as CM
import numpy as np
import glob
import sklearn.svm as svm
import sklearn.cluster as skc
import sklearn.mixture as skm
import sklearn.metrics as metrics
import argparse
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


colored_path = "/home/srbh/agrirobo_proj/with_pcls/data/"

fields = ["x","y","z","intensity","rgb","tgi","vari","curvature"]
pcl_fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('intensity', 12, PointField.FLOAT32, 1),
          PointField('tgi', 16, PointField.FLOAT32, 1),
          PointField('rgb', 20, PointField.FLOAT32, 1),
          PointField('vari', 24, PointField.FLOAT32, 1),
          PointField('curvature', 28, PointField.FLOAT32, 1),]

def label_pcl(df):

    test_data = pd.DataFrame(
            {
                "tgi": df["tgi"].tolist(),
                "vari": df["vari"].tolist()
            }
        )
    test_data = test_data.replace([np.inf, -np.inf], np.nan)
    test_data = test_data.fillna(value=0)
    n_classes_km = 2
    kmeans = skc.MiniBatchKMeans(n_classes_km, random_state= 74).fit(test_data)
    labels_km = kmeans.predict(test_data)

    n_classes = 2
    gmm = skm.GaussianMixture(n_components=n_classes, random_state= 88).fit(test_data)
    labels_gmm = gmm.predict(test_data)
    scores = gmm.score_samples(test_data)
    scores_norm = [(float(i)-min(scores))/(max(scores)-min(scores))  for i in scores]


    labels_true = []
    for k, g, s in zip(labels_km, labels_gmm, scores_norm):
        if k == g:
            labels_true.append(k)
        else:
            if s > 0.88:
                labels_true.append(g)
            else:
                labels_true.append(k)
    
    return labels_true


def open_data(pcl_name, pcl_topic):
    count = 0
    topic_out = pcl_topic.replace(pcl_topic.split("/")[-1], '')
    print(topic_out)
    in_bag = rosbag.Bag(pcl_name)
    out_bag_name = pcl_name.split(".")[0] + "_Labelled.bag"
    out_bag = rosbag.Bag(out_bag_name, 'w')   
    for topic, msg, t in in_bag.read_messages():
        out_bag.write(topic, msg, t)
        if topic == pcl_topic:
            df = pd.DataFrame(columns=fields)
            header = Header()
            points_g = []
            points_p = []
            for p in pc2.read_points(msg, skip_nans=True, field_names = fields): #("x","y","z","intensity","rgb","tgi","vari","curvature")
                df_p = pd.Series(p, index= fields)
                df = df.append(df_p, ignore_index=True)
            print("labelling")
            labels = label_pcl(df)
            for d, i in enumerate(labels):
                if d == 0:
                    points_g.append(list(df.iloc[i]))
                else:
                    points_p.append(list(df.iloc[i]))
            count += 1
            header.seq = count
            header.stamp = t
            header.frame_id = msg.header.frame_id
            pc2_g = pc2.create_cloud(header,pcl_fields,points_g)
            pc2_p = pc2.create_cloud(header,pcl_fields,points_p)
            print("writing new topics")
            out_bag.write(topic_out + "segmented_pointcloud_ground", pc2_g, t)
            out_bag.write(topic_out + "segmented_pointcloud_plants", pc2_p, t)
    in_bag.close()
    out_bag.close() 
    print("done")

# def pclClassifier(filename, pcl_topic):

def pclClassifier():


    rospy.init_node('pclClassifier', anonymous=True) 
    filename = rospy.get_param("~input_bag")
    pcl_topic = rospy.get_param("~topic")
    open_data(filename, pcl_topic)
        
if __name__ == "__main__":

    # parser = argparse.ArgumentParser()
    # parser.add_argument("filename")
    # parser.add_argument("topic")
    # args = parser.parse_args()

    try:
        # pclClassifier(args.filename, args.topic)
        pclClassifier()
    except rospy.ROSInterruptException:
      pass






