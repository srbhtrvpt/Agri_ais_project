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

fields = ["x","y","z","intensity","rgb","tgi","vari","curvature"]
pcl_fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('intensity', 12, PointField.FLOAT32, 1),
          PointField('tgi', 16, PointField.FLOAT32, 1),
          PointField('rgb', 20, PointField.FLOAT32, 1),
          PointField('vari', 24, PointField.FLOAT32, 1),
          PointField('curvature', 28, PointField.FLOAT32, 1)]

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

    tgi1 = 0
    tgi2 = 0
    c1 = 0
    c2 = 0
    for i, d in enumerate(labels_true):
        if d == 0:
            tgi1 += df["tgi"].iloc[i]
            c1 +=1
        else:
            tgi2 += df["tgi"].iloc[i]
            c2 +=1
    tgi1 = tgi1/c1
    tgi2 = tgi2/c2
    if tgi1 > tgi2:
        p_label = 0
    else :
        p_label = 1
    
    return labels_true, p_label


def open_data(filename, pcl_topic):
    count = 0
    topic_out = pcl_topic.replace(pcl_topic.split("/")[-1], '')
    in_bag = rosbag.Bag(filename)
    out_bag_name = filename.split(".")[0] + "_Labelled.bag"
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
            labels, p_label = label_pcl(df)
            for i, d in enumerate(labels):
                if d == p_label:
                    points_p.append(df.iloc[i].values.tolist())
                else:
                    points_g.append(df.iloc[i].values.tolist())
            count += 1
            header.seq = count
            header.stamp = t
            header.frame_id = msg.header.frame_id
            pc2_g = pc2.create_cloud(header,pcl_fields,points_g)
            pc2_p = pc2.create_cloud(header,pcl_fields,points_p)
            print("writing new topics")
            out_bag.write(topic_out + "segmented_pointcloud_ground", pc2_p, t)
            out_bag.write(topic_out + "segmented_pointcloud_plants", pc2_g, t)
    in_bag.close()
    out_bag.close() 
    print("done")

# def pclClassifier():

    # rospy.init_node('pclClassifier', anonymous=True) 
    # filename = rospy.get_param("~input_bag")
    # pcl_topic = rospy.get_param("~topic")
    # open_data(filename, pcl_topic)
        
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("filename")
    parser.add_argument("topic")
    args = parser.parse_args()
    # try:
    #     # pclClassifier()
    # except rospy.ROSInterruptException:
    #   pass

    open_data(args.filename, args.topic)






