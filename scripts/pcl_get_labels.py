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


colored_path = "/home/srbh/agrirobo_proj/with_pcls/data/"

def open_data(pcd_name):
    df = pd.read_csv(colored_path + pcd_name + ".txt" ,sep = "\t", names=["x","y","z","intensity","rgb","tgi","vari","curvature"])
    return df

def label_pcl(df,pcd_name):

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


    with open(colored_path + "labels_" + pcd_name + ".txt", "w") as outfile:
        outfile.write("\n".join(str(item) for item in labels_true))


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("filename")
    args = parser.parse_args()
    df = open_data(args.filename)
    label_pcl(df,args.filename)
    print("done", args.filename)






