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



path = "../data/"
path_acc = "../data/accumulated/colored/"
colored_path = "/home/srbh/agrirobo_proj/with_pcls/data/colored/"

colored_files = glob.glob(colored_path + "/*.txt")
li = []
for filename in colored_files:
    df = pd.read_csv(filename, names=["x","y","z","vari","tgi"] ,sep = "\t")
    li.append(df)
colored_df = pd.concat(li, axis=0, ignore_index=True)

df = pd.read_csv(path + "1535635817097713.txt" ,sep = "\t", names=["x","y","z","vari","tgi"])


intensity_path = "/home/srbh/agrirobo_proj/with_pcls/data/intensity/"
curvature_path = "/home/srbh/agrirobo_proj/with_pcls/data/curvature/"

intensity_files = glob.glob(intensity_path + "/*.txt")
curvature_files = glob.glob(curvature_path + "/*.txt")

li2 = []
for filename in intensity_files:
    df1 = pd.read_csv(filename, names=["values"])
    df1 = (df1 - df1.min()) / (df1.max() - df1.min())
    li2.append(df1)
intensity_df = pd.concat(li2, axis=0, ignore_index=True)

li1 = []
# li = [pd.read_csv(filename, names=["values"]) for filename in curvature_files]
for filename in curvature_files:
    df1 = pd.read_csv(filename, names=["values"])
    df1 = (df1 - df1.min()) / (df1.max() - df1.min())
    li1.append(df1)
curvature_df = pd.concat(li1, axis=0, ignore_index=True)


intensity_df_single = pd.read_csv(
            path + "1535635817097713_intensity.txt", names=["values"]
        )
curvature_df_single = pd.read_csv(
            path + "1535635817097713_curvature.txt", names=["values"]
        )
x_df = pd.read_csv(path + "1535635817097713_x.txt", names=["values"])
y_df = pd.read_csv(path + "1535635817097713_y.txt", names=["values"])


train_data_pred = pd.DataFrame(
        {
            "intensity": intensity_df["values"].tolist(),
            "curvature": curvature_df["values"].tolist(),
        }
    )

test_data_pred = pd.DataFrame(
        {
            "x":x_df["values"].tolist(),
            "y":y_df["values"].tolist(),
            "intensity": intensity_df_single["values"].tolist(),
            "curvature": curvature_df_single["values"].tolist(),
        }
    )

# test_data_pred = pd.DataFrame(columns = ["x","y","intensity","curvature"])
# k = 0
# for i in range(len(df.index)):
#     for j in range(len(data_pred.index)):
#         if((df["x"].loc[i] == data_pred["x"].loc[j]) and (df["y"].loc[i] == data_pred["y"].loc[j])):
#             row = [data_pred["x"].loc[j], data_pred["y"].loc[j], data_pred["intensity"].loc[j], data_pred["curvature"].loc[j]]
#             test_data_pred.loc[k] = row
#             k += 1

# print(test_data_pred.index)
# fig1, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=df["tgi"],
#             cmap=CM.jet,
#             s=0.5 ** 2,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig1.colorbar(sc1, ax=ax, label="tgi")
# fig1.suptitle("tgi")
# fig1.savefig(path + "tgi.png", dpi=400)

# fig2, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=df["vari"],
#             cmap=CM.jet,
#             s=0.5 ** 2,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig2.colorbar(sc1, ax=ax, label="vari")
# fig2.suptitle("vari")
# fig2.savefig(path + "vari.png", dpi=400)

# fig3, ax = plt.subplots()
# sc1 = ax.scatter(
#             colored_df["vari"],
#             colored_df["tgi"],
#             s=0.5 ** 2,
#             marker="H",
#         )
# ax.set_xlabel("vari")
# ax.set_ylabel("tgi")
# fig3.savefig(path_acc + "vari_vs_tgi.png", dpi=400)


train_data = pd.DataFrame(
        {
            "tgi": colored_df["tgi"].tolist(),
            "vari": colored_df["vari"].tolist()
        }
    )
train_data = train_data.fillna(value=0)
test_data = pd.DataFrame(
        {
            "tgi": df["tgi"].tolist(),
            "vari": df["vari"].tolist()

        }
    )
n_classes_km = 2
kmeans = skc.MiniBatchKMeans(n_classes_km, random_state= 42).fit(train_data)
labels_km = kmeans.predict(test_data)
centroids = kmeans.cluster_centers_
# fig5, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=labels_km,
#              cmap="inferno",
#             s=0.5 ** 2,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig5.colorbar(sc1, ax=ax, label="labels")
# fig5.suptitle("kmeans segmentation")
# fig5.savefig(path_acc + "kmeans_segmentation.png", dpi=400)


n_classes = 2
gmm = skm.GaussianMixture(n_components=n_classes, random_state= 2).fit(train_data)
labels_gmm = gmm.predict(test_data)
# #probs = gmm.predict_proba(test_data)

scores = gmm.score_samples(test_data)
scores_norm = [(float(i)-min(scores))/(max(scores)-min(scores))  for i in scores]
# fig10, ax = plt.subplots()
# sc = ax.scatter(
#     df["x"],
#     df["y"],
#     c=labels_gmm,
#      cmap="inferno",
#     s=0.5 ** 2,
#     marker="H",
# )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig10.colorbar(sc, ax=ax, label="labels")
# fig10.suptitle("gmm segmentation")
# fig10.savefig(path_acc + "gmm_segmentation.png", dpi=400)


labels_true = []
for k, g, s in zip(labels_km, labels_gmm, scores_norm):
    if k == g:
        labels_true.append(k)
    else:
        if s > 0.88:
            labels_true.append(g)
        else:
            labels_true.append(k)

# n_classes = 2
# brc = skc.Birch(n_clusters = n_classes).fit(test_data)
# labels_brc = brc.predict(test_data)
# fig10, ax = plt.subplots()
# sc = ax.scatter(
#     df["x"],
#     df["y"],
#     c=labels_true,
#     cmap="inferno",
#     s=0.5 ** 2,
#     marker="H",
# )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig10.colorbar(sc, ax=ax, label="labels")
# fig10.suptitle("Ground truth")
# fig10.savefig(path_acc + "true_segmentation.png", dpi=400)



n_classes_km = 2
kmeans = skc.MiniBatchKMeans(n_classes_km, random_state= 42).fit(train_data_pred)
labels_km_pred = kmeans.predict(test_data_pred)

n_classes = 2
gmm = skm.GaussianMixture(n_components=n_classes, random_state= 2).fit(train_data_pred)
labels_gmm_pred = gmm.predict(test_data_pred)

gmm_acc = metrics.accuracy_score(labels_true, labels_gmm_pred)
km_acc = metrics.accuracy_score(labels_true, labels_km_pred)