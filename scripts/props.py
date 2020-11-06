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
import scipy.stats as stat
from sklearn.cluster import AgglomerativeClustering
from sklearn.cluster import AffinityPropagation
from sklearn.cluster import DBSCAN


def change_labels(x):
    l = []
    for i,d in enumerate(x):
        if d == 0:
            x[i] = 1
        else:
            x[i] = 0
    return x


path = "../data/"
# path_acc = "../data/main_figures/small/plants/"

path_acc = "../data/main_figures/plants/"

# pcd_name = "1535634489692060240_small.txt"
# pcd_name = "1535635781571793877.txt"
pcd_name = "1535635781568669803_plants.txt"
# pcd_name = "1535634489695269178_small_plants.txt"

size = 1.6 ** 2


df = pd.read_csv(path + pcd_name ,sep = "\t", names=["x","y","z","intensity","rgb","tgi","vari","curvature", "alt"])
df = df[df["alt"] >= 0]
df = df[df["y"] <= 0.8]
df = df[df["y"] >= -2.1]


# int_list =  df["intensity"].tolist()
# curv_list =  df["curvature"].tolist()


test_data = pd.DataFrame(
        {
            "intensity":df["intensity"].tolist(),
            "curvature":df["curvature"].tolist(),            
            "alt":df["alt"].tolist()
        }
    )
n_clusters=3
labels_aglo = AgglomerativeClustering(linkage='ward', n_clusters=n_clusters).fit_predict(test_data)
labels_km = skc.MiniBatchKMeans(n_clusters, random_state= 74).fit_predict(test_data)
# labels_dbs = skc.DBSCAN(eps=0.006).fit_predict(test_data)
gmm = skm.GaussianMixture(n_components=n_clusters, random_state= 9).fit(test_data)
labels_gmm = gmm.predict(test_data)



# fig21, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=labels_aglo,
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig21.colorbar(sc1, ax=ax, label="labels_aglo")
# fig21.suptitle("aglo clustering")
# fig21.savefig(path_acc + "aglo.png", dpi=400)

# fig20, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=labels_km,
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig20.colorbar(sc1, ax=ax, label="labels_km")
# fig20.suptitle("km clustering")
# fig20.savefig(path_acc + "km.png", dpi=400)


# fig3, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=labels_dbs,
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig3.colorbar(sc1, ax=ax, label="labels_dbs")
# fig3.suptitle("DBscan clustering")
# fig3.savefig(path_acc + "dbs.png", dpi=400)



# fig1, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=df["tgi"],
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig1.colorbar(sc1, ax=ax, label="tgi")
# fig1.suptitle("tgi")
# fig1.savefig(path_acc + "tgi.png", dpi=400)

# fig2, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=df["vari"],
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig2.colorbar(sc1, ax=ax, label="vari")
# fig2.suptitle("vari")
# fig2.savefig(path_acc + "vari.png", dpi=400)

# fig3, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=df["curvature"],
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig3.colorbar(sc1, ax=ax, label="curvature")
# fig3.suptitle("curvature")
# fig3.savefig(path_acc + "curvature.png", dpi=400)

# fig4, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=df["intensity"],
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig4.colorbar(sc1, ax=ax, label="intensity")
# fig4.suptitle("intensity")
# fig4.savefig(path_acc + "intensity.png", dpi=400)


# fig5, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=df["alt"],
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig5.colorbar(sc1, ax=ax, label="offset")
# fig5.suptitle("offset from ground plane")
# fig5.savefig(path_acc + "alt.png", dpi=400)



fig2, ax = plt.subplots()
sc = ax.scatter(
    df["x"],
    df["y"],
    c=labels_gmm,
    cmap="inferno",
    s=size,
    marker="H",
)
ax.set_xlabel("x")
ax.set_ylabel("y")
fig2.colorbar(sc, ax=ax, label="labels")
fig2.suptitle("gm")
fig2.savefig(path_acc + "gmm_segmentation.png", dpi=400)

# spl = int(0.8*len(test_data))
# clf = svm.SVC().fit(test_data[:spl], labels_true[:spl])
# labels_svc = clf.predict(test_data[spl:])

# # # gm_f1 = metrics.f1_score(labels_true, labels_gmm_pred)
# # # km_f1 = metrics.f1_score(labels_true, labels_km_pred)

# # # gm_mc = metrics.matthews_corrcoef(labels_true, labels_gmm_pred)
# # # km_mc = metrics.matthews_corrcoef(labels_true, labels_km_pred)
# # # # kvg = metrics.accuracy_score(labels_gmm, labels_km)
# # # print(gmm_acc, gm_f1,  gm_mc)
# # # print(km_acc, km_f1, km_mc)
# # # # print(true_acc)
# # # # print(kvg)

# gmm_acc = metrics.accuracy_score(labels_true, labels_gmm)
# km_acc = metrics.accuracy_score(labels_true, labels_km)

# gmm_acc2 = metrics.accuracy_score(labels_true, labels_gmm_pred)
# km_acc2 = metrics.accuracy_score(labels_true, labels_km_pred)

# svc_acc = metrics.accuracy_score(labels_true[spl:], labels_svc)
# print(gmm_acc, km_acc)

# # print(gmm_acc2, km_acc2)
# print(svc_acc)