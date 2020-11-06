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
from sklearn.gaussian_process.kernels import RBF
from sklearn.model_selection import train_test_split
from sklearn.gaussian_process import GaussianProcessClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.cluster import SpectralClustering
import wandb
# wandb.init(project="visualize-sklearn")

def change_labels(x):
    l = []
    for i, d in enumerate(x):
        if d == 0:
            x[i] = 1
        else:
            x[i] = 0
    return x


path = "../data/"
# path_acc = "../data/main_figures/small/"
path_acc = "../data/main_figures/"

# pcd_name = "1535634489692060240_small.txt"
pcd_name = "1535635781571793877.txt"
# pcd_name = "1535635781568669803_plants.txt"
# pcd_name = "1535634489695269178_small_plants.txt"

size = 0.9 ** 2
#size = 1.1 ** 2


# colored_files = glob.glob(colored_path + "/*.txt")
# li = []
# for filename in colored_files:
#     df = pd.read_csv(filename, names=["x","y","z","intensity","rgb","tgi","vari","curvature"] ,sep = "\t")
#     li.append(df)
# full_df = pd.concat(li, axis=0, ignore_index=True)

# df = pd.read_csv(path + pcd_name ,sep = "\t", names=["x","y","z","intensity","rgb","tgi","vari","curvature", "alt"])
# df = df[df["alt"] >= 0]
# df = df[df["y"] <= 0.8]
# df = df[df["y"] >= -2.1]

df = pd.read_csv(
    path + pcd_name,
    sep="\t",
    names=["x", "y", "z", "intensity", "rgb", "tgi", "vari", "curvature"],
)

# train_data_pred = pd.DataFrame(
#         {
#             "intensity": full_df["intensity"].tolist(),
#             "curvature": full_df["curvature"].tolist()
#         }
#     )
# train_data_pred = train_data_pred.replace([np.inf, -np.inf], np.nan)
# train_data_pred = train_data_pred.fillna(value=0)
int_list = df["intensity"].tolist()
curv_list = df["curvature"].tolist()
test_data_pred = pd.DataFrame({"intensity": int_list, "curvature": curv_list})
test_data = pd.DataFrame({"tgi": df["tgi"].tolist(), "vari": df["vari"].tolist()})

# test_data = pd.DataFrame(
#         {
#             "intensity":df["intensity"].tolist(),
#             "curvature":df["curvature"].tolist(),
#             "alt":df["alt"].tolist()
#         }
#     )
# n_clusters=3
# labels_aglo = AgglomerativeClustering(linkage='ward', n_clusters=n_clusters).fit_predict(test_data)
# labels_km = skc.MiniBatchKMeans(n_clusters, random_state= 74).fit_predict(test_data)
# labels_spc2 = skc.DBSCAN(eps=0.9, min_samples=100).fit_predict(test_data_pred)
# labels_spc = skc.DBSCAN(eps=0.5, min_samples=100).fit_predict(test_data)

# labels_spc = SpectralClustering(n_clusters=2,assign_labels="discretize",random_state=0).fit_predict(test_data)
# labels_spc2 = SpectralClustering(n_clusters=2,assign_labels="discretize",random_state=0).fit_predict(test_data_pred)

#
# fig3, ax = plt.subplots()
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
# fig3.colorbar(sc1, ax=ax, label="labels_aglo")
# fig3.suptitle("aglo clustering")
# fig3.savefig(path_acc + "aglo.png", dpi=400)

# fig2, ax = plt.subplots()
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
# fig2.colorbar(sc1, ax=ax, label="labels_km")
# fig2.suptitle("km clustering")
# fig2.savefig(path_acc + "km.png", dpi=400)


# fig3, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=labels_spc2,
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig3.colorbar(sc1, ax=ax, label="labels")
# fig3.suptitle("spectral clustering")
# fig3.savefig(path_acc + "ivc_spc.png", dpi=400)

# fig3, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=labels_spc,
#             cmap="inferno",
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig3.colorbar(sc1, ax=ax, label="labels")
# fig3.suptitle("spectral clustering")
# fig3.savefig(path_acc + "spc.png", dpi=400)

# test_data_pred = pd.DataFrame(columns = ["x","y","intensity","curvature"])
# k = 0
# for i in range(len(df.index)):
#     for j in range(len(data_pred.index)):
#         if((df["x"].loc[i] == data_pred["x"].loc[j]) and (df["y"].loc[i] == data_pred["y"].loc[j])):
#             row = [data_pred["x"].loc[j], data_pred["y"].loc[j], data_pred["intensity"].loc[j], data_pred["curvature"].loc[j]]
#             test_data_pred.loc[k] = row
#             k += 1
# print(test_data_pred.index)
# fig16, ax = plt.subplots()
# ax.hexbin(
#         df["tgi"],
#         df["vari"],
#         norm=mcolors.PowerNorm(0.3),
#         cmap="inferno",
#     )
# ax.set_xlabel("intensity")
# fig16.suptitle("intensity vs curvature")
# fig16.savefig(path_acc + "tgvar.png", dpi=400)

# fig1, ax = plt.subplots()
# sc1 = ax.scatter(
#             df["x"],
#             df["y"],
#             c=df["tgi"],
#             cmap=CM.jet,
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
#             cmap=CM.jet,
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


# fig4, ax = plt.subplots()
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
# fig4.colorbar(sc1, ax=ax, label="offset")
# fig4.suptitle("offset from ground plane")
# fig4.savefig(path_acc + "alt.png", dpi=400)

# fig3, ax = plt.subplots()
# sc1 = ax.scatter(
#             colored_df["vari"],
#             colored_df["tgi"],
#             s=size,
#             marker="H",
#         )
# ax.set_xlabel("vari")
# ax.set_ylabel("tgi")
# fig3.savefig(path_acc + "vari_vs_tgi.png", dpi=400)


n_classes_km = 2
kmeans = skc.MiniBatchKMeans(n_classes_km, random_state=74).fit(test_data)
labels_km = kmeans.predict(test_data)
# labels_km = change_labels(labels_km)

centroids = kmeans.cluster_centers_
# fig5, ax = plt.subplots()
# sc1 = ax.scatter(df["x"], df["y"], c=labels_km, cmap="inferno", s=size, marker="H",)
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig5.colorbar(sc1, ax=ax, label="labels")
# fig5.suptitle("kmeans segmentation")
# fig5.savefig(path_acc + "kmeans_segmentation.png", dpi=400)


n_classes = 2
gmm = skm.GaussianMixture(n_components=n_classes, random_state=88).fit(test_data)
labels_gmm = gmm.predict(test_data)
# labels_gmm = change_labels(labels_gmm)
# #probs = gmm.predict_proba(test_data)

scores = gmm.score_samples(test_data)
scores_norm = [(float(i) - min(scores)) / (max(scores) - min(scores)) for i in scores]


# fig10, ax = plt.subplots()
# sc = ax.scatter(df["x"], df["y"], c=labels_gmm, cmap="inferno", s=size, marker="H",)
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
# labels_true = change_labels(labels_true)

int_plants = []
int_ground = []
for i, k in enumerate(labels_true):
    if k == 1:
        int_plants.append(int_list[i])
    else:
        int_ground.append(int_list[i])

int_bplot = {"plants": int_plants, "non-plants": int_ground}
fig3, ax = plt.subplots()
ax.boxplot(int_bplot.values(), showmeans=True, meanline=True)
ax.set_xticklabels(int_bplot.keys())
fig3.suptitle("intensity boxplot")
fig3.savefig(path_acc + "intensity_boxplot.png", dpi=400)

print("intensity",stat.ttest_ind(int_plants, int_ground, equal_var=False))

curv_plants = []
curv_ground = []
for i, k in enumerate(labels_true):
    if k == 1:
        curv_plants.append(curv_list[i])
    else:
        curv_ground.append(curv_list[i])

curv_bplot = {"plants": curv_plants, "non-plants": curv_ground}
fig4, ax = plt.subplots()
ax.boxplot(curv_bplot.values(), showmeans=True, meanline=True)
ax.set_xticklabels(curv_bplot.keys())
fig4.suptitle("curvature boxplot")
fig4.savefig(path_acc + "curvature_boxplot.png", dpi=400)

print("curvature",stat.ttest_ind(curv_plants, curv_ground, equal_var=False))

# # # n_classes = 2
# # # brc = skc.Birch(n_clusters = n_classes).fit(test_data)
# # # labels_brc = brc.predict(test_data)
# # # fig12, ax = plt.subplots()
# # # sc = ax.scatter(
# # #     df["x"],
# # #     df["y"],
# # #     c=labels_brc,
# # #     cmap="inferno",
# # #     s=size,
# # #     marker="H",
# # # )
# # # ax.set_xlabel("x")
# # # ax.set_ylabel("y")
# # # fig12.colorbar(sc, ax=ax, label="labels")
# # # fig12.suptitle("Birch")
# # # fig12.savefig(path_acc + "brc_segmentation.png", dpi=400)

# fig12, ax = plt.subplots()
# sc = ax.scatter(df["x"], df["y"], c=labels_true, cmap="inferno", s=size, marker="H",)
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig12.colorbar(sc, ax=ax, label="labels")
# fig12.suptitle("Ground truth")
# fig12.savefig(path_acc + "true_segmentation.png", dpi=400)


n_classes_km = 2
kmeans = skc.MiniBatchKMeans(n_classes_km, random_state=27).fit(test_data_pred)
labels_km_pred = kmeans.predict(test_data_pred)
labels_km_pred = change_labels(labels_km_pred)

# fig18, ax = plt.subplots()
# sc = ax.scatter(df["x"], df["y"], c=labels_km_pred, cmap="inferno", s=size, marker="H",)
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig18.colorbar(sc, ax=ax, label="labels")
# fig18.suptitle("km ivc")
# fig18.savefig(path_acc + "ivc_kmeans_segmentation.png", dpi=400)

n_classes = 2
gmm = skm.GaussianMixture(n_components=n_classes, random_state=9).fit(test_data_pred)
labels_gmm_pred = gmm.predict(test_data_pred)
# labels_gmm_pred = change_labels(labels_gmm_pred)

# fig19, ax = plt.subplots()
# sc = ax.scatter(
#     df["x"], df["y"], c=labels_gmm_pred, cmap="inferno", s=size, marker="H",
# )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig19.colorbar(sc, ax=ax, label="labels")
# fig19.suptitle("gm ivc")
# fig19.savefig(path_acc + "ivc_gmm_segmentation.png", dpi=400)

X_train, X_test, y_train, y_test = train_test_split(test_data_pred, labels_true, test_size=0.2, random_state=42)
# spl = int(0.75 * len(test_data))
# clf = svm.SVC().fit(test_data_pred[:spl], labels_true[:spl])
# labels_svc = clf.predict(test_data_pred[spl:])
clf = svm.SVC(probability=True).fit(X_train, y_train)
labels_svc = clf.predict(X_test)
y_probas = clf.predict_proba(X_test)

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

print(p_label)
svc_acc = metrics.accuracy_score(y_test, labels_svc)
svc_fi = metrics.matthews_corrcoef(y_test, labels_svc)

# labels = ['','']
# wandb.sklearn.plot_classifier(clf, X_train, X_test, y_train, y_test, labels_svc, y_probas, labels,
#                                                          model_name='SVC', feature_names=['int', 'curv'])

# clf_g = GaussianProcessClassifier(kernel=1.0 * RBF(1.0),random_state=0).fit(X_train, y_train)
# labels_g = clf_g.predict(X_test)
# g_acc = metrics.accuracy_score(y_test, labels_g)
# g_fi = metrics.matthews_corrcoef(y_test, labels_g)

clf_rf = RandomForestClassifier(max_depth=2, random_state=0).fit(X_train, y_train)
labels_rf = clf_rf.predict(X_test)
rf_acc = metrics.accuracy_score(y_test, labels_rf)
rf_fi = metrics.matthews_corrcoef(y_test, labels_rf)


# # # gm_f1 = metrics.f1_score(labels_true, labels_gmm_pred)
# # # km_f1 = metrics.f1_score(labels_true, labels_km_pred)

# # # gm_mc = metrics.matthews_corrcoef(labels_true, labels_gmm_pred)
# # # km_mc = metrics.matthews_corrcoef(labels_true, labels_km_pred)
# # # # kvg = metrics.accuracy_score(labels_gmm, labels_km)
# # # print(gmm_acc, gm_f1,  gm_mc)
# # # print(km_acc, km_f1, km_mc)
# # # # print(true_acc)
# # # # print(kvg)

# # gmm_acc = metrics.accuracy_score(labels_true, labels_gmm)
# # km_acc = metrics.accuracy_score(labels_true, labels_km)


gmm_acc2 = metrics.accuracy_score(labels_true, labels_gmm_pred)
km_acc2 = metrics.accuracy_score(labels_true, labels_km_pred)

gm_mc = metrics.matthews_corrcoef(labels_true, labels_gmm_pred)
km_mc = metrics.matthews_corrcoef(labels_true, labels_km_pred)

# svc_acc = metrics.accuracy_score(labels_true[spl:], labels_svc)
# svc_fi = metrics.matthews_corrcoef(labels_true[spl:], labels_svc)

# # print(gmm_acc, km_acc)

print("gmm accuracy",gmm_acc2, "km accuracy",km_acc2)
print("gmm MCC", gm_mc, "km MCC" ,km_mc)
print("svc accuracy", svc_acc, "svc MCC", svc_fi)
# print("gaus accuracy", g_acc, "gaus MCC", g_fi)
print("rf accuracy", rf_acc, "rf MCC", rf_fi)

