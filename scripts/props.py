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



path = "../data/"
# path_acc = "../data/accumulated/full_pcl/small/"
path_acc = "../data/accumulated/full_pcl/"

colored_path = "/home/srbh/agrirobo_proj/with_pcls/data/"
# pcd_name = "1535634489692060240_small.txt"
pcd_name = "1535635817392096375.txt"


# colored_files = glob.glob(colored_path + "/*.txt")
# li = []
# for filename in colored_files:
#     df = pd.read_csv(filename, names=["x","y","z","intensity","rgb","tgi","vari","curvature"] ,sep = "\t")
#     li.append(df)
# full_df = pd.concat(li, axis=0, ignore_index=True)

df = pd.read_csv(path + pcd_name ,sep = "\t", names=["x","y","z","intensity","rgb","tgi","vari","curvature"])


# train_data_pred = pd.DataFrame(
#         {
#             "intensity": full_df["intensity"].tolist(),
#             "curvature": full_df["curvature"].tolist()
#         }
#     )
# train_data_pred = train_data_pred.replace([np.inf, -np.inf], np.nan)
# train_data_pred = train_data_pred.fillna(value=0)
int_list =  df["intensity"].tolist()
curv_list =  df["curvature"].tolist()
test_data_pred = pd.DataFrame(
        {
            "intensity":int_list,
            "curvature":curv_list
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


# train_data = pd.DataFrame(
#         {
#             "tgi": full_df["tgi"].tolist(),
#             "vari": full_df["vari"].tolist()
#         }
#     )
# train_data = train_data.replace([np.inf, -np.inf], np.nan)
# train_data = train_data.fillna(value=0)
test_data = pd.DataFrame(
        {
            "tgi": df["tgi"].tolist(),
            "vari": df["vari"].tolist()
        }
    )
n_classes_km = 2
kmeans = skc.MiniBatchKMeans(n_classes_km, random_state= 74).fit(test_data)
labels_km = kmeans.predict(test_data)
centroids = kmeans.cluster_centers_
fig5, ax = plt.subplots()
sc1 = ax.scatter(
            df["x"],
            df["y"],
            c=labels_km,
             cmap="viridis",
            s=0.5 ** 2,
            marker="H",
        )
ax.set_xlabel("x")
ax.set_ylabel("y")
fig5.colorbar(sc1, ax=ax, label="labels")
fig5.suptitle("kmeans segmentation")
fig5.savefig(path_acc + "kmeans_segmentation.png", dpi=400)


n_classes = 2
gmm = skm.GaussianMixture(n_components=n_classes, random_state= 88).fit(test_data)
labels_gmm = gmm.predict(test_data)
# #probs = gmm.predict_proba(test_data)

scores = gmm.score_samples(test_data)
scores_norm = [(float(i)-min(scores))/(max(scores)-min(scores))  for i in scores]



fig10, ax = plt.subplots()
sc = ax.scatter(
    df["x"],
    df["y"],
    c=labels_gmm,
     cmap="viridis",
    s=0.5 ** 2,
    marker="H",
)
ax.set_xlabel("x")
ax.set_ylabel("y")
fig10.colorbar(sc, ax=ax, label="labels")
fig10.suptitle("gmm segmentation")
fig10.savefig(path_acc + "gmm_segmentation.png", dpi=400)


labels_true = []
for k, g, s in zip(labels_km, labels_gmm, scores_norm):
    if k == g:
        labels_true.append(k)
    else:
        if s > 0.88:
            labels_true.append(g)
        else:
            labels_true.append(k)

int_plants = []
int_ground = []
for i, k  in enumerate(labels_true):
    if k == 1:
        int_plants.append(int_list[i])
    else:
        int_ground.append(int_list[i])

int_bplot = {'plants':int_plants,'ground':int_ground}
fig3 , ax = plt.subplots()
ax.boxplot(int_bplot.values(),showmeans=True, meanline=True)
ax.set_xticklabels(int_bplot.keys())
fig3.suptitle("intensity boxplot")
fig3.savefig(path_acc + "intensity_boxplot.png", dpi=400)

print(stat.ttest_ind(int_plants,int_ground, equal_var=False))

curv_plants = []
curv_ground = []
for i, k  in enumerate(labels_true):
    if k == 1:
        curv_plants.append(curv_list[i])
    else:
        curv_ground.append(curv_list[i])

curv_bplot = {'plants':int_plants,'ground':int_ground}
fig4 , ax = plt.subplots()
ax.boxplot(curv_bplot.values(),showmeans=True, meanline=True)
ax.set_xticklabels(curv_bplot.keys())
fig4.suptitle("curvature boxplot")
fig4.savefig(path_acc + "curvature_boxplot.png", dpi=400)

print(stat.ttest_ind(curv_plants,curv_ground, equal_var=False))

# n_classes = 2
# brc = skc.Birch(n_clusters = n_classes).fit(test_data)
# labels_brc = brc.predict(test_data)
# fig12, ax = plt.subplots()
# sc = ax.scatter(
#     df["x"],
#     df["y"],
#     c=labels_brc,
#     cmap="inferno",
#     s=0.5 ** 2,
#     marker="H",
# )
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# fig12.colorbar(sc, ax=ax, label="labels")
# fig12.suptitle("Birch")
# fig12.savefig(path_acc + "brc_segmentation.png", dpi=400)

fig12, ax = plt.subplots()
sc = ax.scatter(
    df["x"],
    df["y"],
    c=labels_true,
    cmap="viridis",
    s=0.5 ** 2,
    marker="H",
)
ax.set_xlabel("x")
ax.set_ylabel("y")
fig12.colorbar(sc, ax=ax, label="labels")
fig12.suptitle("Ground truth")
fig12.savefig(path_acc + "true_segmentation.png", dpi=400)


# n_classes_km = 2
# kmeans = skc.MiniBatchKMeans(n_classes_km, random_state= 4).fit(train_data_pred)
# labels_km_pred = kmeans.predict(test_data_pred)


# # fig1, ax = plt.subplots()
# # sc = ax.scatter(
# #     df["x"],
# #     df["y"],
# #     c=labels_km_pred,
# #     cmap="inferno",
# #     s=0.5 ** 2,
# #     marker="H",
# # )
# # ax.set_xlabel("x")
# # ax.set_ylabel("y")
# # fig1.colorbar(sc, ax=ax, label="labels")
# # fig1.suptitle("km ivc")
# # fig1.savefig(path_acc + "ivc_kmeans_segmentation.png", dpi=400)

# n_classes = 2
# gmm = skm.GaussianMixture(n_components=n_classes, random_state= 99).fit(train_data_pred)
# labels_gmm_pred = gmm.predict(test_data_pred)

# # fig2, ax = plt.subplots()
# # sc = ax.scatter(
# #     df["x"],
# #     df["y"],
# #     c=labels_gmm_pred,
# #     cmap="inferno",
# #     s=0.5 ** 2,
# #     marker="H",
# # )
# # ax.set_xlabel("x")
# # ax.set_ylabel("y")
# # fig2.colorbar(sc, ax=ax, label="labels")
# # fig2.suptitle("gm ivc")
# # fig2.savefig(path_acc + "ivc_gmm_segmentation.png", dpi=400)

# # gmm_acc = metrics.accuracy_score(labels_true, labels_gmm)
# # km_acc = metrics.accuracy_score(labels_true, labels_km)
# # true_acc = metrics.accuracy_score(labels_true, labels_brc)
# # kvg = metrics.accuracy_score(labels_gmm, labels_km)
# spl = int(0.8*len(test_data))
# clf = svm.SVC().fit(test_data[:spl], labels_true[:spl])
# labels_svc = clf.predict(test_data[spl:])


# with open(colored_path + "labels_" + pcd_name, "w") as outfile:
#     outfile.write("\n".join(str(item) for item in labels_true))

# svc_acc = metrics.accuracy_score(labels_true[spl:], labels_svc)
# print(svc_acc)
# gmm_acc = metrics.accuracy_score(labels_true, labels_gmm_pred)
# km_acc = metrics.accuracy_score(labels_true, labels_km_pred)

# gm_f1 = metrics.f1_score(labels_true, labels_gmm_pred)
# km_f1 = metrics.f1_score(labels_true, labels_km_pred)

# gm_mc = metrics.matthews_corrcoef(labels_true, labels_gmm_pred)
# km_mc = metrics.matthews_corrcoef(labels_true, labels_km_pred)

# print(gmm_acc, gm_f1,  gm_mc)
# print(km_acc, km_f1, km_mc)
# # print(true_acc)
# # print(kvg)

gmm_acc = metrics.accuracy_score(labels_true, labels_gmm)
km_acc = metrics.accuracy_score(labels_true, labels_km)

print(gmm_acc)
print(km_acc)