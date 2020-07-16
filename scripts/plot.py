import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm as CM
import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture


path = "../data/"

intensity_df = pd.read_csv(path + "1535635817097713_intensity.txt", names=["values"])
curvature_df = pd.read_csv(path + "1535635817097713_curvature.txt", names=["values"])
x_df = pd.read_csv(path + "1535635817097713_x.txt", names=["values"])
y_df = pd.read_csv(path + "1535635817097713_y.txt", names=["values"])

normalized_intensity_df = (intensity_df - intensity_df.min()) / (
    intensity_df.max() - intensity_df.min()
)
normalized_curvature_df = (curvature_df - curvature_df.min()) / (
    curvature_df.max() - curvature_df.min()
)

"""
fig1, ax = plt.subplots(1, 2)
ax[0].scatter(intensity_df['values'], curvature_df['values'], s= 0.01**2)
ax[0].set_xlabel("intensity")
ax[0].set_ylabel("curvature")
ax[1].hexbin(intensity_df['values'], curvature_df['values'], norm = mcolors.PowerNorm(0.3), cmap='inferno')
ax[1].set_xlabel("intensity")
ax[1].set_ylabel("curvature")
fig1.suptitle("intensity vs curvature")
fig1.savefig(path + "ivc.png", dpi=400)


fig2, ax = plt.subplots(1, 2)
ax[0].hist(intensity_df["values"], bins=500)
ax[0].set_ylabel("count")
ax[0].set_title("intensity")
ax[1].hist(curvature_df["values"], bins=500)
ax[1].set_title("curvature")
fig2.savefig(path + "intensity_curvature_histograms.png", dpi=400) 



fig3 , ax = plt.subplots(1,2)
ax[0].boxplot(curvature_df['values'], showmeans=True, meanline=True)
ax[0].set_ylabel("curvature")
ax[1].boxplot(normalized_curvature_df['values'], showmeans=True, meanline=True)
ax[1].set_ylabel("normalized_curvature")
fig3.suptitle("raw values vs normalised")
fig3.savefig(path + "curvature_boxplot.png", dpi=400) """

data = pd.DataFrame(
    {
        "intensity": intensity_df["values"].tolist(),
        "curvature": curvature_df["values"].tolist(),
    }
)


"""
fig4, ax = plt.subplots()
ax = data.boxplot(showmeans=True)
ax.set_ylabel("value")
fig4.suptitle("curvature vs intensity")
fig4.savefig(path + "ivc_boxplot.png", dpi=400) """

n_classes_km = 3
kmeans = KMeans(n_classes_km, n_init=100, algorithm="full").fit(data)
labels_km = kmeans.predict(data)
centroids = kmeans.cluster_centers_
fig5, ax = plt.subplots()
colmap = {1: "b", 2: "r", 3: "y", 4: "g", 5:"k"}
colors = map(lambda x: colmap[x + 1], labels_km)
ax.scatter(data["intensity"], data["curvature"], color=colors, alpha=0.3, edgecolor="k")
for idx, centroid in enumerate(centroids):
    ax.scatter(*centroid, color=colmap[idx + 3])
ax.set_xlabel("intensity")
ax.set_ylabel("curvature")
fig5.suptitle("curvature vs intensity kmeans scatter")
fig5.savefig(path + "kmeans_%d.png"%(n_classes_km), dpi=400)


n_classes = 3
fig6, ax = plt.subplots()
gmm = GaussianMixture(n_components=n_classes, covariance_type="full").fit(data)
labels = gmm.predict(data)
probs = gmm.predict_proba(data)
hx = ax.hexbin(data["intensity"], data["curvature"], C=probs.max(1), cmap=CM.jet)
ax.set_xlabel("intensity")
ax.set_ylabel("curvature")
fig6.colorbar(hx, ax=ax, label="probability")
fig6.suptitle("curvature vs intensity gmm heatmap")
fig6.savefig(path + "gmm_%d.png"%(n_classes), dpi=400)


fig7, ax = plt.subplots()
hx1 = ax.hexbin(
    x_df["values"],
    y_df["values"],
    C=data["curvature"],
    cmap="inferno",
    gridsize=(160, 120),
)
ax.set_xlabel("x")
ax.set_ylabel("y")
fig7.colorbar(hx1, ax=ax, label="curvature")
fig7.suptitle("x-y curvature heatmap")
fig7.savefig(path + "x_y_curv.png", dpi=400)
curv_vals = hx1.get_array()
curv_positions = hx1.get_offsets()


fig8, ax = plt.subplots()
hx2 = ax.hexbin(
    x_df["values"],
    y_df["values"],
    C=data["intensity"],
    cmap="inferno",
    gridsize=(160, 120),
)
ax.set_xlabel("x")
ax.set_ylabel("y")
fig8.colorbar(hx2, ax=ax, label="intensity")
fig8.suptitle("x-y intensity heatmap")
fig8.savefig(path + "x_y_int.png", dpi=400)
int_vals = hx2.get_array()
int_positions = hx2.get_offsets()

fig9, ax = plt.subplots()
hx2 = ax.hexbin(x_df["values"], y_df["values"], cmap="inferno", gridsize=(160, 120))
ax.set_xlabel("x")
ax.set_ylabel("y")
fig9.colorbar(hx2, ax=ax, label="intensity")
fig9.suptitle("x-y count heatmap")
fig9.savefig(path + "x_y_count.png", dpi=400)
count_vals = hx2.get_array()
count_positions = hx2.get_offsets()

binned_data = pd.DataFrame({"intensity": int_vals, "curvature": curv_vals})

fig10, ax = plt.subplots()
ax.scatter(
    x_df["values"], y_df["values"], c=labels_km, cmap=CM.jet, s=0.5 ** 2, marker="H"
)
ax.set_xlabel("x")
ax.set_ylabel("y")
fig10.suptitle("km segmentation")
fig10.savefig(path + "km_%d_segmentation.png"%(n_classes_km), dpi=400) 

"""
fig10, ax = plt.subplots()
sc = ax.scatter(
    x_df["values"], y_df["values"], c=labels, cmap=CM.jet, s=0.5 ** 2, marker="H"
)
ax.set_xlabel("x")
ax.set_ylabel("y")
fig10.colorbar(sc, ax=ax, label="labels")
fig10.suptitle("gmm segmentation")
fig10.savefig(path + "gmm_%d_segmentation.png"%(n_classes), dpi=400)



fig10, ax = plt.subplots(1, 2)
ax[0].hexbin(
    intensity_df["values"],
    curvature_df["values"],
    norm=mcolors.PowerNorm(0.3),
    cmap="inferno",
)
ax[0].set_xlabel("intensity")
ax[0].set_ylabel("curvature")
ax[0].set_title("raw")
ax[1].hexbin(
    binned_data["intensity"],
    binned_data["curvature"],
    norm=mcolors.PowerNorm(0.3),
    cmap="inferno",
)
ax[1].set_xlabel("intensity")
ax[1].set_title("binned")
fig10.suptitle("intensity vs curvature")
fig10.savefig(path + "ivc_raw_binned.png", dpi=400)

fig11, ax = plt.subplots()
gmm = GaussianMixture(n_components=3, covariance_type="full").fit(binned_data)
labels = gmm.predict(binned_data)
probs = gmm.predict_proba(binned_data)
hx = ax.hexbin(
    binned_data["intensity"], binned_data["curvature"], C=probs.max(1), cmap=CM.jet
)
ax.set_xlabel("intensity")
ax.set_ylabel("curvature")
fig11.colorbar(hx, ax=ax, label="probability")
fig11.suptitle("curvature vs intensity gmm heatmap binned")
fig11.savefig(path + "gmm_binned_3.png", dpi=400) """

