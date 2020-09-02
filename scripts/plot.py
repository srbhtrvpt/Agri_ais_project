import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm as CM
import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture
from scipy import stats
import glob
import bisect


path = "../data/"

def get_idx(h, n):
    idx = bisect.bisect(h, n)
    if 0 < idx < len(h):
        return idx-1
    elif idx == len(h):
        return idx-2
    else:
        raise ValueError(n, " is out of bounds of ", h)

def load_ivc(single_file):
    global path
    if single_file:
        intensity_df = pd.read_csv(
            path + "1535635817097713_intensity.txt", names=["values"]
        )
        curvature_df = pd.read_csv(
            path + "1535635817097713_curvature.txt", names=["values"]
        )
        return intensity_df, curvature_df

    else:
        path = "../data/accumulated/"
        intensity_path = "/home/srbh/agrirobo_proj/with_pcls/data/intensity/"
        curvature_path = "/home/srbh/agrirobo_proj/with_pcls/data/curvature/"

        intensity_files = glob.glob(intensity_path + "/*.txt")
        curvature_files = glob.glob(curvature_path + "/*.txt")

        li = []
        for filename in intensity_files:
            df = pd.read_csv(filename, names=["values"])
            df = (df - df.min()) / (df.max() - df.min())
            li.append(df)
        intensity_df = pd.concat(li, axis=0, ignore_index=True)

        li1 = []
        # li = [pd.read_csv(filename, names=["values"]) for filename in curvature_files]
        for filename in curvature_files:
            df = pd.read_csv(filename, names=["values"])
            df = (df - df.min()) / (df.max() - df.min())
            li1.append(df)
        curvature_df = pd.concat(li1, axis=0, ignore_index=True)
        return intensity_df, curvature_df


def load_xy():
    x_df = pd.read_csv(path + "1535635817097713_x.txt", names=["values"])
    y_df = pd.read_csv(path + "1535635817097713_y.txt", names=["values"])
    return x_df, y_df


def plot(xy_plot):

    intensity_df, curvature_df = load_ivc(single_file=xy_plot)
    data = pd.DataFrame(
        {
            "intensity": intensity_df["values"].tolist(),
            "curvature": curvature_df["values"].tolist(),
        }
    )
    normalized_intensity_df = (intensity_df - intensity_df.min()) / (
        intensity_df.max() - intensity_df.min()
    )
    normalized_curvature_df = (curvature_df - curvature_df.min()) / (
        curvature_df.max() - curvature_df.min()
    )

    fig1, ax = plt.subplots(1, 2)
    ax[0].scatter(intensity_df["values"], curvature_df["values"], s=0.01 ** 2)
    ax[0].set_xlabel("intensity")
    ax[0].set_ylabel("curvature")
    ax[1].hexbin(
        intensity_df["values"],
        curvature_df["values"],
        norm=mcolors.PowerNorm(0.3),
        cmap="inferno",
    )
    ax[1].set_xlabel("intensity")
    fig1.suptitle("intensity vs curvature")
    fig1.savefig(path + "ivc.png", dpi=400)

    fig2, ax = plt.subplots(1, 2)
    n1, bins1, patch1 = ax[0].hist(intensity_df["values"], bins=500)
    ax[0].set_ylabel("count")
    ax[0].set_title("intensity")
    n2, bins2, patch2 = ax[1].hist(curvature_df["values"], bins=50)
    ax[1].set_title("curvature")
    fig2.savefig(path + "intensity_curvature_histograms.png", dpi=400)

    z_int = np.abs(stats.zscore(intensity_df))
    z_curv = np.abs(stats.zscore(curvature_df))  # z scores for outliers

    z_count = np.abs(stats.zscore(n1))
    z_count2 = np.abs(stats.zscore(n2))
    # print(z_count2[(z_count2 > 2)])
    print("no. outliers intensity: ", n1[np.where(z_count > 5)[0]][0])
    print("no. outliers curvature: ", n2[np.where(z_count2 > 2)[0]][0])

    # int_outliers = intensity_df.loc[(intensity_df["values"] > bins1[np.where(z_count > 5)[0]][0]) & (intensity_df["values"] < bins1[np.where(z_count > 5)[0] +1][0])]
    mask = (intensity_df["values"] > bins1[np.where(z_count > 5)[0]][0]) & (intensity_df["values"] < bins1[np.where(z_count > 5)[0] +1][0])
    labels_outlier_intensity = [0 if mask[i] == True else 1 for i in range(mask.size)]
    # print(len(label_outlier))
    # print(mask)

    mask2 = (curvature_df["values"] > bins2[np.where(z_count2 > 2)[0]][0]) & (curvature_df["values"] < bins2[np.where(z_count2 > 2)[0] +1][0])
    labels_outlier_curvature = [0 if mask2[i] == True else 1 for i in range(mask2.size)]
    # print(np.where(z_count > 5))

    z_val_int = [z_count[get_idx(bins1, r["values"])] for i,r in intensity_df.iterrows()]
    z_val_curv = [z_count2[get_idx(bins2, r["values"])] for i,r in curvature_df.iterrows()]

    

    """fig3 , ax = plt.subplots(1,2)
    ax[0].boxplot(curvature_df['values'], showmeans=True, meanline=True)
    ax[0].set_ylabel("curvature")
    ax[1].boxplot(normalized_curvature_df['values'], showmeans=True, meanline=True)
    ax[1].set_ylabel("normalized_curvature")
    fig3.suptitle("raw values vs normalised")
    fig3.savefig(path + "curvature_boxplot.png", dpi=400) """

    fig4, ax = plt.subplots()
    ax = data.boxplot(showmeans=True)
    ax.set_ylabel("value")
    fig4.suptitle("curvature vs intensity")
    fig4.savefig(path + "ivc_boxplot.png", dpi=400)

    n_classes_km = 2
    kmeans = KMeans(n_classes_km, n_init=100, algorithm="full").fit(data)
    labels_km = kmeans.predict(data)
    centroids = kmeans.cluster_centers_
    fig5, ax = plt.subplots()
    colmap = {1: "b", 2: "r", 3: "y", 4: "g", 5: "k"}
    colors = map(lambda x: colmap[x + 1], labels_km)
    ax.scatter(
        data["intensity"], data["curvature"], color=colors, alpha=0.3, edgecolor="k"
    )
    for idx, centroid in enumerate(centroids):
        ax.scatter(*centroid, color=colmap[idx + 3])
    ax.set_xlabel("intensity")
    ax.set_ylabel("curvature")
    fig5.suptitle("curvature vs intensity kmeans scatter")
    fig5.savefig(path + "kmeans_%d.png" % (n_classes_km), dpi=400)

    n_classes = 2
    fig6, ax = plt.subplots()
    gmm = GaussianMixture(n_components=n_classes, covariance_type="full").fit(data)
    labels = gmm.predict(data)
    probs = gmm.predict_proba(data)
    hx = ax.hexbin(data["intensity"], data["curvature"], C=probs.max(1), cmap=CM.jet)
    ax.set_xlabel("intensity")
    ax.set_ylabel("curvature")
    fig6.colorbar(hx, ax=ax, label="probability")
    fig6.suptitle("curvature vs intensity gmm heatmap")
    fig6.savefig(path + "gmm_%d.png" % (n_classes), dpi=400)

    if xy_plot:
        x_df, y_df = load_xy()
        fig7, ax = plt.subplots()
        hx1 = ax.hexbin(
            x_df["values"],
            y_df["values"],
            C=data["curvature"],
            cmap="inferno",
            gridsize=(160, 120),
            norm=mcolors.PowerNorm(0.5),
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
            norm=mcolors.PowerNorm(0.5),
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig8.colorbar(hx2, ax=ax, label="intensity")
        fig8.suptitle("x-y intensity heatmap")
        fig8.savefig(path + "x_y_int.png", dpi=400)
        int_vals = hx2.get_array()
        int_positions = hx2.get_offsets()

        fig9, ax = plt.subplots()
        hx2 = ax.hexbin(
            x_df["values"],
            y_df["values"],
            cmap="inferno",
            gridsize=(160, 120),
            norm=mcolors.PowerNorm(0.5),
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig9.colorbar(hx2, ax=ax, label="count")
        fig9.suptitle("x-y count heatmap")
        fig9.savefig(path + "x_y_count.png", dpi=400)
        count_vals = hx2.get_array()
        count_positions = hx2.get_offsets()

        binned_data = pd.DataFrame({"intensity": int_vals, "curvature": curv_vals})

        fig10, ax = plt.subplots()
        ax.scatter(
            x_df["values"],
            y_df["values"],
            c=labels_km,
            cmap=CM.jet,
            s=0.5 ** 2,
            marker="H",
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig10.suptitle("km segmentation")
        fig10.savefig(path + "kmeans_%d_segmentation.png" % (n_classes_km), dpi=400)

        fig10, ax = plt.subplots()
        sc = ax.scatter(
            x_df["values"],
            y_df["values"],
            c=labels,
            cmap=CM.jet,
            s=0.5 ** 2,
            marker="H",
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig10.colorbar(sc, ax=ax, label="labels")
        fig10.suptitle("gmm segmentation")
        fig10.savefig(path + "gmm_%d_segmentation.png" % (n_classes), dpi=400)


        fig11, ax = plt.subplots()
        sc = ax.scatter(
            x_df["values"],
            y_df["values"],
            c=labels_outlier_intensity,
            cmap=CM.jet,
            s=0.5 ** 2,
            marker="H",
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig11.colorbar(sc, ax=ax, label="labels")
        fig11.suptitle("outliers intensity")
        fig11.savefig(path + "outliers_intensity.png", dpi=400)

        fig12, ax = plt.subplots()
        sc = ax.scatter(
            x_df["values"],
            y_df["values"],
            c=labels_outlier_curvature,
            cmap=CM.jet,
            s=0.5 ** 2,
            marker="H",
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig12.colorbar(sc, ax=ax, label="labels")
        fig12.suptitle("outliers curvature")
        fig12.savefig(path + "outliers_curvature.png", dpi=400)


        fig13, ax = plt.subplots()
        hx2 = ax.hexbin(
            x_df["values"],
            y_df["values"],
            C=z_val_int,
            cmap="inferno",
            gridsize=(160, 120),
            # norm=mcolors.PowerNorm(0.5),
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig13.colorbar(hx2, ax=ax, label="z score")
        fig13.suptitle("x-y intensity z score heatmap")
        fig13.savefig(path + "x_y_int_z.png", dpi=400)

        fig14, ax = plt.subplots()
        hx2 = ax.hexbin(
            x_df["values"],
            y_df["values"],
            C=z_val_curv,
            cmap="inferno",
            gridsize=(160, 120),
            # norm=mcolors.PowerNorm(0.3)
        )
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        fig14.colorbar(hx1, ax=ax, label="z score")
        fig14.suptitle("x-y curvature z score heatmap")
        fig14.savefig(path + "x_y_curv_z.png", dpi=400)
"""
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


if __name__ == "__main__":
    plot(xy_plot=True)

