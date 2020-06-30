import pandas as pd 
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture


path = "../data/"

intensity_df = pd.read_csv(path + "1535635817097713_intensity.txt", names= ["values"])
curvature_df = pd.read_csv(path + "1535635817097713_curvature.txt", names= ["values"])
normalized_intensity_df=(intensity_df-intensity_df.min())/(intensity_df.max()-intensity_df.min())
normalized_curvature_df=(curvature_df-curvature_df.min())/(curvature_df.max()-curvature_df.min())


fig1, ax = plt.subplots(1, 2)
ax[0].scatter(intensity_df['values'], curvature_df['values'], s= 0.01**2)
ax[0].set_xlabel("intensity")
ax[0].set_ylabel("curvature")
ax[1].hist2d(intensity_df['values'], curvature_df['values'], 25, normed= True, norm = mcolors.PowerNorm(0.3))
ax[1].set_xlabel("intensity")
ax[1].set_ylabel("curvature")
fig1.suptitle("intensity vs curvature")
fig1.savefig(path + "int_vs_curv.png", dpi=400)

fig2 , ax = plt.subplots(1,2)
ax[0].boxplot(intensity_df['values'], showmeans=True, meanline=True)
ax[0].set_ylabel("intensity")
ax[1].boxplot(normalized_intensity_df['values'], showmeans=True, meanline=True)
ax[1].set_ylabel("normalised_intensity")
fig2.suptitle("raw values vs normalised")
fig2.savefig(path + "intensity_boxplot.png", dpi=fig2.dpi)

fig3 , ax = plt.subplots(1,2)
ax[0].boxplot(curvature_df['values'], showmeans=True, meanline=True)
ax[0].set_ylabel("curvature")
ax[1].boxplot(normalized_curvature_df['values'], showmeans=True, meanline=True)
ax[1].set_ylabel("normalized_curvature")
fig3.suptitle("raw values vs normalised")
fig3.savefig(path + "curvature_boxplot.png", dpi=fig3.dpi)

fig4, ax = plt.subplots()
data = pd.DataFrame({'intensity':intensity_df['values'].tolist(), 'curvature':curvature_df['values'].tolist()}) 
ax = data.boxplot(showmeans = True)
ax.set_ylabel("value")
fig4.suptitle("curvature vs intensity")
fig4.savefig(path + "int_vs_curv_raw_boxplot.png", dpi=400)

kmeans = KMeans(2)
kmeans.fit(data)
labels = kmeans.predict(data)
centroids = kmeans.cluster_centers_
fig5, ax = plt.subplots()
colmap = {1: 'b', 2: 'r', 3:'y', 4:'g'}
colors = map(lambda x: colmap[x+1], labels)

ax.scatter(data['intensity'], data['curvature'], color=colors, alpha=0.3, edgecolor='k')
for idx, centroid in enumerate(centroids):
    ax.scatter(*centroid, color=colmap[idx+3])
    print(centroid)
ax.set_xlabel("intensity")
ax.set_ylabel("curvature")
fig5.suptitle("curvature vs intensity kmeans scatter")
fig5.savefig(path + "int_vs_curv_kmeans.png", dpi=400)

fig6, ax = plt.subplots()
gmm = GaussianMixture(n_components=2, covariance_type= 'full').fit(data)
labels = gmm.predict(data)
ax.scatter(data['intensity'], data['curvature'], c=labels, cmap='viridis')
ax.set_xlabel("intensity")
ax.set_ylabel("curvature")
fig6.suptitle("curvature vs intensity gmm scatter")
fig6.savefig(path + "int_vs_curv_gmm.png", dpi=400)



