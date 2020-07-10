import pandas as pd 
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm as CM
import numpy as np
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture


path = "../data/"

intensity_df = pd.read_csv(path + "1535635817097713_intensity.txt", names= ["values"])
curvature_df = pd.read_csv(path + "1535635817097713_curvature.txt", names= ["values"])
x_df = pd.read_csv(path + "1535635817097713_x.txt", names= ["values"])
y_df = pd.read_csv(path + "1535635817097713_y.txt", names= ["values"])

normalized_intensity_df=(intensity_df-intensity_df.min())/(intensity_df.max()-intensity_df.min())
normalized_curvature_df=(curvature_df-curvature_df.min())/(curvature_df.max()-curvature_df.min())


fig1, ax = plt.subplots(1, 2)
ax[0].scatter(intensity_df['values'], curvature_df['values'], s= 0.01**2)
ax[0].set_xlabel("intensity")
ax[0].set_ylabel("curvature")
ax[1].hist2d(intensity_df['values'], curvature_df['values'], 25, normed= True, norm = mcolors.PowerNorm(0.3), cmap='viridis')
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

kmeans = KMeans(2, n_init=100, algorithm='full').fit(data)
labels = kmeans.predict(data)
centroids = kmeans.cluster_centers_
fig5, ax = plt.subplots()
colmap = {1: 'b', 2: 'r', 3:'y', 4:'g'}
colors = map(lambda x: colmap[x+1], labels)
ax.scatter(data['intensity'], data['curvature'], color=colors, alpha=0.3, edgecolor='k')
for idx, centroid in enumerate(centroids):
    ax.scatter(*centroid, color=colmap[idx+3])
    #print(centroid)
ax.set_xlabel("intensity")
ax.set_ylabel("curvature")
fig5.suptitle("curvature vs intensity kmeans scatter")
fig5.savefig(path + "int_vs_curv_kmeans.png", dpi=400)

fig6, ax = plt.subplots()
gmm = GaussianMixture(n_components=2, covariance_type= 'full').fit(data)
labels = gmm.predict(data)
probs = gmm.predict_proba(data)
#ax.scatter(data['intensity'], data['curvature'], c=labels, cmap='viridis')
hx = ax.hexbin(data['intensity'], data['curvature'],  C= probs.max(1), cmap=CM.jet)
#ax.hist2d(data['intensity'], data['curvature'], c=labels, cmap='viridis')
ax.set_xlabel("intensity")
ax.set_ylabel("curvature")
fig6.colorbar(hx, ax=ax, label = 'probability')
fig6.suptitle("curvature vs intensity gmm heatmap")
fig6.savefig(path + "int_vs_curv_gmm.png", dpi=400)


fig7, ax = plt.subplots()
hx1 = ax.hexbin(x_df['values'], y_df['values'], C= data['curvature'], cmap='inferno', gridsize= (40,30))
ax.set_xlabel("x")
ax.set_ylabel("y")
fig7.colorbar(hx1, ax= ax , label = 'curvature')
fig7.suptitle("x-y curvature heatmap")
fig7.savefig(path + "x_y_curv.png", dpi=400)
curv_vals = hx1.get_array()
curv_positions = hx1.get_offsets()


fig8, ax = plt.subplots()
hx2 = ax.hexbin(x_df['values'], y_df['values'], C= data['intensity'], cmap='inferno', gridsize= (40,30))
ax.set_xlabel("x")
ax.set_ylabel("y")
fig8.colorbar(hx2, ax=ax, label = 'intensity')
fig8.suptitle("x-y intensity heatmap")
fig8.savefig(path + "x_y_int.png", dpi=400)
int_vals = hx2.get_array()
int_positions = hx2.get_offsets()
