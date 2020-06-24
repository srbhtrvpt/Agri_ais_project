import pandas as pd 
import matplotlib.pyplot as plt

path = "../data/"

intensity_data = pd.read_csv(path + "intensity.txt", names= ["values"])
curvature_data = pd.read_csv(path + "curvature.txt", names= ["values"])
normalized_intensity_df=(intensity_data-intensity_data.min())/(intensity_data.max()-intensity_data.min())
normalized_curvature_df=(curvature_data-curvature_data.min())/(curvature_data.max()-curvature_data.min())


fig1, ax = plt.subplots(1, 2)
ax[0].scatter(intensity_data['values'], curvature_data['values'], s= 0.01**2)
ax[0].set_xlabel("intensity")
ax[0].set_ylabel("curvature")
ax[1].hist2d(intensity_data['values'], curvature_data ['values'], 25)
ax[1].set_xlabel("intensity")
ax[1].set_ylabel("curvature")
fig1.suptitle("intensity vs curvature")
fig1.savefig(path + "int_vs_curv.png", dpi=400)

fig2 = plt.figure(2)
plt.boxplot(intensity_data['values'], showmeans=True, meanline=True)
plt.ylabel("intensity")
fig2.savefig(path + "intensity_boxplot.png", dpi=fig2.dpi)
fig3 = plt.figure(3)
plt.boxplot(curvature_data['values'], showmeans=True, meanline=True)
plt.ylabel("curvature")
fig3.savefig(path + "curvature_boxplot.png", dpi=fig3.dpi)
fig4 = plt.figure(4)
plt.boxplot(normalized_intensity_df['values'], showmeans=True, meanline=True)
plt.ylabel("normalised_intensity")
fig4.savefig(path + "norm_intensity_boxplot.png", dpi=fig4.dpi)
fig5 = plt.figure(5)
plt.boxplot(normalized_curvature_df['values'], showmeans=True, meanline=True)
plt.ylabel("normalized_curvature")
fig5.savefig(path + "norm_curvature_boxplot.png", dpi=fig5.dpi)




