import pandas as pd 
import matplotlib.pyplot as plt

path = "/home/srbh/catkin_ws/src/myproject1/src/data/"

intensity_data = pd.read_csv(path + "intensity.txt", names= ["values"])
curvature_data = pd.read_csv(path + "curvature.txt", names= ["values"])
normalized_intensity_df=(intensity_data-intensity_data.min())/(intensity_data.max()-intensity_data.min())
normalized_curvature_df=(curvature_data-curvature_data.min())/(curvature_data.max()-curvature_data.min())


fig1 = plt.figure(1)
plt.scatter(intensity_data['values'], curvature_data['values'], s= 0.01**2)
fig1.savefig(path + "int_vs_curv.png", dpi=400)
fig2 = plt.figure(2)
plt.boxplot(intensity_data['values'], showmeans=True, meanline=True)
fig2.savefig(path + "intensity_boxplot.png", dpi=fig2.dpi)
fig3 = plt.figure(3)
plt.boxplot(curvature_data['values'], showmeans=True, meanline=True)
fig3.savefig(path + "curvature_boxplot.png", dpi=fig3.dpi)
fig4 = plt.figure(4)
plt.boxplot(normalized_intensity_df['values'], showmeans=True, meanline=True)
fig4.savefig(path + "norm_intensity_boxplot.png", dpi=fig4.dpi)
fig5 = plt.figure(5)
plt.boxplot(normalized_curvature_df['values'], showmeans=True, meanline=True)
fig5.savefig(path + "norm_curvature_boxplot.png", dpi=fig5.dpi)



