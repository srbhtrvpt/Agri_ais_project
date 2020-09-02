import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm as CM
import numpy as np


path = "../data/"

df = pd.read_csv(path + "1535635819847282.txt" ,sep = "\t", names=["x","y","z","rgb","tgi"])

fig1, ax = plt.subplots()
sc1 = ax.scatter(
            df["x"],
            df["y"],
            c=df["tgi"],
            cmap=CM.jet,
            s=0.5 ** 2,
            marker="H",
        )
ax.set_xlabel("x")
ax.set_ylabel("y")
fig1.colorbar(sc1, ax=ax, label="tgi")
fig1.suptitle("tgi")
fig1.savefig(path + "tgi.png", dpi=400)

