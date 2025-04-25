import os
import subprocess
import pandas as pd
import shutil
import time as timer
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("sensitivity_analysis_scenario_A/combined_results_A.csv")

grouped = df.groupby("num_aircraft")["time_to_destination"].agg(["mean", "std", "max", "count"])
print(grouped)

plt.errorbar(grouped.index, grouped["mean"], yerr=grouped["std"], fmt='o-', capsize=4)
plt.title("Average Time to Destination vs Number of Aircraft")
plt.xlabel("Number of Aircraft in Simulation")
plt.ylabel("Average Time to Destination (s)")
plt.grid(True)
plt.tight_layout()
plt.show()
