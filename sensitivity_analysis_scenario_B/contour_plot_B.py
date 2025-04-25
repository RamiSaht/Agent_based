import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# === Load the CSV files ===
df4 = pd.read_csv("sensitivity_analysis_scenario_B\combined_results_B_tug#=4.csv")
df5 = pd.read_csv("sensitivity_analysis_scenario_B\combined_results_B_tug#=5.csv")
df6 = pd.read_csv("sensitivity_analysis_scenario_B\combined_results_B_tug#=6.csv")

# === Tag each file with number of tugs ===
df4["num_tugs"] = 4
df5["num_tugs"] = 5
df6["num_tugs"] = 6

# === Combine them into one DataFrame ===
combined_df = pd.concat([df4, df5, df6], ignore_index=True)

# === Group by number of aircraft and tugs and calculate mean time ===
grouped = combined_df.groupby(["num_aircraft", "num_tugs"])["time_to_destination"].mean().reset_index()

# === Pivot the table to get format suitable for heatmap ===
pivot = grouped.pivot(index="num_aircraft", columns="num_tugs", values="time_to_destination")

# === Plot the heatmap ===
plt.figure(figsize=(8, 6))
sns.heatmap(pivot, annot=True, fmt=".1f", cmap="Blues", cbar_kws={'label': 'Mean Time to Destination'})
plt.title("Mean Time to Destination for Aircraft")
plt.xlabel("Number of Tugs")
plt.ylabel("Number of Aircraft")
plt.gca().invert_yaxis()  # Invert y-axis to have the lowest number of aircraft at the top
plt.tight_layout()
plt.show()
