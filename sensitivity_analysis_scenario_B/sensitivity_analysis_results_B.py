import os
import subprocess
import pandas as pd
import shutil
import glob
import time as timer
import numpy as np
import matplotlib.pyplot as plt

import pandas as pd
import glob
import os

import os
import pandas as pd
import matplotlib.pyplot as plt

# Directory where individual CSV files are stored
log_dir = "sensitivity_analysis_scenario_B"

# Find all individual CSV files in the directory
csv_files = [f for f in os.listdir(log_dir) 
             if f.startswith("scenario_B_n") and f.endswith(".csv")]

all_results = []

# Load each file and extract parameters from filename
for file in csv_files:
    print(f"Loading {file}...")
    
    # Extract num_aircraft and run_id from filename (format: scenario_B_n{num_aircraft}_run{run_id}.csv)
    parts = file.split('_')
    num_aircraft = int(parts[2][1:])  # Extract number after 'n'
    run_id = int(parts[3][3:-4])      # Extract number after 'run' and remove '.csv'
    
    try:
        # Load the CSV
        df = pd.read_csv(f"{log_dir}/{file}")
        
        # Add metadata columns
        df["num_aircraft"] = num_aircraft
        df["run_id"] = run_id
        
        all_results.append(df)
    except Exception as e:
        print(f"Error loading {file}: {str(e)}")
        continue

# Check if any files were loaded
if not all_results:
    print("No valid CSV files found in the directory!")
    exit()

# Combine all DataFrames
results_df = pd.concat(all_results, ignore_index=True)

# Save combined results
combined_path = f"{log_dir}/combined_results_B.csv"
results_df.to_csv(combined_path, index=False)
print(f"\nSuccessfully combined {len(csv_files)} files into {combined_path}")

#%% ANALYSIS AND PLOTTING (same as your original)
print("\nAnalyzing average time to destination per aircraft count...\n")

# Group and calculate statistics
grouped = results_df.groupby("num_aircraft")["time_to_destination"].agg(["mean", "std", "count"])
print(grouped)

# Save the summary
summary_path = f"{log_dir}/summary_by_aircraft_count.csv"
grouped.to_csv(summary_path)
print(f"\nSummary statistics saved to {summary_path}")

# Create plot
plt.figure(figsize=(10, 6))
plt.errorbar(grouped.index, grouped["mean"], 
             yerr=grouped["std"], 
             fmt='o-', 
             capsize=5,
             linewidth=2,
             markersize=8)
plt.title("Average Time to Destination vs Number of Aircraft", fontsize=14)
plt.xlabel("Number of Aircraft", fontsize=12)
plt.ylabel("Avg Time to Destination (s)", fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)

# Save and show plot
plot_path = f"{log_dir}/plot_avg_time_vs_aircraft_B.png"
plt.tight_layout()
plt.savefig(plot_path, dpi=300, bbox_inches='tight')
print(f"Plot saved to {plot_path}")
plt.show()
