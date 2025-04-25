import os
import subprocess
import pandas as pd
import shutil
import time as timer
import numpy as np
import matplotlib.pyplot as plt

# Parameters to vary
num_aircraft_range = [5, 10, 15]  # Number of aircraft to simulate
repetitions = 10  # Number of runs per aircraft count/simulation

# Directory to store all outputs
log_dir = "sensitivity_analysis_scenario_B"
os.makedirs(log_dir, exist_ok=True)

# Combined results storage
all_results = []

for num_aircraft in num_aircraft_range:
    for run_id in range(repetitions):
        print(f"\n--> Running Scenario B with {num_aircraft} aircraft (run {run_id + 1})")

        # Write config file for run_me.py to read
        with open("run_config.py", "w") as f:
            f.write(f"random_schedule = True\n")
            f.write(f"num_aircraft = {num_aircraft}\n")
            f.write(f"random_generation_time = 80\n")
            f.write(f"simulation_time = 120\n")

        # Run the simulation (this assumes run_me.py is in the same folder)
        subprocess.run(["python", "run_me_2.py"], check=True)

        # Rename and save output
        base_output = "aircraft_time_to_destination_scenario_B.csv"
        output_file = f"{log_dir}/scenario_B_n{num_aircraft}_run{run_id + 1}.csv"
        shutil.copy(base_output, output_file)

        # Load and tag for analysis
        df = pd.read_csv(output_file)
        df["num_aircraft"] = num_aircraft
        df["run_id"] = run_id + 1
        all_results.append(df)

# Combine all and save
results_df = pd.concat(all_results, ignore_index=True)
results_df.to_csv(f"{log_dir}/combined_results_B.csv", index=False)

print("\n Sensitivity analysis complete. Results saved to:", f"{log_dir}/combined_results_B.csv")

#%% ANALYSE SIMULATION RESULTS
# =============================================================================
# Step 6: Analyze average time vs number of aircraft
print("\n Analyzing average time to destination per aircraft count...\n")

grouped = results_df.groupby("num_aircraft")["time_to_destination"].agg(["mean", "std", "count"])
print(grouped)

# Optional: Save the summary
grouped.to_csv(f"{log_dir}/summary_by_aircraft_count.csv")


plt.figure()
plt.errorbar(grouped.index, grouped["mean"], yerr=grouped["std"], fmt='o-', capsize=5)
plt.title("Average Time to Destination vs Number of Aircraft")
plt.xlabel("Number of Aircraft")
plt.ylabel("Avg Time to Destination (s)")
plt.grid(True)
plt.tight_layout()
plt.savefig(f"{log_dir}/plot_avg_time_vs_aircraft_B.png")
plt.show()


