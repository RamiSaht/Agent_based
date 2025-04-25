import subprocess
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import shutil
from scipy import stats

# === CONFIGURATION ===
N = 200                     # Max number of runs
min_runs = 20              # Minimum runs before checking convergence
target_cov = 0.02           # Target Coefficient of Variation threshold
stable_steps = 3            # Stop if CoV varies <2% for this many checks
step_size = 5               # Analyze every N runs
num_aircraft = 10           # Number of aircraft
sim_time = 75              # Simulation time (seconds)
gen_time = 50               # Aircraft generation window (seconds)

# === SETUP ===
output_dir = "convergence_analysis_scenario_A"
os.makedirs(output_dir, exist_ok=True)
cov_data = []
all_results = []

def calculate_ci(data, confidence=0.95):
    """Calculate mean and confidence intervals."""
    n = len(data)
    mean = np.mean(data)
    ci = stats.t.interval(confidence, n-1, loc=mean, scale=stats.sem(data))
    return mean, ci[0], ci[1]

for run_id in range(1, N+1):
    print(f"Run {run_id}/{N}...")

    # Update config file (EXCLUDING PLANNER TO LEAVE IT UNCHANGED)
    with open("run_config.py", "w") as f:
        f.write(f"random_schedule = True\n")
        f.write(f"num_aircraft = {num_aircraft}\n")
        f.write(f"random_generation_time = {gen_time}\n")
        f.write(f"simulation_time = {sim_time}\n")

    # Run Scenario A simulation (using run_me.py)
    subprocess.run(["python", "run_me.py"], check=True)

    # Load and save results (adjust filename if needed)
    result_file = "output_time_to_destination_scenario_A.csv" #scenario A
    if not os.path.exists(result_file):
        raise FileNotFoundError(f"{result_file} not found. Check Scenario A's output.")
    
    target_file = os.path.join(output_dir, f"run_{run_id}.csv")
    shutil.copy(result_file, target_file)

    df = pd.read_csv(result_file)
    df["run"] = run_id
    all_results.append(df)

    # Check convergence periodically
    if run_id >= min_runs and run_id % step_size == 0:
        combined = pd.concat(all_results)
        avg_times = combined.groupby("run")["time_to_destination"].mean()
        
        mean, ci_low, ci_high = calculate_ci(avg_times)
        cov = np.std(avg_times) / mean
        
        cov_data.append({
            "runs": run_id,
            "mean": mean,
            "ci_low": ci_low,
            "ci_high": ci_high,
            "cov": cov
        })

        # Early termination if CoV stabilizes
        if len(cov_data) > stable_steps:
            last_covs = [d["cov"] for d in cov_data[-stable_steps:]]
            cov_change = np.abs(np.diff(last_covs)).max()
            if cov_change < 0.01 * target_cov:
                print(f"Converged at {run_id} runs (CoV change <1%).")
                break

# Save convergence metrics
cov_df = pd.DataFrame(cov_data)
cov_df.to_csv(os.path.join(output_dir, "convergence_metrics_A.csv"), index=False)

# === PLOT RESULTS ===
plt.figure(figsize=(10, 6))
plt.plot(cov_df["runs"], cov_df["cov"], "o-", label="CoV")
plt.fill_between(
    cov_df["runs"], 
    cov_df["ci_low"], 
    cov_df["ci_high"], 
    alpha=0.2, 
    label="95% CI"
)
plt.axhline(y=target_cov, color="r", linestyle="--", label="Target CoV")
plt.xlabel("Number of Runs")
plt.ylabel("Coefficient of Variation")
plt.title(f"Scenario A Convergence for {num_aircraft} Aircraft")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, "convergence_plot_A.png"))
plt.show()