import os
import subprocess
import pandas as pd
import shutil

# Parameters
file_name_tug_number = "Tug2"  # ##### change manually for each tug run
base_input_dir = "sensitivity_analysis_scenario_B"
log_dir = "tug_simulation_outputs"
schedule_dir = "schedules"
os.makedirs(log_dir, exist_ok=True)
os.makedirs(schedule_dir, exist_ok=True)

# Simulation parameters
sim_time = 150  # Simulation time
gen_time = 50   # Generation time window

# List all scenario CSV files
scenario_files = sorted(
    [f for f in os.listdir(base_input_dir) if f.startswith("scenario_B_n") and f.endswith(".csv")]
)

print(f"Found {len(scenario_files)} scenario files to process.")

for scenario_file in scenario_files:
    full_path = os.path.join(base_input_dir, scenario_file)
    print(f"\n--> Processing {scenario_file}")

    # Load the individual scenario result
    df = pd.read_csv(full_path)

    # Extract run_id and num_aircraft from filename
    parts = scenario_file.replace(".csv", "").split("_")
    num_aircraft = int(parts[2][1:])  # 'n12' -> 12
    run_id = int(parts[3][3:])        # 'run1' -> 1

    # Process the schedule
    processed_schedule = []
    for _, row in df.iterrows():
        ac_id = row['aircraft_id']
        path = eval(row['path'])  # Convert string to list (path is a string representation of a list)
        start_node = path[0]
        end_node = path[-1]
        start_time = row['start_time']

        # Determine arrival/departure type based on start_node
        arrival_departure = "A" if start_node in [37, 38] else "D"

        # Create the processed row
        processed_schedule.append({
            "ac_id": ac_id,
            "arrival_departure": arrival_departure,
            "start_node": start_node,
            "end_node": end_node,
            "t": start_time
        })
    # Convert the processed schedule to DataFrame
    processed_df = pd.DataFrame(processed_schedule)

    # Save the processed schedule to a CSV file
    schedule_file = f"{schedule_dir}/schedule_run{run_id}_n{num_aircraft}.csv"
    processed_df.to_csv(schedule_file, index=False)

    # Write the config file
    with open("run_config.py", "w") as f:
        f.write(f"random_schedule = False\n")
        f.write(f"schedule_file = '{schedule_file}'\n")
        f.write(f"num_aircraft = {num_aircraft}\n")
        f.write(f"random_generation_time = {gen_time}\n")
        f.write(f"simulation_time = {sim_time}\n")

    # Run the simulation
    subprocess.run(["python", "run_me_2.py"], check=True)

    # Save/rename the tug output
    base_output = "tug_energy_scenario_B.csv"  # adjust if necessary
    output_file = f"{log_dir}/tug_data_run{run_id}_n{num_aircraft}.csv"
    shutil.copy(base_output, output_file)

print("\n All simulations complete. Tug data saved in:", log_dir)