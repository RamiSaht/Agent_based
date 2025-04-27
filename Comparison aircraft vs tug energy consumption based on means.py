import numpy as np
import pandas as pd
import numpy as np
from scipy import stats

def calculate_real_energies(percentages, total_battery=840):
    # Convert percentages to actual energy values in kWh
    real_energies = [(percent / 100) * total_battery for percent in percentages]
    return real_energies


# Load the data from each file
energy_5 = pd.read_csv('energy_consumption_means_5.csv')


total_energy_5 = energy_5['total_energy_consumed'] * 5


print(f"Mean battery consumption for file 5: {total_energy_5}")

consumed_energy_5 = calculate_real_energies(total_energy_5)

print(f"Consumed energy for file 5 in kWH: {consumed_energy_5}")

fuel_energy_density =43.2 * 0.277777778 #conversion to kWH
print("Here")
print(fuel_energy_density)
fuel_consumption_per_minute = 0.1 * 60 #kg/min
energy_ac = pd.read_csv('mean_results_A.csv')
time_to_destination = energy_ac['time_to_destination']
energy_6_ac = time_to_destination[0] * 6 * fuel_consumption_per_minute* fuel_energy_density
energy_8_ac = time_to_destination[1] * 8 * fuel_consumption_per_minute* fuel_energy_density
energy_10_ac = time_to_destination[2] * 10 * fuel_consumption_per_minute* fuel_energy_density
energy_12_ac = time_to_destination[3] * 12 * fuel_consumption_per_minute* fuel_energy_density
print(f"Consumed energy for file ac in kWH: {[float(energy_6_ac), float(energy_8_ac), float(energy_10_ac), float(energy_12_ac)]}")


# Example energy savings data
energy_tugs = consumed_energy_5  # H8 aircraft using tugs
energy_self_moving = np.array([float(energy_6_ac), float(energy_8_ac), float(energy_10_ac), float(energy_12_ac)])  # Self-moving H8 aircraft
# Perform an independent t-test (one-tailed, since we're testing if tugs result in less energy consumption)
t_stat, p_value = stats.ttest_ind(energy_tugs, energy_self_moving, alternative='less')

# Print results
print(f"T-statistic: {t_stat}")
print(f"P-value: {p_value}")

# Interpret the p-value
alpha = 0.05  # Significance level
if p_value < alpha:
    print("Reject the null hypothesis: H7 aircraft using tugs consume significantly less energy.")
else:
    print("Fail to reject the null hypothesis: No significant difference in energy consumption.")