import pandas as pd
import matplotlib.pyplot as plt

# Load your data (adjust path if needed)
df = pd.read_csv("convergence_analysis_scenario_A\convergence_metrics_A.csv")  # Or use full path if not in the same folder

# Create the plot
plt.figure(figsize=(7, 5))
plt.plot(df["runs"], df["cov"], label="Scenario A Coefficient of Variation", color="blue")

# Style the plot like the reference image
plt.title("Coefficient of Variation Across Simulations (Scenario A)")
plt.xlabel("Number of Simulations")
plt.ylabel("Coefficient of Variation")
plt.grid(True)
plt.legend()
plt.tight_layout()

# Show or save the plot
plt.savefig("convergence_analysis_scenario_A\cov_convergence_scenario_A.png")
plt.show()
