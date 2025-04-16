# Visualize data collected during trials
# Meant to be ran on laptop, not the Pi

import matplotlib.pyplot as plt
import numpy as np

TOTAL_TIME = 4.0

# Insert/Paste Data
steerDC = []
throtDC = []
distToCenter = []

# Make Accurate Timesteps
steerTimeSteps = np.linspace(0, TOTAL_TIME, len(steerDC))
throtTimeSteps = np.linspace(0, TOTAL_TIME, len(throtDC))
centerTimeSteps = np.linspace(0, TOTAL_TIME, len(distToCenter))

# # Plot Steering
fig, axs = plt.subplots(2, 1, figsize=(8, 6))  # 2 rows, 1 column
axs[0].scatter(steerTimeSteps, steerDC)
axs[0].set_xlabel("Time (s)")
axs[0].set_ylabel("Steering DC")
axs[0].set_title("Steering DC vs. Time")
###
axs[1].scatter(centerTimeSteps, distToCenter)
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Throttle DC")
axs[1].set_title("Distance to Center vs. Time")
plt.tight_layout()  # Optional: makes sure plots don't overlap
plt.show()

# Plot Throttle
plt.figure()
plt.scatter(throtTimeSteps, throtDC)
plt.xlabel("Time (s)")
plt.ylabel("Throttle DC")
plt.title("Throttle DC vs. Time")
plt.show()


