import numpy as np
import matplotlib.pyplot as plt

# Simulation to show the angle deviation when taking a turn at max speed without tipping


# Angle lost based on turn radius (MAX SPEED)
# Distance slipped straight
d = 4.0  # meters
radii = np.linspace(1, 20, 100)
theta_rad = 2 * np.arcsin(d / (2 * radii))
theta_deg = np.degrees(theta_rad)

# Max radius without tipping - radius based on velocity
H = 0.08    # car's COG height (m)
g = 9.81
t = 0.332   # track width (m)

v = np.linspace(0, 20.1, 100)
r = (v**2 * H) / (g * (t / 2))

# Get Angle Deviation based on speed and max turn radius
# d will change

# v, r, theta_deg

rpm = v*60/(2*3.14*0.0821)  # m/s to rpm

slipDistance = (v/10.0)*2   # m/s -> m/100ms -> x2 for worst case (200ms)
theta_rad2 = 2 * np.arcsin(slipDistance / (2 * r))
theta_deg2 = np.degrees(theta_rad2)


# Subplot 1: Max Turn Radius vs Speed (no tipping)
plt.figure()
plt.plot(v, r, color='blue')
plt.title("Max Turn Radius Without Tipping")
plt.xlabel("Velocity (m/s)")
plt.ylabel("Turn Radius (m)")
plt.grid(True)
plt.show()

# Subplot 1: Max Turn Radius vs Speed (no tipping)
plt.figure()
plt.plot(v, slipDistance, color='blue')
plt.title("Max Slip Distance vs. Velocity")
plt.xlabel("Velocity (m/s)")
plt.ylabel("Slip Distance (m)")
plt.grid(True)
plt.show()

# Subplot 2: Angle Deviation vs Turn Radius (fixed 4m slip)
plt.figure()
plt.plot(radii, theta_deg2, color='purple')
plt.title("Angle Deviation vs Turn Radius\n(At Max Velocity of Given Radius)")
plt.xlabel("Turn Radius (m)")
plt.ylabel("Angle Deviation (degrees)")
plt.axhline(y=55, color='black', linestyle='--', label='Camera FOV Acceptable Threshold')
plt.grid(True)
plt.legend()
plt.show()



'''
# Subplot 2: Angle Deviation vs Turn Radius (fixed 4m slip)
axs[1].plot(radii, theta_deg, label='Angle Deviation', color='purple')
axs[1].set_title("Angle Deviation vs Turn Radius\n(4m Slip Straight - Max velocity)")
axs[1].set_xlabel("Turn Radius (m)")
axs[1].set_ylabel("Angle Deviation (degrees)")
axs[1].grid(True)
axs[1].legend()
'''





