# Simulation file for a slip-detecting steering module

import numpy as np
import matplotlib.pyplot as plt
from kalman import KalmanFilter


# Simulation parameters
dt = 0.002   # 500 Hz - 2 ms/sample (IMU Rate)
total_time = 5.0
steps = int(total_time / dt)
time = np.linspace(0, total_time, steps)

# Buffer to hold throttle amounts. Pop oldest (written 100 ms later to represent mechanical delay) - FIXED LAG
mechanical_delay_steps = int(0.1 / dt)  # 100 ms - every 50 timesteps
throttle_buffer = [0.0] * mechanical_delay_steps

# Target speed (m/s)
target_speed = 19.0

# Car dynamics
response_time = 1.5  # seconds to get to max speed
max_throttle = 1.0
Kp = 0.6
wheel_slip_gain = 0.6  # how much wheels overshoot when slipping
slip_threshold = 0.15
scaling_factor = 0.9
throttle_force = 5.4    # m/s^2 at full throttle
drag = 0.2

# Sensor settings - IMU update interval == dt
accel_noise_std = 3.0   # Actual Std dev of the IMU
gps_noise_std = 0.5     # TODO: Get std dev of GPS velocity
gps_update_interval = int(1 / (50*dt))  # 10 Hz in steps
wheel_speed_update_interval = int(1 / (10*dt))  # 50 Hz in steps

# Write at 100 Hz (10 ms)
write_interval = int(1 / (5*dt))

# Display interval info
print(f"IMU updating at: {dt}s ({1/dt} Hz).\nRPM updating at: {1/wheel_speed_update_interval} s ({wheel_speed_update_interval} Hz).\nGPS updating at: {1/gps_update_interval} s ({gps_update_interval} Hz).")
print(f"Writing every {1/write_interval} s ({write_interval} Hz)")

# Initialize filter
kf = KalmanFilter(dt, process_var=0.1, gps_var=gps_noise_std**2)

# State variables
velocity = 0.0
throttle = 0.0
wheel_speed = 0.0
accel = 0.0

# Logging
actual_velocities = []
wheel_speeds = []
kf_estimates = []
throttles = []
slip_ratios = []
raw_throttles = []


### FUNCTION to reduce throttle if slipping
# wheel_speed read from encoders
# est_vehicle_speed is from Kalman filter / IMU / GPS
# raw_throttle is our output, adjusted for slip if necessary
# scaling_factor is a factor (0-1.0) for "how bad" the slip is (concrete low and ice high)
def compute_safe_throttle(wheel_speed, est_vehicle_speed, throttleToPassOrNerf,
                          slip_threshold=0.15, scaling_factor=0.6):
    # print(f"Wheel speed: {wheel_speed}\tspeed read in: {est_vehicle_speed}\traw throttle: {throttleToPassOrNerf}")
    epsilon = 0.01
    slip = np.clip((wheel_speed - est_vehicle_speed) / max(wheel_speed, epsilon), 0, 1.0)
    slip_ratios.append(slip)
    if slip > slip_threshold:
        return throttleToPassOrNerf * scaling_factor * (1 - (slip - slip_threshold))    # Reduce throttle more if more slipping
    elif (slip > slip_threshold - 0.05): # If we're really close to slip ratio
        return throttleToPassOrNerf * 0.9
    else:
        return throttleToPassOrNerf


# Throttle filter parameters - smoothing throttle
alpha = 0.1
smoothed_throttle = 0.0

for step in range(steps):

    # time
    t = step * dt

    # PID Controller with Kalman filter - P controller right now
    kf_velocity = kf.get_velocity()
    speed_error = target_speed - kf_velocity
    raw_throttle = np.clip(1 + Kp * speed_error, 0, max_throttle)   # Gets set to max_throttle at beginning because early error is large
    smoothed_throttle = alpha*raw_throttle + (1-alpha) * smoothed_throttle

    # Updates every iteration
    kf_velocity += accel * dt

    # Update velocity at gps interval
    if step % gps_update_interval == 0:
        noisy_gps = kf_velocity + np.random.normal(0, gps_noise_std)
        kf.update(noisy_gps)

    # Update wheel_speed at RPM interval
    if step % wheel_speed_update_interval == 0:
        # TODO: read hall effect sensors for wheel_speed
        # rpm_error = np.random.randint(-20, 20)  # RPM error
        rpm_error = float(np.random.randint(-173,173) / 1000.0) # RPM error in m/s (-0.173 -> 0.173)
        wheel_speed = kf_velocity + rpm_error
        # Random slippage occurring 20% of time - Range from a 0.17-1.4 m/s difference between wheel_speed and kf_velocity (and rpm_error)
        x = np.random.randint(1,21)
        y = np.random.randint(1,21)
        if (x > 16):
            wheel_speed += (x / 100.0 * y)
        safe_throttle = compute_safe_throttle(wheel_speed, kf_velocity, smoothed_throttle,
                                        slip_threshold=0.15, scaling_factor=0.6)
    else:
        # Slip limiter adjusts throttle if necessary - returns smoothed_throttle if not needed
        safe_throttle = compute_safe_throttle(wheel_speed, kf_velocity, smoothed_throttle,
                                            slip_threshold=0.25, scaling_factor=0.6)
    
    # Fill throttle buffer - get oldest
    throttle_buffer.append(safe_throttle)
    delayed_throttle = throttle_buffer.pop(0)

       
    # Car responds to throttle (simple first-order model)
    # This is target, and what we should write to our control system, but it should be real readings
    # NOTE: Should be a live reading from Kalman filter system
    accel = (delayed_throttle * target_speed - kf_velocity) / response_time

    # Simulate noisy acceleration based on accel_noise_std as a normal random distro
    noisy_accel = accel + np.random.normal(0, accel_noise_std)
    kf.predict(noisy_accel)


    # Log data
    wheel_speeds.append(wheel_speed)
    kf_estimates.append(kf_velocity)
    throttles.append(delayed_throttle)
    raw_throttles.append(raw_throttle)

# Plot results
plt.figure(figsize=(12, 6))
plt.title("Always checking for slip")
plt.subplot(3, 1, 1)
plt.plot(time, kf_estimates, label="Kalman Velocity", linestyle='--', color='r')
plt.plot(time, wheel_speeds, label='Wheel Speed', linestyle='--', color='y')
plt.ylabel("Speed (m/s)")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, throttles, label='Adjusted Throttle')
plt.plot(time, raw_throttles, label="Raw Throttle", linestyle='--', color='r')
plt.ylabel("Throttle")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, slip_ratios, label="Slip Ratio")
plt.axhline(slip_threshold, color='r', linestyle='--', label="Slip Threshold")
plt.xlabel("Time (s)")
plt.ylabel("Slip")
plt.legend()


plt.tight_layout()
plt.show()
