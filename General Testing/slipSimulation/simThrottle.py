import numpy as np
import matplotlib.pyplot as plt
from kalman import KalmanFilter


# Simulation parameters
dt = 0.002   # 500 Hz - 2 ms/sample (IMU Rate)
total_time = 5.0
steps = int(total_time / dt)
time = np.linspace(0, total_time, steps)

# Target speed (m/s)
target_speed = 22.0

# Car dynamics
response_time = 0.5  # seconds
max_throttle = 1.0
Kp = 0.6
wheel_slip_gain = 0.6  # how much wheels overshoot when slipping
slip_threshold = 0.15
scaling_factor = 0.6

# Sensor settings - IMU update interval == dt
accel_noise_std = 3.0   # Actual Std dev of the IMU
gps_noise_std = 0.5     # TODO: Get std dev of GPS velocity
gps_update_interval = int(1 / (50*dt))  # 10 Hz
wheel_speed_update_interval = int(1 / (10*dt))  # 50 Hz

# Write at 100 Hz (10 ms)
write_interval = 5*dt

# Display interval info
print(f"IMU updating at: {dt} s.\nRPM updating at: {wheel_speed_update_interval} s.\nGPS updating at: {gps_update_interval} s.")

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
def compute_safe_throttle(wheel_speed, est_vehicle_speed, raw_throttle,
                          slip_threshold=0.15, scaling_factor=0.6):
    print(f"Wheel speed: {wheel_speed}\tspeed read in: {est_vehicle_speed}\traw throttle: {raw_throttle}")
    epsilon = 0.01
    slip = abs(wheel_speed - est_vehicle_speed) / max(wheel_speed, epsilon)
    slip_ratios.append(slip)
    if slip > slip_threshold:
        return raw_throttle * scaling_factor
    return raw_throttle


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

    # Slip limiter adjusts throttle if necessary - returns raw_throttle if no 
    safe_throttle = compute_safe_throttle(wheel_speed, kf_velocity, smoothed_throttle,
                                          slip_threshold, scaling_factor)
    
    # Ignore for now
    pwmOP = safe_throttle * 5.0

    # Car responds to throttle (simple first-order model)
    # This is target, and what we should write to our control system, but it should be real readings
    # NOTE: Should be a live reading from Kalman filter system
    accel = (safe_throttle * target_speed - velocity) / response_time

    # accel updates a lot more often than velocity
    velocity += accel * dt

    # TODO: read hall effect sensors for wheel_speed
    # Simulate wheel speed (wheel can spin faster than vehicle during high throttle) - should be when there is a difference, not throttle-based
    wheel_slip = (raw_throttle > 0.85)  # simulate slip condition when throttle is high
    if wheel_slip:
        wheel_speed = velocity + wheel_slip_gain * (raw_throttle - 0.8)
    else:
        wheel_speed = velocity

    # Simulate noisy acceleration based on accel_noise_std
    noisy_accel = accel + np.random.normal(0, accel_noise_std)
    kf.predict(noisy_accel)

    # Update velocity at gps interval
    if step % gps_update_interval == 0:
        noisy_gps = velocity + np.random.normal(0, gps_noise_std)
        kf.update(noisy_gps)

    # Log data
    actual_velocities.append(velocity)
    wheel_speeds.append(wheel_speed)
    kf_estimates.append(kf_velocity)
    throttles.append(safe_throttle)
    raw_throttles.append(raw_throttle)

# Plot results
plt.figure(figsize=(12, 6))
plt.subplot(3, 1, 1)
plt.plot(time, actual_velocities, label="Actual Speed")
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
