def compute_safe_throttle(wheel_speed, est_vehicle_speed, raw_throttle,
                          slip_threshold=0.15, scaling_factor=0.7):
    epsilon = 0.1  # minimum vehicle speed to avoid div/0
    slip = (wheel_speed - est_vehicle_speed) / max(est_vehicle_speed, epsilon)

    if slip > slip_threshold:
        print(f"Slip detected: {slip:.2f}. Throttle scaled down.")
        limited_throttle = raw_throttle * scaling_factor
    else:
        limited_throttle = raw_throttle

    return limited_throttle

