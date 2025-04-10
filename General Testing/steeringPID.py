import numpy as np

# PID gains (to be tuned)
Kp = 0.5
Ki = 0.01
Kd = 0.1

# State for integration and derivative
integral = 0.0
last_error = 0.0

def pid_steering(error, dt):
    global integral, last_error

    integral += error * dt
    derivative = (error - last_error) / dt
    last_error = error

    output = Kp * error + Ki * integral + Kd * derivative
    return np.clip(output, -1.0, 1.0)  # Limit output to safe steering range


# How we would call this in our function
'''
steering_correction = pid_steering(centerline_offset, dt)
servo_pwm = steering_correction_to_pwm(steering_correction)  # convert to actual output
'''

