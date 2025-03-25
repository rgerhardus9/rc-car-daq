import time
import lgpio
# import PWM_signals

#initializeCamera()
# global PWM_signal_steering
# global PWM_signal_throttle
# V_STEADY = 20.5     # Max velocity (m/s)
# ACCEL = 10.25    # Max acceleration (m/s^2)
# D_TOTAL   = 82.0   # Total displacement (m)

# PWM_signal_throttle = 15
# PWM_signal_steering = 15
#                                         #Kp         Ki         Kd  
# steeringController = SteeringController(0.061388 , 0.682021 , 0.0)
# throttleController = ThrottleController(1.0 , 1.0 , 1.0)


def runAndPrintControlSystem(HANDLE, frequency, steeringController, throttleController, velocityProfile):
        print("Hello")
        start_time = time.time()

        throttleController.update_pwm(start_time, velocityProfile)
        steeringController.update_pwm(start_time, velocityProfile)


        
                
        PWM_signal_throttle = throttleController.pwm
        PWM_signal_steering = steeringController.pwm
        print(f"Desired Position: {throttleController.xd:.2f} m") 
        print(f"Current Position: {throttleController.x:.2f} m")
        print(f"Desired Velocity: {throttleController.v:.2f} m/s") 
        print(f"Current Velocity: {throttleController.x:.2f} m")
        print()  # Prints a blank line
        print(f"Desired Steering Angle: {steeringController.desired_steering_angle:.2f} degrees")
        print(f"Current Steering Angle: {steeringController.current_steering_angle:.2f} degrees")


        # Generate PWM
        lgpio.tx_pwm(HANDLE, 13, frequency, PWM_signal_throttle)
        lgpio.tx_pwm(HANDLE, 12, frequency, PWM_signal_steering)



