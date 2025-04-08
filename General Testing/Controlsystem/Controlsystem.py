import time
import lgpio


def runAndPrintControlSystem(HANDLE, frequency, steeringController, throttleController, velocityProfile, desiredSteeringAngle, start_time):
        print("Hello")

        throttleController.update_pwm(velocityProfile, start_time)
        steeringController.update_pwm(desiredSteeringAngle)
                
        PWM_signal_throttle = throttleController.pwm
        PWM_signal_steering = steeringController.pwm
        print(f"Desired Position: {throttleController.xd:.2f} m") 
        print(f"Current Position: {throttleController.x:.2f} m")
        print(f"Desired Velocity: {throttleController.vd:.2f} m/s") 
        print(f"Current Velocity: {throttleController.v:.2f} m")
        print()  # Prints a blank line
        print(f"Desired Steering Angle: {steeringController.desired_steering_angle:.2f} degrees")
        print(f"Current Steering Angle: {steeringController.current_steering_angle:.2f} degrees")
        print(f"PWM throttle: {throttleController.pwm:.2f} %")
        print(f"PWM steering: {steeringController.pwm:.2f} %")
        # Comment

        # Generate PWM
        lgpio.tx_pwm(HANDLE, 13, frequency, PWM_signal_throttle)
        lgpio.tx_pwm(HANDLE, 12, frequency, PWM_signal_steering)



