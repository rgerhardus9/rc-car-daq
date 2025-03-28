#! /usr/bin/env python

import lgpio # type: ignore
import time
import Controlsystem
from Controller import ThrottleController
from Controller import SteeringController
from VelocityProfile import VelocityProfile


state = 'N'
prevState = 'N'
running = True

global PWM_signal_steering
global PWM_signal_throttle
# Update this value with the distance to the centerline from the camera
desiredSteeringAngle = 20

HANDLE = lgpio.gpiochip_open(0)

def setup_pwm(HANDLE, pin):
    """
    Sets up PWM output on a GPIO pin.
   
    Args:
        chip (int): GPIO chip (0 for the Raspberry Pi main GPIO).
        pin (int): GPIO pin number (BCM mode).
        frequency (int): Frequency of the PWM in Hz.
        duty_cycle (float): Duty cycle in percentage (0.0 to 100.0).
    Returns:
        handle: PWM handle.
    """
    lgpio.gpio_claim_output(HANDLE, pin)  # Claim pin as output

def main():
    print("Running")
    pin_throttle = 13  # GPIO pin to output PWM (BCM numbering)
    pin_steering = 12 # GPIO pin to output PWM


    frequency = 100  # Frequency in Hz
    setup_pwm(HANDLE, pin_throttle)
    setup_pwm(HANDLE, pin_steering)


    # duty_cycle_throttle = Controlsystem.PWM_signal_throttle  # throttle duty cycle in percentage 
    # duty_cycle_steering = Controlsystem.PWM_signal_steering  # steering duty cycle in percentage

    V_STEADY = 20.5     # Max velocity (m/s)
    ACCEL = 10.25    # Max acceleration (m/s^2)
    D_TOTAL   = 82.0   # Total displacement (m)
                                            #      Kp              Ki            Kd  
    steeringController = SteeringController(    0.061388    ,   0.682021    ,   0.0)              
    throttleController = ThrottleController(      1.0       ,     1.0       ,   1.0)

    velocityProfile = VelocityProfile(D_TOTAL, V_STEADY, ACCEL)

    lgpio.tx_pwm(HANDLE, pin_throttle, frequency, 15.0)
    lgpio.tx_pwm(HANDLE, pin_steering, frequency, 15.0)
    time.sleep(2.0)

    
    while throttleController.x <= D_TOTAL:
        Controlsystem.runAndPrintControlSystem(HANDLE, frequency, steeringController, throttleController, velocityProfile, desiredSteeringAngle)


    # Return to neutral
    lgpio.tx_pwm(HANDLE, pin_throttle, frequency, 15.0)
    lgpio.tx_pwm(HANDLE, pin_steering, frequency, 15.0)
    time.sleep(2.0)
    lgpio.gpiochip_close(HANDLE)


if __name__ == "__main__":
    main()


