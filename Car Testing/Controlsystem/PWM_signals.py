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
desired_dist_to_centerline = 0

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
                                            #Kp         Ki         Kd  
    steeringController = SteeringController(0.061388 , 0.682021 , 0.0)              
    throttleController = ThrottleController(1.0 , 1.0 , 1.0)

    velocityProfile = VelocityProfile(D_TOTAL, V_STEADY, ACCEL)

    lgpio.tx_pwm(HANDLE, pin_throttle, frequency, 15.0)
    lgpio.tx_pwm(HANDLE, pin_steering, frequency, 15.0)
    time.sleep(2.0)
    
    while throttleController.x <= D_TOTAL:

        Controlsystem.runAndPrintControlSystem(HANDLE, frequency, steeringController, throttleController, velocityProfile, desired_dist_to_centerline)

        '''
        try:
            print(f"Generating PWM on GPIO {pin_throttle} with {frequency}Hz and {duty_cycle_throttle}% duty cycle.")
            print("Press Ctrl+C to stop.")

            #duty_cycle_throttle = 15.5     # is reveiced from Controllsystem
            #while (duty_cycle_throttle < 16.0):
                #duty_cycle += 0.05
                #print(f"Duty cycle incremented to {duty_cycle_throttle}")
                # lgpio.gpio_claim_output(handle, pin)  # Claim pin as output - Only have to do once
            lgpio.tx_pwm(HANDLE, pin_throttle, frequency, duty_cycle_throttle)  # Start PWM
            time.sleep(0.15)   # Sleep for 1 second
            
        

            lgpio.gpio_claim_output(handle, pin_throttle)  # Claim pin as output
            lgpio.tx_pwm(handle, pin_throttle, frequency, 10.3)  # Start PWM
            print("Full Reverse")
            time.sleep(5)   # Sleep for 1 second
        
        


            time.sleep(1)  # Keep the script running
        except KeyboardInterrupt:
            print("\nStopping PWM and cleaning up GPIO.")
        finally:
            lgpio.tx_pwm(handle, pin_throttle, 0, 0)  # Stop PWM
            lgpio.gpiochip_close(handle)  # Close GPIO chip

        # steering
        try:
                print(f"Generating PWM on GPIO {pin_steering} with {frequency}Hz and {duty_cycle_steering}% duty cycle.")
                print("Press Ctrl+C to stop.")

                #duty_cycle_throttle = 15.5     # is reveiced from Controllsystem
                #while (duty_cycle_throttle < 16.0):
                    #duty_cycle += 0.05
                    #print(f"Duty cycle incremented to {duty_cycle_throttle}")
                    # lgpio.gpio_claim_output(handle, pin)  # Claim pin as output - Only have to do once
                lgpio.tx_pwm(handle, pin_steering, frequency, duty_cycle_steering)  # Start PWM
                time.sleep(0.15)   # Sleep for 1 second
                
            

                lgpio.gpio_claim_output(handle, pin_steering)  # Claim pin as output
                lgpio.tx_pwm(handle, pin_steering, frequency, 10.3)  # Start PWM
                print("Full Reverse")
                time.sleep(5)   # Sleep for 1 second


            
            


                time.sleep(1)  # Keep the script running
        except KeyboardInterrupt:
                print("\nStopping PWM and cleaning up GPIO.")
        finally:
                lgpio.tx_pwm(handle, pin_steering, 0, 0)  # Stop PWM
                lgpio.gpiochip_close(handle)  # Close GPIO chip

        '''

    # Return to neutral
    lgpio.tx_pwm(HANDLE, pin_throttle, frequency, 15.0)
    lgpio.tx_pwm(HANDLE, pin_steering, frequency, 15.0)
    time.sleep(2.0)
    lgpio.gpiochip_close(HANDLE)


if __name__ == "__main__":
    main()


