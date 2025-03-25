#! /usr/bin/env python

import lgpio # type: ignore
import time
import Controlsystem
import Controller


state = 'N'
prevState = 'N'
running = True

def setup_pwm(chip, pin, frequency, duty_cycle):
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
    handle = lgpio.gpiochip_open(chip)  # Open GPIO chip
    lgpio.gpio_claim_output(handle, pin)  # Claim pin as output
    lgpio.tx_pwm(handle, pin, frequency, duty_cycle)  # Start PWM
    return handle

def main():
    chip = 0  # GPIO chip (default is 0 for the Raspberry Pi main GPIO)
    pin_throttle = 13  # GPIO pin to output PWM (BCM numbering)
    pin_steering = 2 # GPIO pin to output PWM
    frequency = 100  # Frequency in Hz
    duty_cycle_throttle = Controlsystem.PWM_signal_throttle  # throttle duty cycle in percentage 
    duty_cycle_steering = Controlsystem.PWM_signal_steering  # steering duty cycle in percentage

    handle = setup_pwm(chip, pin_throttle, frequency, duty_cycle_throttle)
    handle = setup_pwm(chip,pin_steering,frequency, duty_cycle_steering)

    try:
        print(f"Generating PWM on GPIO {pin_throttle} with {frequency}Hz and {duty_cycle_throttle}% duty cycle.")
        print("Press Ctrl+C to stop.")

        #duty_cycle_throttle = 15.5     # is reveiced from Controllsystem
        #while (duty_cycle_throttle < 16.0):
            #duty_cycle += 0.05
            #print(f"Duty cycle incremented to {duty_cycle_throttle}")
            # lgpio.gpio_claim_output(handle, pin)  # Claim pin as output - Only have to do once
        lgpio.tx_pwm(handle, pin_throttle, frequency, duty_cycle_throttle)  # Start PWM
        time.sleep(0.15)   # Sleep for 1 second
           
       

        lgpio.gpio_claim_output(handle, pin_throttle)  # Claim pin as output
        lgpio.tx_pwm(handle, pin_throttle, frequency, 10.3)  # Start PWM
        print("Full Reverse")
        time.sleep(5)   # Sleep for 1 second


        # lgpio.tx_pwm(handle, pin, frequency, 15.3)  # Start PWM
        # print("Neutral")
        # time.sleep(5)   # Sleep for 1 second
        # lgpio.tx_pwm(handle, pin, frequency, 13.5)  # Start PWM
        # print("Backwards")
        # time.sleep(4)
       

        # Duty cycle >= 16.0
        # while(duty_cycle > 14.0):
        #     duty_cycle -= 0.2
        #     print(f"Duty cycle decremented to {duty_cycle}")
        #     lgpio.gpio_claim_output(handle, pin)  # Claim pin as output
        #     lgpio.tx_pwm(handle, pin, frequency, duty_cycle)  # Start PWM
        #     time.sleep(1)   # Sleep for 1 second
       
        # while (duty_cycle < 15.0):
        #     duty_cycle += 0.1
        #     print(f"Duty cycle incremented to {duty_cycle}")
        #     lgpio.gpio_claim_output(handle, pin)  # Claim pin as output
        #     lgpio.tx_pwm(handle, pin, frequency, duty_cycle)  # Start PWM
        #     time.sleep(0.2)

           
        # if state == 'N':
        #     if (prevState == 'D'):
        #         duty_cycle -= 0.2
        #         handle = setup_pwm(chip, pin, frequency, duty_cycle)
        #         if (duty_cycle <= 14.4):




        #     duty_cycle = 16.0   # Drive
        #     handle = setup_pwm(chip, pin, frequency, duty_cycle)
        #     state = 'D'           # Drive
       
        # elif state == 'D':
        #     duty_cycle -= 0.2       # Slow down
        #     handle = setup_pwm(chip, pin, frequency, duty_cycle)
        #     if (duty_cycle - 0.1) <= 15.0:
        #         state = 'N'
        #         prevState = 'D'
       
       


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


            # lgpio.tx_pwm(handle, pin, frequency, 15.3)  # Start PWM
            # print("Neutral")
            # time.sleep(5)   # Sleep for 1 second
            # lgpio.tx_pwm(handle, pin, frequency, 13.5)  # Start PWM
            # print("Backwards")
            # time.sleep(4)
        

            # Duty cycle >= 16.0
            # while(duty_cycle > 14.0):
            #     duty_cycle -= 0.2
            #     print(f"Duty cycle decremented to {duty_cycle}")
            #     lgpio.gpio_claim_output(handle, pin)  # Claim pin as output
            #     lgpio.tx_pwm(handle, pin, frequency, duty_cycle)  # Start PWM
            #     time.sleep(1)   # Sleep for 1 second
        
            # while (duty_cycle < 15.0):
            #     duty_cycle += 0.1
            #     print(f"Duty cycle incremented to {duty_cycle}")
            #     lgpio.gpio_claim_output(handle, pin)  # Claim pin as output
            #     lgpio.tx_pwm(handle, pin, frequency, duty_cycle)  # Start PWM
            #     time.sleep(0.2)

            
            # if state == 'N':
            #     if (prevState == 'D'):
            #         duty_cycle -= 0.2
            #         handle = setup_pwm(chip, pin, frequency, duty_cycle)
            #         if (duty_cycle <= 14.4):




            #     duty_cycle = 16.0   # Drive
            #     handle = setup_pwm(chip, pin, frequency, duty_cycle)
            #     state = 'D'           # Drive
        
            # elif state == 'D':
            #     duty_cycle -= 0.2       # Slow down
            #     handle = setup_pwm(chip, pin, frequency, duty_cycle)
            #     if (duty_cycle - 0.1) <= 15.0:
            #         state = 'N'
            #         prevState = 'D'
        
        


            time.sleep(1)  # Keep the script running
    except KeyboardInterrupt:
            print("\nStopping PWM and cleaning up GPIO.")
    finally:
            lgpio.tx_pwm(handle, pin_steering, 0, 0)  # Stop PWM
            lgpio.gpiochip_close(handle)  # Close GPIO chip


    if __name__ == "__main__":
        main()


