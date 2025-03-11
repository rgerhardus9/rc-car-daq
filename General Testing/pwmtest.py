#! /usr/bin/env python

import lgpio
import time


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
    pin = 13  # GPIO pin to output PWM (BCM numbering)
    frequency = 100  # Frequency in Hz
    duty_cycle = 15.0  # Duty cycle in percentage

    handle = setup_pwm(chip, pin, frequency, duty_cycle)
    time.sleep(2)

    try:
        print(f"Generating PWM on GPIO {pin} with {frequency}Hz and {duty_cycle}% duty cycle.")
        print("Press Ctrl+C to stop.")

        

        duty_cycle = 15.0
        
        # START - Test full range of steering
        '''
        while (duty_cycle < 20.0):
            duty_cycle += 0.05
            print(f"Duty cycle incremented to {duty_cycle}")
            lgpio.tx_pwm(handle, pin, frequency, duty_cycle)  # Start PWM
            time.sleep(0.05)   # Sleep for 1 second
        while (duty_cycle > 10.0):
            duty_cycle -= 0.05
            print(f"Duty cycle decremented to {duty_cycle}")
            lgpio.tx_pwm(handle, pin, frequency, duty_cycle)  # Start PWM
            time.sleep(0.05)   # Sleep for 1 second
        '''
        # END - Test full range of steering
        
        # START - Test forward throttle
        while (duty_cycle < 16.5):
            duty_cycle += 0.05
            print(f"Duty cycle incremented to {duty_cycle}")
            lgpio.tx_pwm(handle, pin, frequency, duty_cycle)  # Start PWM
            time.sleep(0.05)   # Sleep for 1 second
        # END - Test forward throttle

        #lgpio.tx_pwm(handle, pin, frequency, 10.3)  # Start PWM
        #print("Reverse/break")
        #time.sleep(1)   # Sleep for 1 second
        lgpio.tx_pwm(handle, pin, frequency, 15.0)  # Reset to neutral
        print("Back to neutral")
        time.sleep(2)



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
        
        


        #time.sleep(1)  # Keep the script running
    except KeyboardInterrupt:
        print("\nStopping PWM and cleaning up GPIO.")
    finally:
        lgpio.tx_pwm(handle, pin, 0, 0)  # Stop PWM
        lgpio.gpiochip_close(handle)  # Close GPIO chip

if __name__ == "__main__":
    main()
