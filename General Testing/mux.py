#! /usr/bin/env python

# Implementation of reading and sending control signal to MUX
# Currently not doing in parallel, rather in series with some latency introduced
# Also: Need some time to determine DC? - Prolly at least 50 ms to be accurate

import lgpio
import time

# GPIO pin definitions
INPUT_PIN = 6 # Pin 31
SELECT_PIN = 25 # Pin 22 ==> LOW = Y0 (transmitter), HIGH = Y1 (generated PWM)
THROTTLE_GENERATE_PIN = 13 # Pin 33
STEERING_GENERATE_PIN = 12 # Pin 32
# 3.3 V Power from Pi to MUX = Pin 1/17

GPIO_CHIP = 0
PERIOD = 0.01   # 10 ms
FREQUENCY = 100 # Hz

# GPIO initialization
HANDLE = lgpio.gpiochip_open(GPIO_CHIP)   # Always do this
# SELECT
lgpio.gpio_claim_output(HANDLE, SELECT_PIN)  # Claim pin as OUTPUT
lgpio.gpio_write(HANDLE, SELECT_PIN, 1)     # Init to HIGH - Program Generated PWMs
# PWM INPUT/READ
lgpio.gpio_claim_input(HANDLE, INPUT_PIN, lgpio.SET_PULL_DOWN)  # Claim pin as INPUT
# PWM Generation
lgpio.gpio_claim_output(HANDLE, THROTTLE_GENERATE_PIN)
lgpio.gpio_claim_output(HANDLE, STEERING_GENERATE_PIN)




# duration currently in seconds
def read_pwm_duty_cycle(chip, INPUT_PIN, duration = 0.3):   # 0.3s to calculate DC and switch MUX select

    print("Measuring PWM duty cycle...")
    # Run until DC is outside of range
    init_time = time.time()    # Safety feature doing this instead of while(True)
    while(time.time() - init_time < 5.0):
        # Initialize times
        high_time = 0
        low_time = 0
        start_time = time.time()    # time in seconds since epoch

        # Initialize level and time
        previous_level = lgpio.gpio_read(chip, INPUT_PIN)
        # print(f"Previous level: {previous_level}")

        pulse_start = time.time()
        try:
            while (time.time() - start_time) < duration:
                # print(f"Time: {(time.time() - start_time):.5f}")

                # This should work with the input changing - reading high or low of the PWM
                current_level = lgpio.gpio_read(chip, INPUT_PIN)       
                # print(f"Current level: {current_level}")
            
                # Detect level change
                if current_level != previous_level:
                    pulse_end = time.time()
                    pulse_duration = pulse_end - pulse_start

                    if previous_level == 1:
                        high_time += pulse_duration
                    else:
                        low_time += pulse_duration
                    
                    # Update state
                    previous_level = current_level
                    pulse_start = pulse_end
                
                time.sleep(0.00001)  # For even increments

        finally:
            total_time = high_time + low_time
            
            # No division by zero
            if total_time == 0:
                return 0
            
            # Duty cycle in percent
            dc = (high_time / total_time) * 100
            #print(f"DC: {dc}")

            # Change MUX signal or continue - do this in or out of function?
            if (dc < 13.0):
                lgpio.gpio_write(chip, SELECT_PIN, 0)     # Select signal to LOW = PWM transmitter signal
                return 0
            else:
                continue
    
    return 1


"""
Sets up PWM output on a GPIO pin.

Args:
    chip (int): GPIO chip (0 for the Raspberry Pi main GPIO).
    pin (int): GPIO pin number (BCM mode).
    frequency (int): Frequency of the PWM in Hz.
    throttle_dc (float): Duty cycle in percentage (0.0 to 100.0).
Returns:
    Nothing.
"""
def send_pwm(chip, THROTTLE_GENERATE_PIN, FREQUENCY, throttle_dc, duration=5.0):
    # Initialize a steering PWM value
    steering_dc = 15.0
    lgpio.tx_pwm(chip, STEERING_GENERATE_PIN, FREQUENCY, steering_dc)

    # Init to neutral/stop
    # Initialize PWM at neutral
    lgpio.tx_pwm(chip, THROTTLE_GENERATE_PIN, FREQUENCY, 15.0)
    time.sleep(2)

    # Write/increment PWM
    start_time = time.time()
    lgpio.tx_pwm(chip, THROTTLE_GENERATE_PIN, FREQUENCY, throttle_dc)  # Constant right now so outside loop
    lgpio.tx_pwm(chip, STEERING_GENERATE_PIN, FREQUENCY, steering_dc)  # Constant right now so outside loop


    while (time.time() - start_time < duration):
        try:
            print(f"Generating PWM on GPIO {THROTTLE_GENERATE_PIN} with {FREQUENCY}Hz and {throttle_dc}% duty cycle.")
            print("Press Ctrl+C to stop.")

            # Write pwm - if this is constant, do it outside of the while loop
            # lgpio.tx_pwm(chip, THROTTLE_GENERATE_PIN, FREQUENCY, throttle_dc)  # Start PWM

            # Modify steering values (testing)
            '''
            if (steering_dc < 19.5):
                steering_dc += 3.0
            else:
                steering_dc = 15.0
            lgpio.tx_pwm(chip, STEERING_GENERATE_PIN, FREQUENCY, steering_dc)
            print(f"Steering PWM: {steering_dc}")
            '''

            # Test incrementing the duty cycle - won't read the receiver DC during this time
            '''
            while (throttle_dc < 16.7):
                throttle_dc += 0.05
                print(f"Duty cycle incremented to {throttle_dc}")
                lgpio.tx_pwm(chip, THROTTLE_GENERATE_PIN, FREQUENCY, throttle_dc)  # Start PWM
                time.sleep(0.05)   # Sleep for 1 second
            '''

            # Read PWM and change MUX if needed
            select = read_pwm_duty_cycle(chip, INPUT_PIN)

            if (select == 0):
                print("Returning 0 from send_pwm() function")
                return 0
            else:
                continue
            

        except KeyboardInterrupt:
            print("\nStopping PWM and cleaning up GPIO.")
    
    # None of this happens if we have taken manual control over (select == 0)

    # Return to neutral
    lgpio.tx_pwm(chip, THROTTLE_GENERATE_PIN, FREQUENCY, 15.0)  # Return to neutral
    time.sleep(2.0)

    # Stop sending any PWM signal
    lgpio.tx_pwm(chip, THROTTLE_GENERATE_PIN, 0, 0)

    # Don't close GPIO chip
    # Give control back to transmitter either way
    lgpio.gpio_write(chip, SELECT_PIN, 0)     # Select signal to LOW = PWM transmitter signal
    return 0


if __name__ == "__main__":

    dc_to_send = 15.9

    print(f"Sending a PWM value of {dc_to_send} percent to pin {THROTTLE_GENERATE_PIN}.")
    send_pwm(HANDLE, THROTTLE_GENERATE_PIN, FREQUENCY, dc_to_send, duration=15.0)
    
    # Close the chip
    print("Exited function and closing the chip.")
    lgpio.gpiochip_close(HANDLE)   # Always do this
