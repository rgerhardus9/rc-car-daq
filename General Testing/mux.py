#! /usr/bin/env python

# Implementation of reading and sending control signal to MUX
# Currently not doing in parallel, rather in series with some latency introduced
# Also: Need some time to determine DC? - Prolly at least 50 ms to be accurate

import lgpio
import time

# GPIO pin definitions
INPUT_PIN = 6
SELECT_PIN = 25 # LOW = Y0 (transmitter), HIGH = Y1 (generated PWM)
PWM_GENERATE_PIN = 13

GPIO_CHIP = 0
PERIOD = 0.01   # 10 ms
FREQUENCY = 100 # Hz

# GPIO initialization
HANDLE = lgpio.gpiochip_open(GPIO_CHIP)   # Always do this
# SELECT
lgpio.gpio_claim_output(HANDLE, SELECT_PIN)  # Claim pin as OUTPUT
lgpio.gpio_write(HANDLE, SELECT_PIN, 1)     # Init to HIGH
# PWM INPUT/READ
lgpio.gpio_claim_input(HANDLE, INPUT_PIN, lgpio.SET_PULL_DOWN)  # Claim pin as INPUT
# PWM Generation
lgpio.gpio_claim_output(HANDLE, PWM_GENERATE_PIN)    # Set pull down so it's low when no signal is present




# duration currently in seconds
def read_pwm_duty_cycle(chip, INPUT_PIN, duration = 0.3):   # 0.5 second delay before it will switch the MUX select

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
            print(f"DC: {dc}")

            # Change MUX signal or continue - do this in or out of function?
            if (dc < 14.0 or dc > 16.0):
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
    duty_cycle (float): Duty cycle in percentage (0.0 to 100.0).
Returns:
    Nothing.
"""
def send_pwm(chip, PWM_GENERATE_PIN, FREQUENCY, duty_cycle, duration=5.0):
    # Init to neutral/stop
    # Initialize PWM at neutral
    lgpio.tx_pwm(chip, PWM_GENERATE_PIN, FREQUENCY, 15.0)
    time.sleep(2)

    # Write/increment PWM
    start_time = time.time()
    while (time.time() - start_time < duration):
        try:
            print(f"Generating PWM on GPIO {PWM_GENERATE_PIN} with {FREQUENCY}Hz and {duty_cycle}% duty cycle.")
            print("Press Ctrl+C to stop.")

            # Write pwm
            lgpio.tx_pwm(chip, PWM_GENERATE_PIN, FREQUENCY, duty_cycle)  # Start PWM

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
    lgpio.tx_pwm(chip, PWM_GENERATE_PIN, FREQUENCY, 15.0)  # Return to neutral
    time.sleep(2.0)

    # Stop sending any PWM signal
    lgpio.tx_pwm(chip, PWM_GENERATE_PIN, 0, 0)

    # Don't close GPIO chip
    # Give control back to transmitter either way
    return 0


if __name__ == "__main__":

    dc_to_send = 15.4

    print(f"Sending a PWM value of {dc_to_send} percent to pin {PWM_GENERATE_PIN}.")
    send_pwm(HANDLE, PWM_GENERATE_PIN, FREQUENCY, dc_to_send)
    
    # Close the chip
    print("Exited function and closing the chip.")
    lgpio.gpiochip_close(HANDLE)   # Always do this





