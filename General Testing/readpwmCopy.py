import lgpio
import time

# GPIO pin definitions
INPUT_PIN = 26  # Change to your input pin number
# OUTPUT_PIN = 24  # Change to your output pin number - PWM when implemented

# GPIO setup
chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(chip, INPUT_PIN)
# lgpio.gpio_claim_output(chip, OUTPUT_PIN)

def read_pwm_duty_cycle(pin):
    # Measure pulse width to calculate duty cycle
    high_time = lgpio.gpio_read_pulse_length(chip, pin, 1)  # Measure HIGH pulse length
    low_time = lgpio.gpio_read_pulse_length(chip, pin, 0)   # Measure LOW pulse length

    if high_time + low_time == 0:
        return 0  # Avoid divide-by-zero if no valid PWM detected
    
    duty_cycle = high_time / (high_time + low_time)  # Duty cycle in decimal
    print(f"DC: {duty_cycle}")
    return duty_cycle


if __name__ == "__main__":
    ctr = 0.0
    try:
        while ctr < 15.0:
            duty_cycle = read_pwm_duty_cycle(INPUT_PIN)
            print(f"Duty Cycle: {duty_cycle:.2f}%")


            # Currently just writes high or low - gonna need to change
            '''
            if duty_cycle < 12.0:  # Threshold condition
                lgpio.gpio_write(chip, OUTPUT_PIN, 1)  # Turn on output if duty cycle drops
            else:
                lgpio.gpio_write(chip, OUTPUT_PIN, 0)  # Turn off output otherwise
            '''

            time.sleep(0.5)  # Polling interval
            ctr += 0.5  # Ctr is seconds
    finally:
        lgpio.gpiochip_close(chip)




import lgpio
import time

# GPIO pin definitions
INPUT_PIN = 26  # Change to your input pin number
# OUTPUT_PIN = 24  # Change to your output pin number

# GPIO setup
chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(chip, INPUT_PIN)
# lgpio.gpio_claim_output(chip, OUTPUT_PIN)

# Store timestamps and pulse duration
high_start_time = 0
high_duration = 0
low_duration = 0
period = 0.01  # Expected period (100 Hz, adjust if different frequency)
duty_cycle = 0


def edge_callback(chip, pin, level, tick):
    global high_start_time, high_duration, low_duration, duty_cycle

    if level == 1:  # Rising edge detected
        high_start_time = tick
    elif level == 0:  # Falling edge detected
        pulse_width = lgpio.tick_diff(high_start_time, tick)
        high_duration = pulse_width
        low_duration = period * 1000000 - high_duration # 8 1,000,000 because 
        if period > 0:
            duty_cycle = (high_duration / (high_duration + low_duration)) # In decimal


# Set up callback for edge detection
RISING_EDGE = 2
lgpio.gpio_claim_alert(chip, INPUT_PIN, eFlags=RISING_EDGE)

if __name__ == '__main__':

    # Set up callback for edge detection
    RISING_EDGE = 0
    lgpio.gpio_claim_alert(chip, INPUT_PIN, eFlags=RISING_EDGE)

    ctr = 0.0
    try:
        while ctr < 15.0:
            print(f"Duty Cycle: {duty_cycle:.2f}")

            '''
            if duty_cycle < 12.0:
                lgpio.gpio_write(chip, OUTPUT_PIN, 1)
            else:
                lgpio.gpio_write(chip, OUTPUT_PIN, 0)
            '''
            ctr += 0.5
            time.sleep(0.5)  # Polling interval
    finally:
        lgpio.gpiochip_close(chip)
