import lgpio
import time

# GPIO pin definitions
INPUT_PIN = 6  # Change to your input pin number
GPIO_CHIP = 0
PERIOD = 0.01   # 10 ms
# GPIO setup - CHANGE INPUT AND OUTPUT HERE
chip = lgpio.gpiochip_open(GPIO_CHIP)
lgpio.gpio_claim_input(chip, INPUT_PIN, lgpio.SET_PULL_DOWN)    # Set pull down so it's low when no signal is present
# lgpio.gpio_claim_output(chip, INPUT_PIN)

# Testing
# lgpio.gpio_write(chip, INPUT_PIN, 1)

# duration currently in seconds
def read_pwm_duty_cycle(INPUT_PIN, chip, duration = 1.0):
    # Initialize times
    high_time = 0
    low_time = 0
    start_time = time.time()    # time in seconds since epoch

    # Initialize level and time
    previous_level = lgpio.gpio_read(chip, INPUT_PIN)
    print(f"Previous level: {previous_level}")

    pulse_start = time.time()
    switchoff = 0
    try:
        while (time.time() - start_time) < duration:
            # print(f"Time: {(time.time() - start_time):.5f}")

            # This should work with the input changing - reading high or low of the PWM
            current_level = lgpio.gpio_read(chip, INPUT_PIN)
            
            
            print(f"Current level: {current_level}")

            
            # Testing - switching pin between low and hi - don't do when reading - we shouldn't have to write at all
            # Works with input and output configurations
            # DC should be about 50% - the time.sleep() works well to help that so that samples are taken at even intervals
            '''
            if switchoff == 0:
                lgpio.gpio_write(chip, INPUT_PIN, 1)    
                switchoff = 1
                time.sleep(0.0001)    # Makes much more accurate for testing
            else:
                lgpio.gpio_write(chip, INPUT_PIN, 0)    
                switchoff = 0
                time.sleep(0.0001)
            '''
            



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
        lgpio.gpiochip_close(chip)
    
    total_time = high_time + low_time

    # No division by zero
    if total_time == 0:
        return 0
    
    # Duty cycle in percent
    dc = (high_time / total_time) * 100
    return dc


if __name__ == "__main__":
    print("Measuring PWM duty cycle...")
    duty_cycle = read_pwm_duty_cycle(INPUT_PIN, chip)
    print(f"After 1 second, duty cycle is: {duty_cycle:.2f}")
