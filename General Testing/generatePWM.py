import lgpio
import time

# Define constants
GPIO_CHIP = "gpiochip0"  # gpiochip0 is the default GPIO chip on Raspberry Pi
GPIO_PIN = 13  # GPIO Pin 13
PWM_FREQUENCY = 100  # Set PWM frequency to 1 kHz
DUTY_CYCLE = 15.0  # 15% duty cycle


def setup_pwm():
    try:
        # Open the GPIO chip and set the GPIO pin as an output
        handle = lgpio.gpiochip_open(GPIO_CHIP)
        lgpio.gpio_claim_output(handle, GPIO_PIN)
        return handle
    except Exception as e:
        print(f"Error setting up GPIO PWM: {e}")
        return None


def run_pwm(handle):
    try:
        while True:
            # Set GPIO to HIGH for the duty cycle on time
            lgpio.gpio_write(handle, GPIO_PIN, 1)
            time.sleep(duty_cycle_on_time)
            
            # Set GPIO to LOW for the remainder of the period
            lgpio.gpio_write(handle, GPIO_PIN, 0)
            time.sleep((1 / PWM_FREQUENCY) - duty_cycle_on_time)
    except KeyboardInterrupt:
        print("\nPWM stopped by user.")
    finally:
        lgpio.gpiochip_close(handle)


def main():
    handle = setup_pwm()
    if handle:
        print("Starting PWM output on GPIO Pin 13 with 15% duty cycle.")
        run_pwm(handle)
    else:
        print("Failed to initialize GPIO chip.")


if __name__ == "__main__":
    main()