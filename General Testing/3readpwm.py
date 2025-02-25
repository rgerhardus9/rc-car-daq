import lgpio
import time

PWM_INPUT_PIN = 6  # GPIO pin for the PWM input
CHIP = 0            # Default GPIO chip

class PWMReader:
    def __init__(self, gpio_chip, gpio_pin):
        self.gpio_chip = gpio_chip
        self.gpio_pin = gpio_pin
        self.high_time = 0
        self.low_time = 0
        self.last_timestamp = 0
        self.last_level = None
        self.handle = lgpio.gpiochip_open(self.gpio_chip)

        # Claim the pin for input with alerts for both rising and falling edges
        # Think BOTH_EDGES may not be a valid parameter
        lgpio.gpio_claim_alert(self.handle, self.gpio_pin, lgpio.BOTH_EDGES)

        # Initialize timing
        self.start_time = time.time()

    def alert_callback(self, gpio, level, timestamp):
        """
        Callback triggered on GPIO level changes.
        :param gpio: GPIO pin
        :param level: Pin level (1 for high, 0 for low)
        :param timestamp: Event timestamp in nanoseconds
        """
        if self.last_timestamp == 0:
            self.last_timestamp = timestamp
            self.last_level = level
            return

        # Pulse duration of either high or low
        pulse_duration_ns = timestamp - self.last_timestamp

        # Update high and low time variables
        if self.last_level == 1:
            self.high_time += pulse_duration_ns
        else:
            self.low_time += pulse_duration_ns

        # Update state
        self.last_timestamp = timestamp
        self.last_level = level

    def measure_duty_cycle(self, duration=1.0):
        """Measure duty cycle over the specified duration."""
        self.high_time = 0
        self.low_time = 0
        self.last_timestamp = 0
        self.last_level = None

        # Attach the callback function
        lgpio.callback(self.handle, self.gpio_pin, lgpio.BOTH_EDGES, self.alert_callback)

        # Wait for the duration
        time.sleep(duration)

        # Detach the callback after the duration
        lgpio.cancel_callback(self.handle)

        # Compute duty cycle
        total_time_ns = self.high_time + self.low_time
        if total_time_ns == 0:
            return 0.0

        duty_cycle = (self.high_time / total_time_ns) * 100
        return duty_cycle

    def cleanup(self):
        """Clean up GPIO resources."""
        lgpio.gpiochip_close(self.handle)


if __name__ == "__main__":
    pwm_reader = PWMReader(CHIP, PWM_INPUT_PIN)

    try:
        print("Measuring PWM duty cycle...")
        duty_cycle = pwm_reader.measure_duty_cycle(duration=1.0)
        print(f"Duty Cycle: {duty_cycle:.2f}%")
    except KeyboardInterrupt:
        print("Measurement interrupted by user.")
    finally:
        pwm_reader.cleanup()
