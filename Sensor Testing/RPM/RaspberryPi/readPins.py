import lgpio
import time

# GPIO pin definitions
INPUT_PINS = [17, 27, 22, 10] 

# GPIO initialization
HANDLE = lgpio.gpiochip_open(0)   # Always do this
# PWM INPUT/READ
lgpio.gpio_claim_input(HANDLE, INPUT_PINS[0], lgpio.SET_PULL_DOWN)  # Claim pin as INPUT
lgpio.gpio_claim_input(HANDLE, INPUT_PINS[1], lgpio.SET_PULL_DOWN)  # Claim pin as INPUT
lgpio.gpio_claim_input(HANDLE, INPUT_PINS[2], lgpio.SET_PULL_DOWN)  # Claim pin as INPUT
lgpio.gpio_claim_input(HANDLE, INPUT_PINS[3], lgpio.SET_PULL_DOWN)  # Claim pin as INPUT

while True:
    a = lgpio.gpio_read(HANDLE, INPUT_PINS[0])
    b = lgpio.gpio_read(HANDLE, INPUT_PINS[1])
    c = lgpio.gpio_read(HANDLE, INPUT_PINS[2])
    d = lgpio.gpio_read(HANDLE, INPUT_PINS[3])

    print(a, b, c, d)