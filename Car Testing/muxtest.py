#! /usr/bin/env python

import lgpio
import time

# GPIO pin definitions
SELECT_PIN = 25 # LOW = Y0 (transmitter), HIGH = Y1 (generated PWM)
PWM_GENERATE_PIN = 13

OP_HIGH = 5
OP_LOW = 16

GPIO_CHIP = 0
PERIOD = 0.01   # 10 ms
FREQUENCY = 100 # Hz

# GPIO initialization
HANDLE = lgpio.gpiochip_open(GPIO_CHIP)   # Always do this
# SELECT
lgpio.gpio_claim_output(HANDLE, SELECT_PIN)  # Claim pin as OUTPUT
# PWM Generation
lgpio.gpio_claim_output(HANDLE, PWM_GENERATE_PIN)
# Test outputs
lgpio.gpio_claim_output(HANDLE, OP_HIGH)  # Claim pin as OUTPUT
lgpio.gpio_claim_output(HANDLE, OP_LOW)  # Claim pin as OUTPUT

lgpio.gpio_write(HANDLE, OP_HIGH, 1)     # Set to HIGH
lgpio.gpio_write(HANDLE, OP_LOW, 0)     # Set to LOW

start_time = time.time()
select = 0

while (time.time() - start_time < 60.0):
    if (select == 0):
        print(f"select = {select}")
        lgpio.gpio_write(HANDLE, SELECT_PIN, 1)
        select = 1
    else:
        print(f"select = {select}")
        lgpio.gpio_write(HANDLE, SELECT_PIN, 0)
        select = 0
    time.sleep(10.0)


    







