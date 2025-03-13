#! /usr/bin/env python

import lgpio
import time
import numpy as np
import cv2

# Initialize camera
cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0)
cameraCenter = cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2
steeringFactor = 5  # Defines the min/max duty cycle range or "steering aggresssivness"
p = float(10**1)  # Floating point to handle rounding using integer math instead of the slower round(num, 2)

steeringPin = 12  # GPIO pin to output PWM for steering control 
throttlePin = 13  # GPIO pin to output PWM for throttle control 
frequency = 100  # Frequency in Hz

# Duty cycle values for updating PWM and sending PWM less often
neutralDuty = 15  # Duty cycle in which the car goes straight
old_steering_duty_cycle = 0 # Values to compare current duty cycle to send PWM if there is a difference
old_throttle_duty_cycle = 0

# Apply HSV thresholding for neon pink detection
def get_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([140, 20, 210], np.uint8)  # Adjust for lighting conditions
    upper_pink = np.array([175, 255, 255], np.uint8)
    mask = cv2.inRange(hsv, lower_pink, upper_pink)
    
    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Close small holes
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove small noise

    return mask

# Return PWM based on horizontal distance of line to camera centerline
def get_duty_cycle(mask):  # Set a minimum area threshold
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Filter out small contours
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:  # Ignore small contours
            x, y, w, h = cv2.boundingRect(largest_contour)

            center_x = x + w // 2  # Only the x position determines the duty cycle calculation

            ratioToCenter = (center_x - cameraCenter) / cameraCenter # Ratio of steering relative to frame size (-1 to 1)

            steeringFactor = 5*abs(ratioToCenter)
            
            steerAmount =  ratioToCenter * steeringFactor  # Ratio of steering relative to frame size (-1 to 1)

            # Rounding is faster through integer manipulation
            if steerAmount > 0:
                steerAmountRounded = int(steerAmount * p + 0.5) / p
            else:
                steerAmountRounded = int(steerAmount * p - 0.5) / p

            # Default is -5 to 5 + 15 = (10 to 20)%
            duty_cycle = steerAmountRounded + neutralDuty



            return duty_cycle
    return -1  # No line detected, kill program

def main():
    gpioChipHandle = lgpio.gpiochip_open(0)  # Open GPIO chip

    # Claim pins 12 & 13 as output
    lgpio.gpio_claim_output(gpioChipHandle, steeringPin)
    lgpio.gpio_claim_output(gpioChipHandle, throttlePin)

    # Set Throttle and Steering to Neutral State (duty cycle of 15%)
    lgpio.tx_pwm(gpioChipHandle, steeringPin, frequency, 15)
    lgpio.tx_pwm(gpioChipHandle, throttlePin, frequency, 15)

    try:
        print(f"Generating PWM on GPIO {steeringPin} (steering) & {throttlePin} (throttle) at {frequency}Hz")
        print("Press Ctrl+C to stop.")
        time.sleep(2)
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to open camera")
                break
            
            mask = get_mask(frame)
            steering_duty_cycle = get_duty_cycle(mask)
            # throttle_duty_cycle = something

            if steering_duty_cycle < 0:
                print("Failed to get duty cycle")
                break
            # if throttle_duty_cycle < 0:
            #     break

            if steering_duty_cycle != old_steering_duty_cycle:
                print(steering_duty_cycle)
                lgpio.tx_pwm(gpioChipHandle, steeringPin, frequency, steering_duty_cycle)
            # Commented out because we haven't determined how we are gonna generate the throttle duty cycle yet
            # if throttle_duty_cycle != old_throttle_duty_cycle:
                # lgpio.tx_pwm(gpioChipHandle, throttlePin, frequency, throttle_duty_cycle)
            time.sleep(1/frequency)


        
    except KeyboardInterrupt:
        print("\nStopping PWM and cleaning up GPIO.")
    finally:
        # Stop PWM
        lgpio.tx_pwm(gpioChipHandle, steeringPin, 0, 0)
        lgpio.tx_pwm(gpioChipHandle, throttlePin, 0, 0)

        # Close GPIO chip
        lgpio.gpiochip_close(gpioChipHandle)

if __name__ == "__main__":
    main()
