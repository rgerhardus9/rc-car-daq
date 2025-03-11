#! /usr/bin/env python

import lgpio
import time
import numpy as np
import cv2

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 15)  # Limit FPS to 15
cameraCenter = cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2
steeringFactor = 5  # Defines the min/max duty cycle range or "steering aggresssivness"
neutralDuty = 15  # Duty cycle in which the car goes straight
p = float(10**2)  # Floating point to handle rounding using integer math instead of the slower round(num, 2)

steeringPin = 12  # GPIO pin to output PWM for steering control 
throttlePin = 13  # GPIO pin to output PWM for throttle control 
frequency = 100  # Frequency in Hz


# Always do this
HANDLE = lgpio.gpiochip_open(0)
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
def get_duty_cycle(mask, prev_dc):  # Set a minimum area threshold
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Filter out small contours
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:  # Ignore small contours
            x, y, w, h = cv2.boundingRect(largest_contour)

            center_x = x + w // 2  # Only the x position determines the duty cycle calculation
            
            steerAmount = ((center_x - cameraCenter) / cameraCenter) * steeringFactor  # Ratio of steering relative to frame size (-1 to 1)

            # Rounding is faster through integer manipulation
            if steerAmount > 0:
                steerAmountRounded = int(steerAmount * p + 0.5) / p
            else:
                steerAmountRounded = int(steerAmount * p - 0.5) / p

            print(f"steerAmountRounded: {steerAmountRounded}")


            # Default is -5 to 5 + 15 = (10 to 20)%
            duty_cycle = steerAmountRounded + neutralDuty

            # Only update dc if it is different from before
            if (abs(duty_cycle - prev_dc) > 0.2):
                lgpio.tx_pwm(HANDLE, steeringPin, frequency, duty_cycle) 
                lgpio.tx_pwm(HANDLE, throttlePin, frequency, duty_cycle) # Duplicate for reading PWM
                return duty_cycle
            else:
                return prev_dc

    return -1  # No line detected, kill program

def main():

    # Claim pins 12 & 13 as output
    lgpio.gpio_claim_output(HANDLE, steeringPin)
    lgpio.gpio_claim_output(HANDLE, throttlePin)

    # Set Throttle and Steering to Neutral State (duty cycle of 15%)
    lgpio.tx_pwm(HANDLE, steeringPin, frequency, 15)
    lgpio.tx_pwm(HANDLE, throttlePin, frequency, 15)

    try:
        print(f"Generating PWM on GPIO {steeringPin} (steering) & {throttlePin} (throttle) at {frequency}Hz")
        print("Press Ctrl+C to stop.")
        time.sleep(2)

        steering_duty_cycle = 15.0

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to open camera")
                break
            
            mask = get_mask(frame)
            steering_duty_cycle = get_duty_cycle(mask, steering_duty_cycle)

            if steering_duty_cycle < 0:
                print("Failed to get duty cycle")
            
        
            print(steering_duty_cycle)
            # Commented out because we haven't determined how we are gonna generate the throttle duty cycle yet
            #lgpio.tx_pwm(HANDLE, throttlePin, frequency, throttle_duty_cycle)


        
    except KeyboardInterrupt:
        print("\nStopping PWM and cleaning up GPIO.")
    finally:
        # Stop PWM
        lgpio.tx_pwm(HANDLE, steeringPin, 0, 0)
        lgpio.tx_pwm(HANDLE, throttlePin, 0, 0)

        # Close GPIO chip
        lgpio.gpiochip_close(HANDLE)





if __name__ == "__main__":
    main()
