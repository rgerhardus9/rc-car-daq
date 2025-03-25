#! /usr/bin/env python

# Uses threading to allow MUX control, throttle control (constant right now), and camera line following steering control

import lgpio
import time
import numpy as np
import cv2

import threading

# Initialize camera
cap = cv2.VideoCapture(0)
# Set camera parameters
'''
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 60)  # Reduce width to 320 pixels
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 40)  # Reduce height to 240 pixels
cap.set(cv2.CAP_PROP_FPS, 10)  # Reduce FPS to 15
'''

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
print(f"width: {width}")
# Default
if width == 0:
    width = 1280
cameraCenter = width / 2

steeringFactor = 4  # Defines the min/max duty cycle range or "steering aggresssivness"
neutralDuty = 15  # Duty cycle in which the car goes straight
p = float(10**2)  # Floating point to handle rounding using integer math instead of the slower round(num, 2)
steering_duty_cycle = 15.0

# Define pins and constants
STEERING_PIN = 12  # Pin 32
THROTTLE_PIN = 13  # Pin 33 
SELECT_PIN = 25    # Pin 22 ==> LOW = Y0 (transmitter), HIGH = Y1 (generated PWM)
INPUT_PIN = 6      # Pin 31 ==> Read receiver PWM to change MUX signal
FREQUENCY = 100  # FREQUENCY in Hz
GPIO_CHIP = 0

# Mutexes
steering_lock = threading.Lock()

# Always do this
HANDLE = lgpio.gpiochip_open(GPIO_CHIP)

# Apply HSV thresholding for neon pink detection - Camera thread
def get_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([140, 20, 210], np.uint8)  # Adjust for lighting conditions
    upper_pink = np.array([175, 255, 255], np.uint8)
    mask = cv2.inRange(hsv, lower_pink, upper_pink)

    # Debugging: Check if there are any non-zero pixels in the mask
    if np.sum(mask) == 0:
        print("Mask has no non-zero pixels.")
    
    # Apply morphological operations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Close small holes
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # Remove small noise

    return mask

# Return PWM based on horizontal distance of line to camera centerline
def get_duty_cycle(mask, prev_dc):
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Filter out small contours
        largest_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest_contour) > 500:  # Ignore small contours
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2  # Only the x position determines the duty cycle calculation         
            
            # Dynamic steeringFactor
            ratioToCenter = (center_x - cameraCenter) / cameraCenter    # Ratio of steering relative to frame size (-1 to 1)
            steeringFactor = 5 * abs(ratioToCenter)
            steerAmount = ratioToCenter * steeringFactor # should be -5 to 5 now

            # Rounding is faster through integer manipulation
            if steerAmount > 0:
                steerAmountRounded = int(steerAmount * p + 0.5) / p
            else:
                steerAmountRounded = int(steerAmount * p - 0.5) / p

            # print(f"steerAmountRounded: {steerAmountRounded}")


            # Default is -5 to 5 + 15 = (10 to 20)%
            duty_cycle = steerAmountRounded + neutralDuty
            return duty_cycle
            
            # Only update dc if it is different from before
            '''
            if (abs(duty_cycle - prev_dc) > 0.2):
                return duty_cycle
            else:
                return prev_dc
            '''

    else:
        print("No contours. get_duty_cycle returning -1.")
        # Make this return prev_dc if we want the program to not kill and keep trying to find contours
        return -1  # No line detected, kill program
    

# Read PWM duty cycle from receiver
# Returns select to send via SELECT_PIN ==> 0 to switch to manual control, 1 to keep program control
def read_pwm_duty_cycle(CHIP, INPUT_PIN, duration = 0.3):   # 0.3s to calculate DC and switch MUX select - delay, but need high enough for a good calculation

    print("Measuring PWM duty cycle...")
    # Run until DC is outside of range - HIT THE BRAKES
    init_time = time.time()   

    # Safety feature doing this instead of while(True) - give back control after giveBackControl seconds
    giveBackControl = 15.0
    while(time.time() - init_time < giveBackControl):
    # while (True):
        # Initialize times
        high_time = 0
        low_time = 0

        # Initialize level and time
        previous_level = lgpio.gpio_read(CHIP, INPUT_PIN)
        # print(f"Previous level: {previous_level}")

        pulse_start = time.time()   # Time in seconds since epoch
        try:
            while (time.time() - pulse_start) < duration:
                # print(f"Time: {(time.time() - start_time):.5f}")

                # This should work with the input changing - reading high or low of the PWM
                current_level = lgpio.gpio_read(CHIP, INPUT_PIN)       
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
            #print(f"DC read from receiver: {dc}")

            # Change MUX signal or continue - do this in or out of function?
            if (dc < 14.0 or dc > 16.0):
                print("Select being writtn to 0!")
                lgpio.gpio_write(CHIP, SELECT_PIN, 0)     # Select signal to LOW = PWM transmitter/receiver signal
                return 0
            else:
                continue


# Mutex for steering_duty_cycle because it is constantly changing via the camera module in main()
def steering_pwm_thread():
    global steering_duty_cycle
    # Always send a PWM - unless MUX has switched select (shouldn't be too important)?
    while True:
        # Lock so steering_duty_cycle doesn't change during execution
        with steering_lock:
            lgpio.tx_pwm(HANDLE, STEERING_PIN, FREQUENCY, steering_duty_cycle)
            # print(f"Thread running with steering duty cycle: {steering_duty_cycle}") # Gonna print a lot
        time.sleep(0.02)  # Small delay to keep CPU usage low - write duty cycle every 20 ms

def mux_thread():
    # Duty cycle to send for program control
    while (True):
        select = read_pwm_duty_cycle(HANDLE, INPUT_PIN) # Stuck here until function returns 0 to switch to manual control - while(True) above unnecessary
        if (select == 0):
            lgpio.gpio_write(HANDLE, SELECT_PIN, select)
            print(f"MUX has given back manual control. Ending thread.")
            return  # End thread - does this end the thread or just the function?

# TODO: Make control system thread


def main():

    global steering_duty_cycle
    # Initialize duty cycle here instead of up top?

    # Claim pins as inputs/outputs
    lgpio.gpio_claim_output(HANDLE, STEERING_PIN)
    lgpio.gpio_claim_output(HANDLE, THROTTLE_PIN)
    lgpio.gpio_claim_output(HANDLE, SELECT_PIN)
    lgpio.gpio_claim_input(HANDLE, INPUT_PIN, lgpio.SET_PULL_DOWN)

    # Initialize pin values that need it
    lgpio.gpio_write(HANDLE, SELECT_PIN, 1) # Init to HIGH - Program Generated PWMs
    throttle_dc = 15.0
    lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, throttle_dc)
    time.sleep(2.0)

    # Define and start threads from functions - daemon threads killed automatically when main() exits
    # Non-daemon threads don't allow main() to be run until they are killed

    # Steering PWM thread - Mutex for steering_duty_cycle because it is calculated in main() by the camera module
    steering_pwm_thread_instance = threading.Thread(target=steering_pwm_thread, daemon=True)
    steering_pwm_thread_instance.start()

    # MUX thread instance - Mutex for select and input pins not needed as they are only changed locally
    mux_thread_instance = threading.Thread(target=mux_thread, daemon=True)
    mux_thread_instance.start()

    # TODO: Implement control system thread to update throttle_dc - Mutex for throttle_dc




    # Update throttle_dc to desired speed - start driving
    throttle_dc = 15.4
    lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, throttle_dc)

    print(f"Generating PWM with duty cycle {throttle_dc} on GPIO {THROTTLE_PIN} (throttle) at {FREQUENCY}Hz")
    print("Press Ctrl+C to stop.")


    prev_duty_cycle = 15.0 

    # Camera steering module
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to open camera")
                break
            
            mask = get_mask(frame)
            new_duty_cycle = get_duty_cycle(mask, prev_duty_cycle)

            if new_duty_cycle < 0:
                # Kill everything - kills threads too
                print("Failed to get duty_cycle/contours - stopping car")
                # Maybe stop throttle here
                lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, 15.0)
                lgpio.tx_pwm(HANDLE, STEERING_PIN, FREQUENCY, 15.0)
                time.sleep(2.0)
                print("1 - Stopping any data transmit on STEERING_PIN and THROTTLE_PIN")
                lgpio.tx_pwm(HANDLE, STEERING_PIN, 0, 0)
                lgpio.tx_pwm(HANDLE, THROTTLE_PIN, 0, 0)

                # Close GPIO chip
                lgpio.gpiochip_close(HANDLE)
                return
            
            with steering_lock:
                steering_duty_cycle = new_duty_cycle    # Should update the global variable for the thread

            prev_duty_cycle = new_duty_cycle

            # Do we want a delay in here?

            #print(f"Steering duty cycle: {steering_duty_cycle}")


        
    except KeyboardInterrupt:
        print("\nStopping PWM and cleaning up GPIO.")
    finally:
        # Stop PWM
        # Maybe stop throttle here
        lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, 15.0)
        lgpio.tx_pwm(HANDLE, STEERING_PIN, FREQUENCY, 15.0)
        time.sleep(2)
        print("2 - Stopping any data transmit on STEERING_PIN and THROTTLE_PIN")
        lgpio.tx_pwm(HANDLE, STEERING_PIN, 0, 0)
        lgpio.tx_pwm(HANDLE, THROTTLE_PIN, 0, 0)

        # End threads and close GPIO chip
        lgpio.gpiochip_close(HANDLE)
        return
    '''
        mux_thread_instance.join()
        steering_pwm_thread_instance.join()
    '''



if __name__ == "__main__":
    main()
