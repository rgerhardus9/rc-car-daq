#! /usr/bin/env python

# Uses threading to allow MUX control, throttle control (constant right now), and camera line following steering control

import lgpio
import time
import numpy as np
import cv2

import threading

# Initialize camera
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Set mode to MJPG (faster) - needs to be done before the pixel blocks below
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))


# Set camera parameters - Must all be in a block together - Camera has a FIXED ASPECT RATIO
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800) # MAIN LOOP TIMES with MJPG (~). 1920p=60ms, 1280p=35ms, 800p=15ms, 640p=13ms
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
cap.set(cv2.CAP_PROP_FPS, 90) 


# Get width in pixels - Should be what we set above
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"width: {width}, height: {height}")
# Default
if width == 0:
    width = 1920

# Manually setting this to the USB default
cameraCenter = width / 2

# 1 and 2 is working best - 4/20
sensitivity = 5  # Defines the min/max duty cycle range or "steering aggresssivness"
mode = 1

neutralDuty = 15  # Duty cycle in which the car goes straight
p = float(10**2)  # Floating point to handle rounding using integer math instead of the slower round(num, 2)
# Initialize throttle and steering PWM values
global steering_duty_cycle
steering_duty_cycle = 15.0
global throttle_dc
throttle_dc = 15.0

# Define pins and constants
STEERING_PIN = 12  # Pin 32
THROTTLE_PIN = 13  # Pin 33 
SELECT_PIN = 25    # Pin 22 ==> LOW = Y0 (transmitter), HIGH = Y1 (generated PWM)
INPUT_PIN = 6      # Pin 31 ==> Read receiver PWM to change MUX signal
FREQUENCY = 100  # FREQUENCY in Hz
GPIO_CHIP = 0
# Throttle control
SIMULATION_TIME = 60.0   # s
STARTING_SPEED = 3.0    # m/s
TARGET_SPEED = 20.1     # m/s - NEVER OVER 20.1
MAX_SPEED = 20.1        # m/s - DO NOT CHANGE
MAX_ACCELERATION = 0.054    # 5.4 m/s^2 = 0.054 m/ (10 ms)^2
ACCELERATION_STEP_10MS = (MAX_ACCELERATION) / (MAX_SPEED / 5)
STARTING_SPEED_DC = (5/MAX_SPEED) * STARTING_SPEED + 15.0   # Percent
TARGET_SPEED_DC = (5/MAX_SPEED) * TARGET_SPEED + 15.0       # Percent


# Ramp acceleration (want slower at beginning)
accRamp = np.linspace(0.017, 0.01, int(SIMULATION_TIME // 0.008))
accRamp = accRamp.tolist()

# Data Collection
COLLECT = False
steerTimeArr = []
steerDCArr = []
steerDistanceToCenterArr = []
throttleTimeArr = []
throttleDCArr = []


# Mutexes
steering_lock = threading.Lock()

# Always do this
HANDLE = lgpio.gpiochip_open(GPIO_CHIP)

# Claim pins as inputs/outputs
lgpio.gpio_claim_output(HANDLE, STEERING_PIN)
lgpio.gpio_claim_output(HANDLE, THROTTLE_PIN)
lgpio.gpio_claim_output(HANDLE, SELECT_PIN)
lgpio.gpio_claim_input(HANDLE, INPUT_PIN, lgpio.SET_PULL_DOWN)

# Initialize pin values that need it
lgpio.gpio_write(HANDLE, SELECT_PIN, 1) # Init to HIGH - Program Generated PWMs
lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, 15.0)

global stop_event
stop_event = threading.Event()

# Apply HSV thresholding for neon pink detection - Camera thread
def get_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([140, 30, 170], np.uint8)  # Adjust for lighting conditions
    upper_pink = np.array([180, 255, 255], np.uint8)
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
def get_duty_cycle(mask):
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Filter out small contours
        largest_contour = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest_contour) > 500:  # Ignore small contours
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2  # Only the x position determines the duty cycle calculation         
            # print(f"center_x: {center_x}, should be ~160")

            # Dynamic steeringFactor
            steerDistanceToCenterArr.append(center_x - cameraCenter)    # Pixels left (-) or right (+) of camera center
            ratioToCenter = (center_x - cameraCenter) / cameraCenter    # Ratio of steering relative to frame size (-1 to 1)

                                                                # Sets the power that scales duty cycle 
            steeringFactor = 5 * abs(ratioToCenter) ** (mode-1)         # Sets how aggressive vehicle steers based on how far the line is from the center
            steerAmount = ratioToCenter * steeringFactor                # Sets max range -5 to 5 following a quadratic esk formula

            # Rounding is faster through integer manipulation
            if steerAmount > 0:
                steerAmountRounded = int(steerAmount * p + 0.5) / p
            else:
                steerAmountRounded = int(steerAmount * p - 0.5) / p



            # Default is -5 to 5 + 15 = (10 to 20)%
            # From neutral, approach 10% or 20% at a rate equal to the power defined in mode -> (DC = (5*x.*abs(x))+15) for quadratic
            duty_cycle = steerAmountRounded + neutralDuty
            
            # print(f"MAIN update steering dc: {duty_cycle}")
            return duty_cycle

    else:
        print("No contours. get_duty_cycle returning -1.")
        # Make this return prev_dc if we want the program to not kill and keep trying to find contours
        return -1  # No line detected, kill program
    

# Read PWM duty cycle from receiver
# Returns select to send via SELECT_PIN ==> 0 to switch to manual control, 1 to keep program control

def read_pwm_duty_cycle(CHIP, INPUT_PIN, duration = 0.3):   # 0.3s to calculate DC and switch MUX select - delay, but need high enough for a good calculation

    # Run until DC is outside of range - HIT THE BRAKES
    global stop_event

    # Safety feature doing this instead of while(True) - give back control after giveBackControl seconds
    # Initialize times
    high_time = 0
    low_time = 0

    # print(f"Previous level: {previous_level}")

    block_start = time.time()
    previous_level = lgpio.gpio_read(HANDLE, INPUT_PIN)
    pulse_start = time.time()   # Time in seconds since epoch
    try:
        while (time.time() - block_start) < duration:
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
            
            time.sleep(0.0005)  # For even increments

    except:
        print("Error reading MUX. Returning -1.")
        return -1
        
    finally:
        total_time = high_time + low_time
        
        # No division by zero
        if total_time == 0:
            return 0
        
        # Duty cycle in percent
        dc = (high_time / total_time) * 100
        #print(f"DC read from receiver: {dc}")

        # Change MUX signal or continue - changed to 13.0 because of the thread not having the CPU power?
        if (dc < 13.0):
            print("Select being written to 0!")
            # lgpio.gpio_write(CHIP, SELECT_PIN, 0)     # Select signal to LOW = Manual Control - commented out because redundant
            return 0
        else:
            return 1    # Keep operating normally - MUX thread will call function again




# Mutex for steering_duty_cycle because it is constantly changing via the camera module in main()
def steering_pwm_thread():
    global steering_duty_cycle
    global stop_event
    print("THREAD: steering PWM running.")
    # Always send a PWM - unless MUX has switched select (shouldn't be too important)?
    pwm_steer_start = time.time()
    while not stop_event.is_set():
        # Lock so steering_duty_cycle doesn't change during execution
        with steering_lock:
            lgpio.tx_pwm(HANDLE, STEERING_PIN, FREQUENCY, steering_duty_cycle)
            steerTime = round(((time.time() - pwm_steer_start)), 3)
            steerDCArr.append(steering_duty_cycle)
            print(f"STEERING - Writing PWM: {round(steering_duty_cycle, 2)} at {steerTime * 1000} ms.") 


        steerTimeArr.append(steerTime)
        # STEERING PWM UPDATE RATE - don't go faster than camera 
        time.sleep(0.010)  # Write duty cycle every 10 ms (100 Hz)
    # Straighten out so we brake with straight wheels
    lgpio.tx_pwm(HANDLE, STEERING_PIN, FREQUENCY, 15.0)
    time.sleep(2.0)
    print("THREAD: Ending Steering")


def mux_thread():
    global stop_event
    print("THREAD: MUX")
    # Duty cycle to send for program control
    while not stop_event.is_set():
        # Calls this about every 300ms
        select = read_pwm_duty_cycle(HANDLE, INPUT_PIN) # Stuck here until function returns 0 to switch to manual control - while(True) above unnecessary
        if (select == 0):
            lgpio.gpio_write(HANDLE, SELECT_PIN, select)
            print(f"MANUAL CONTROL!")
            stop_event.set()
            return  # End thread - does this end the thread or just the function?
        elif (select == -1):
            print("5 - stop event.")
            stop_event.set()
        else:
            continue
    
    # Allow time for other threads to brake and straighten, then give back control
    time.sleep(3.0)
    lgpio.gpio_write(HANDLE, SELECT_PIN, 0)

            


# Set SIMULATION_TIME, MAX_ACCELERATION, STARTING_SPEED, and TARGET_SPEED
def throttle_thread():
    global stop_event
    global throttle_dc
    bump = True
    ctr = 0
    throttle_start_time = time.time()
    print("THREAD: Throttle Running.")
    throttle_dc = STARTING_SPEED_DC
    print(f"Starting throttle {throttle_dc}")
    while not stop_event.is_set() and (time.time() - throttle_start_time < SIMULATION_TIME):
        throtTime = round(time.time() - throttle_start_time, 2)
        if (bump):
            # print(f"THROTTLE - Writing PWM: {round(throttle_dc, 3)} at {throtTime} s.")
            lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, throttle_dc)
            time.sleep(0.200)   # Aim to update at 100 Hz (0.01s)
            bump = False
        else:
            lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, 15.0)
            time.sleep(0.400)
            bump = True


    print("3 - stop event")

    stop_event.set()

    if (throttle_dc > 15.5):
        throttle_dc = 15.0
        print("Time over. Braking.")
        lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, 11.0)
        time.sleep(2.0)
    lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, 15.0)
    time.sleep(0.5)

    return





def main():

    global stop_event
    stop_event.clear()
    global steering_duty_cycle

    # Define and start threads from functions - daemon threads killed automatically when main() exits

    # Steering PWM thread - Mutex for steering_duty_cycle because it is calculated in main() by the camera module
    steering_pwm_thread_instance = threading.Thread(target=steering_pwm_thread, daemon=False)
    steering_pwm_thread_instance.start()

    # MUX thread instance - Mutex for select and input pins not needed as they are only changed locally
    mux_thread_instance = threading.Thread(target=mux_thread, daemon=False)
    mux_thread_instance.start()

    # Throttle thread instance
    throttle_thread_instance = threading.Thread(target=throttle_thread, daemon=False)
    throttle_thread_instance.start()


    # Camera steering module
    try:
        while not stop_event.is_set():

            # t0 = time.time()

            ret, frame = cap.read()
            if not ret:
                print("Failed to open camera")
                break
            

            # Define the region (change as needed) - top works best for sharp turns
            region = "bottom"  # Options: "top (2/3)", "middle(1/3)", "bottom(2/3)"

            # Define region boundaries
            if region == "top":
                fy, fh = 0, (frame.shape[0] * 2) // 3
            elif region == "middle":
                fy, fh = frame.shape[0] // 3, frame.shape[0] // 3
            elif region == "bottom":
                fy, fh = frame.shape[0] // 3, (frame.shape[0] * 2) // 3

            fx, fw = 0, frame.shape[1]  # Keep full width

            # Create mask
            frameMask = np.zeros(frame.shape[:2], dtype="uint8")
            frameMask[fy:fy+fh, fx:fx+fw] = 255
            frame_new = cv2.bitwise_and(frame, frame, mask=frameMask)

            mask = get_mask(frame_new)


            new_duty_cycle = get_duty_cycle(mask)

            # Time end

            if new_duty_cycle < 0:
                # Kill everything - kills threads too
                print("Failed to get duty_cycle/contours - stopping car")

                # Maybe stop throttle here
                print("1 - stop event")
                stop_event.set()
                # Allow threads to brake the car and MUX
                time.sleep(3.0)

                break
            
            with steering_lock:
                steering_duty_cycle = new_duty_cycle    # Should update the global variable for the thread

            # print(f"Main loop took {time.time() - t0} seconds\n")

        
    except KeyboardInterrupt:
        print("\EXCEPT - Stopping PWM and cleaning up GPIO.")
    finally:
        # Stop PWM
        # Maybe stop throttle here
        print("2 - stop event")
        stop_event.set()
        time.sleep(2.0)    # Make sure threads don't write values

        # End threads
        # mux_thread_instance.join()
        steering_pwm_thread_instance.join()
        throttle_thread_instance.join()
        mux_thread_instance.join()

        # BRAKE
        if (throttle_dc > 15.0):
            lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, 11.5)
            time.sleep(1.0)
        # NEUTRAL
        lgpio.tx_pwm(HANDLE, THROTTLE_PIN, FREQUENCY, 15.0)
        lgpio.tx_pwm(HANDLE, STEERING_PIN, FREQUENCY, 15.0)
        time.sleep(0.5)

        print("FINALLY - Stopping any data transmit on STEERING_PIN and THROTTLE_PIN")
        # Stop sending anything
        lgpio.tx_pwm(HANDLE, STEERING_PIN, 0, 0)
        lgpio.tx_pwm(HANDLE, THROTTLE_PIN, 0, 0)

        # Give back control no matter what. Not bad if this is redundant.
        lgpio.gpio_write(HANDLE, SELECT_PIN, 0)


        # End threads and close GPIO chip
        
        lgpio.gpiochip_close(HANDLE)

        # Release camera
        cap.release()

        print(f"Average acceleration: {np.average(accRamp)}")

        # Copy these into arrays on local machine to visualize data 
        if COLLECT:
            print(f"Steer DC:\n\t{steerDCArr}\n\n")
            print(f"Throttle DC:\n\t{throttleDCArr}\n\n")
            print(f"Distance to Center:\n\t{steerDistanceToCenterArr}")

        
        
        return


if __name__ == "__main__":
    main()
