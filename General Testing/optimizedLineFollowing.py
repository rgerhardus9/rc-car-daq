import cv2
import numpy as np

# Initialize camera
cap = cv2.VideoCapture(1)
cameraCenter = cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2
steeringFactor = 5
neutralDuty = 15
p = float(10**2)

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
def get_duty_cycle(mask, min_area=500):  # Set a minimum area threshold
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Filter out small contours
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > min_area:  # Ignore small contours
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            steerAmount = ((center_x - cameraCenter) / cameraCenter) * steeringFactor
            if steerAmount > 0:
                steerAmountRounded = int(steerAmount * p + 0.5) / p
            else:
                steerAmountRounded = int(steerAmount * p - 0.5) / p
            duty_cycle = steerAmountRounded + neutralDuty
            return duty_cycle
    return -1  # No line detected, kill program

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to open camera")
        break
    
    mask = get_mask(frame)
    steering_duty_cycle = get_duty_cycle(mask)

    cv2.imshow("Camera", frame)

    # Code to upload pwm to pi goes here (Rauls code)

    if steering_duty_cycle < 0:
        print("Failed to read duty cycle")
        break

# Release resources
cap.release()
cv2.destroyAllWindows()