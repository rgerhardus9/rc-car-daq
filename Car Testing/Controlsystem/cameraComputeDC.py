from cameraAndThrottle import get_mask, get_duty_cycle
import numpy as np
import cv2



def camera_get_dc(cap):

    ret, frame = cap.read()
    if not ret:
        print("Failed to open camera")
        

    # Define the region (change as needed) - top works best for sharp turns
    region = "middle"  # Options: "top (2/3)", "middle(1/3)", "bottom(2/3)"

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

    if new_duty_cycle < 0:
        print("Failed to get duty_cycle/contours")
        return -1
    
    return new_duty_cycle
    

