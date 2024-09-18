#region : File Description and Imports

import signal
import time
import cv2
import numpy as np

from pal.products.qcar import QCarCameras, QCar, IS_PHYSICAL_QCAR

# Used to enable killswitch 
global KILL_THREAD
KILL_THREAD = False

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True

signal.signal(signal.SIGINT, sig_handler)

def detect_stop_sign(img):
    # Convert image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define range for red color in HSV
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # Threshold the HSV image to get only red colors
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Find contours in the red mask
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over contours
    for cnt in contours:
        # Approximate the contour
        approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
        
        # Check if the contour has 8 vertices (octagon shape) and is sufficiently large
        if len(approx) == 8 and cv2.contourArea(cnt) > 1000:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = w / float(h)
            if 0.8 <= aspect_ratio <= 1.2:  # Check if the bounding box is roughly square
                # Draw a bounding box around the detected stop sign
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                return True  # Stop sign detected

    return False  # No stop sign detected

def process_frame(img):
    if detect_stop_sign(img):
        print("Stop sign detected! Stopping...")
        stop_car()
        time.sleep(3)  # Stop for 3 seconds
        print("Resuming movement...")
        move_car_forward()
    else:
        print("No stop sign detected. Moving forward.")
        move_car_forward()

    return img

def stop_car():
    print("Stopping the QCar.")
    # Implement the actual stop command for the QCar
    QCar.stop()

def move_car_forward():
    print("Moving the QCar forward.")
    # Implement the actual move forward command for the QCar
    QCar.forward(0.2)  # Assuming forward speed is 0.2 m/s, adjust as necessary

def cameraFeed():
    global KILL_THREAD
    # Initialize the CSI cameras
    cameras = QCarCameras(
        enableBack=False,
        enableFront=True,
        enableLeft=False,
        enableRight=False,
    )
    
    fps = 30
    frame_time = 1.0 / fps
    while not KILL_THREAD:
        start_time = time.time()
        
        # Capture RGB Image from CSI
        cameras.readAll()
        imagedata = cameras.csiFront.imageData

        if imagedata is not None and imagedata.size > 0:
            processed_frame = process_frame(imagedata)
            cv2.imshow("Processed Frame", processed_frame)
        
        elapsed_time = time.time() - start_time
        sleep_time = max(frame_time - elapsed_time, 0)
        
        if cv2.waitKey(int(sleep_time * 1000)) & 0xFF == ord('q'):
            break
        
    cameras.close()
    cv2.destroyAllWindows()

#region : Setup and run experiment
if __name__ == '__main__':
    cameraFeed()
    KILL_THREAD = True
    if not IS_PHYSICAL_QCAR:
        input('Experiment complete. Press any key to exit...')
#endregion