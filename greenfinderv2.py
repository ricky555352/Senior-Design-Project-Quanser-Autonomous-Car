#region : File Description and Imports

import signal
import numpy as np
import time
import cv2

from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR
from pal.products.qcar import QCar  # Assuming you have QCar control commands available

# Used to enable killswitch 
global KILL_THREAD
KILL_THREAD = False

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True

signal.signal(signal.SIGINT, sig_handler)

def process(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Lower and upper range of green stoplight color in HSV
    hsv_green_lower = (50, 160, 200)
    hsv_green_upper = (70, 255, 255)

    # Mask with the range of greens for stoplight color
    mask = cv2.inRange(hsv_img, hsv_green_lower, hsv_green_upper)

    # Only show the pixels that contain the stoplight color
    color_image = cv2.bitwise_and(img, img, mask=mask)

    # Check if the average of the green pixels is above a threshold
    if np.average(color_image) >= 0.0004:
        print("Light is green, go!")
        # Send command to make the car move forward
        move_car_forward()
    else:
        print("Light is red, stop!")
        # Send command to make the car stop
        stop_car()

def move_car_forward():
    # Placeholder for car movement logic
    # Replace with actual command to move the QCar forward
    print("Moving car forward...")

def stop_car():
    # Placeholder for car stop logic
    # Replace with actual command to stop the QCar
    print("Stopping car...")

def green_finder():
    global KILL_THREAD
    # Initialize the CSI cameras
    cameras = QCarCameras(
        enableBack=True,
        enableFront=True,
        enableLeft=True,
        enableRight=True,
    )
    
    fps = 30
    frame_time = 1.0 / fps
    while not KILL_THREAD:
        start_time = time.time()
                
        cameras.readAll()
        
        imagedata = cameras.csiFront.imageData
        if imagedata is not None and imagedata.size > 0:
            process(imagedata)
        
        # Stitch images together with black padding
        imageBuffer360 = np.concatenate((cameras.csiRight.imageData,
                                         cameras.csiBack.imageData,
                                         cameras.csiLeft.imageData,
                                         cameras.csiFront.imageData),
                                         axis = 1)
        
        # Display the stitched image
        imageWidth = 640
        imageHeight = 480
        
        cv2.imshow('Combined View', cv2.resize(imageBuffer360,
                                               (int(2*imageWidth),
                                                int(imageHeight/2))))
        cv2.imshow("frame", cameras.csiFront.imageData)
        
        elapsed_time = time.time() - start_time
        sleep_time = max(frame_time - elapsed_time, 0)

        if cv2.waitKey(int(sleep_time * 1000)) & 0xFF == ord('q'):
            break

        time.sleep(0.1)  # Small delay to prevent overloading the system with checks

    cameras.close()
    cv2.destroyAllWindows()

#region : Setup and run experiment
if __name__ == '__main__':
    green_finder()
    KILL_THREAD = True
    if not IS_PHYSICAL_QCAR:
        input('Experiment complete. Press any key to exit...')
#endregion