# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#region : File Description and Imports

import signal
import numpy as np
import time
import cv2

from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR


#Used to enable killswitch 
global KILL_THREAD
KILL_THREAD = False

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True

signal.signal(signal.SIGINT, sig_handler)

def roi(image, vertices):
    mask = np.zeros_like(image)
    mask_color = 255
    cv2.fillPoly(mask, vertices, mask_color)
    cropped_img = cv2.bitwise_and(image, mask)
    return cropped_img

def draw_lines(image, hough_lines):
    if hough_lines is not None:
        for line in hough_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 5) #5 was orignally a 2
    return image

def process(img):
    #cv2.imshow("process test", img)
    #cv2.waitKey(1000)
    height = img.shape[0]
    width = img.shape[1]
    #image pixel index starts at (0,0) from top left and goes to bottom right
    roi_vertices = [ #this outline makes a trapezoid
        (0, height), #bottom left
        (int(1*width/6), int(height/2)), #top left
        (int(2*width/3), int(height/2)), #top right
        (width, height) #bottom right
    ]
       # these are the lines that connect the points (above)
    '''
    img = cv2.line(img, roi_vertices[0], roi_vertices[1], (0,255,0), 9)
    img = cv2.line(img, roi_vertices[1], roi_vertices[2], (0,255,0), 9)
    img = cv2.line(img, roi_vertices[2], roi_vertices[3], (0,255,0), 9)
    img = cv2.line(img, roi_vertices[3], roi_vertices[0], (0,255,0), 9)
    cv2.imshow("Lines", img)
    cv2.waitKey(1000)
    '''
    converted = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)    #RGB to HLS conversion for color mask
    #white color mask
    lower_white = np.uint8([  0, 0, 200]) #200 was originally 80
    upper_white = np.uint8([180, 25, 255])#180,25,255 was originally 0,0,100
    white_mask = cv2.inRange(converted, lower_white, upper_white)
    
    #yellow color mask
    lower_yellow = np.uint8([0, 0, 8])#18,94,140 was originally 35,20,50. 0,0,8
    upper_yellow = np.uint8([160, 202, 234])#48,255,255 was orginally 42,100,180. 160,202,234
    yellow_mask = cv2.inRange(converted, lower_yellow, upper_yellow)
    
    
    #Combine the masks
    #img = cv2.bitwise_and(img, img, mask = yellow_mask)
    #img = cv2.bitwise_and(img, img, mask = white_mask)
    
    combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
    masked_img = cv2.bitwise_and(img, img, mask = combined_mask)
    #above masked_img replaced with img
    
    #cv2.imshow("Masked Image", img)
    #cv2.waitKey(1000)
    #(testing the masked image code)cv2.imshow("Masked Image", img)
    #cv2.waitKey(1000)
    #img = cv2.convertScaleAbs(img, 1.25, 0) #this wasnt needed (converts into 8 bit)
    #(testing the Converted Scale image code)cv2.imshow("Convert Scale Image", img)
    #cv2.waitKey(1000)
    #gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.dilate(gray_img, kernel=np.ones((3, 3), np.uint8))
    #(testing the gray image code)cv2.imshow("Gray", gray_img)
    #cv2.waitKey(1000)
    canny = cv2.Canny(gray_img, 50, 150)#50,150 was originally 130,220
    roi_img = roi(canny, np.array([roi_vertices], np.int32))

    #cv2.imshow("img", img)
    #cv2.waitKey(1000)
    
    lines = cv2.HoughLinesP(roi_img, 1, np.pi / 180, threshold=10, minLineLength=15, maxLineGap=2)
    final_img = draw_lines(img, lines)
    #return final_img

# Lane keeping logic
    if lines is not None:
        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)  # Calculate the slope of the line
            if slope < 0:  # Negative slope -> left lane line
                left_lines.append(line)
            else:  # Positive slope -> right lane line
                right_lines.append(line)

        left_x_avg = np.mean([line[0][0] for line in left_lines]) if left_lines else None
        right_x_avg = np.mean([line[0][0] for line in right_lines]) if right_lines else None

        if left_x_avg is not None and right_x_avg is not None:
            lane_center = (left_x_avg + right_x_avg) / 2
            frame_center = width / 2

            if lane_center < frame_center - 10:  # Car is to the right of the lane center
                steer_left()
            elif lane_center > frame_center + 10:  # Car is to the left of the lane center
                steer_right()
            else:
                go_straight()
        else:
            # If we lose track of one line, it's better to steer straight or stop
            go_straight()
    else:
        # If no lines are detected, we should probably stop
        stop_car()

    return final_img

def steer_left():
    print("Steering left...")
    # Replace with actual command to steer the QCar left

def steer_right():
    print("Steering right...")
    # Replace with actual command to steer the QCar right

def go_straight():
    print("Going straight...")
    # Replace with actual command to move the QCar forward

def stop_car():
    print("Stopping car...")
    # Replace with actual command to stop the QCar

def cameraFeed():
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
    #while TRUE: (USING not KILL_THREAD instead of this)
    while not KILL_THREAD:
        start_time = time.time()
        #time.sleep(1)
        # Capture RGB Image from CSI
        cameras.readAll()
        #time.sleep(1) #wait for 1 seconds
        
        #print(type(cameras.csiFront.imageData))
        #print(cameras.csiFront.imageData.shape)
        imagedata = cameras.csiFront.imageData
        #cv2.imshow("img", imagedata)
        #cv2.waitKey(5000)
        if imagedata is not None and imagedata.size > 0:
            frame = process(imagedata)
            cv2.imshow("Processed Frame", frame)
        # Stitch images together with black padding
        
        imageBuffer360 = np.concatenate((cameras.csiRight.imageData,
                                         cameras.csiBack.imageData,
                                         cameras.csiLeft.imageData,
                                         cameras.csiFront.imageData),
                                         axis = 1)
        
        # Display the stitched image
        imageWidth = 640
        imageHeight = 480
        '''
        if cameras.csiFront.imageData.shape != 0:
            print(imagedata.shape)
            frame = process(imagedata)
            cv2.imshow("Processed Frame", frame)
        '''
        
        cv2.imshow('Combined View', cv2.resize(imageBuffer360,
                                               (int(2*imageWidth),
                                                int(imageHeight/2))))
        cv2.imshow("frame", cameras.csiFront.imageData)
        
        # Wait for 1 millisecond
        #cv2.waitKey(1)
        elapsed_time = time.time() - start_time
        sleep_time = max(frame_time - elapsed_time, 0)
        
        if cv2.waitKey(int(sleep_time * 1000)) & 0xFF == ord('q'):
            break
        
        
    cameras.close()
    cv2.destroyAllWindows()
    #endregion
    
#region : Setup and run experiment
if __name__ == '__main__':
    cameraFeed()
    KILL_THREAD = True
    if not IS_PHYSICAL_QCAR:
        input('Experiment complete. Press any key to exit...')
#endregion