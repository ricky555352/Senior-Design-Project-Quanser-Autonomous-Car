
#https://medium.com/swlh/roi-segmentation-contour-detection-and-image-thresholding-using-opencv-c0d2ea47b787

import cv2
import numpy as np
import copy
import math

x=0.5  # width ratio of the ROI
y=0.5  # height ratio of the ROI
roi_width_ratio = 0.5  # ratio of frame width for the ROI
roi_height_ratio = 0.5  # ratio of frame height for the ROI
threshold = 60  # BINARY threshold
blurValue = 7  # GaussianBlur parameter
bgSubThreshold = 50
learningRate = 0

# variables
isBgCaptured = 0   # whether the background captured

def removeBG(frame):  # Subtracting the background
    fgmask = bgModel.apply(frame, learningRate=learningRate)

    kernel = np.ones((3, 3), np.uint8)
    fgmask = cv2.erode(fgmask, kernel, iterations=1)
    res = cv2.bitwise_and(frame, frame, mask=fgmask)
    return res

# Camera
camera = cv2.VideoCapture(0)
camera.set(0, 200)

while camera.isOpened():
    ret, frame = camera.read()
    if not ret:
        break

    frame = cv2.bilateralFilter(frame, 5, 50, 100)  # smoothening filter
    frame = cv2.flip(frame, 1)  # flip the frame horizontally

    # Get frame dimensions
    frame_height, frame_width = frame.shape[:2]

    # Calculate the ROI centered in the frame
    roi_width = int(roi_width_ratio * frame_width)
    roi_height = int(roi_height_ratio * frame_height)
    roi_x = (frame_width - roi_width) // 2  # X start point (centered)
    roi_y = (frame_height - roi_height) // 2  # Y start point (centered)

    # Draw the centered ROI (blue rectangle)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 0, 0), 2)  # Blue ROI

    cv2.imshow('original', frame)

    #  Main operation
    if isBgCaptured == 1:  # This part won't run until the background is captured
        img = removeBG(frame)
        # Clip the ROI
        img = img[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

        # Convert the image to a binary image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (blurValue, blurValue), 0)

        ret, thresh = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY)  # Thresholding the frame

        # Get the contours
        thresh1 = copy.deepcopy(thresh)
        contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Detecting contours
        length = len(contours)
        maxArea = -1
        if length > 0:
            for i in range(length):  # Find the biggest contour (according to area)
                temp = contours[i]
                area = cv2.contourArea(temp)
                if area > maxArea:
                    maxArea = area
                    ci = i

            res = contours[ci]
            hull = cv2.convexHull(res)  # Applying convex hull technique
            drawing = np.zeros(img.shape, np.uint8)
            cv2.drawContours(drawing, [res], 0, (0, 255, 0), 2)  # Drawing contours
            cv2.drawContours(drawing, [hull], 0, (0, 0, 255), 3)  # Drawing convex hull
    
        cv2.imshow('output', drawing)

    # Keyboard OP
    k = cv2.waitKey(10)
    if k == 27:  # Press ESC to exit
        camera.release()
        cv2.destroyAllWindows()
        break
    elif k == ord('b'):  # Press 'b' to capture the background
        bgModel = cv2.createBackgroundSubtractorMOG2(0, bgSubThreshold)
        isBgCaptured = 1
        print('Background Captured')
    elif k == ord('r'):  # Press 'r' to reset the background
        bgModel = None
        isBgCaptured = 0
        print('Reset Background')
