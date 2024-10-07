import cv2
import numpy as np 

#print("Starting")
# reading image 
img = cv2.imread('StopSignVirtual.png') 
#print("loaded image")
# converting image into grayscale image 
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
#print("made image grey")
# setting threshold of gray image 
_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
#print("set threshold")
# using a findContours() function 
contours, _ = cv2.findContours( 
    threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
#print("found contours")
i = 0

'''
#approximate the biggest shape and draw its contour
bigshape = max(contours, key=cv2.contourArea)

approx = cv2.approxPolyDP(bigshape, 0.01 * cv2.arcLength(bigshape, True), True)

cv2.drawContours(img, [bigshape], 0, (0, 0, 255), 5)
'''
thresholdArea = 1250

# list for storing names of shapes 
for contour in contours: 
    #print("next contour")
    # here we are ignoring first counter because  
    # findcontour function detects whole image as shape 
    if i == 0: 
        i = 1
        continue

    area = cv2.contourArea(contour)
    if area > thresholdArea:
        # cv2.approxPloyDP() function to approximate the shape 
        approx = cv2.approxPolyDP( 
            contour, 0.03 * cv2.arcLength(contour, True), True) 
        
        # using drawContours() function 
        cv2.drawContours(img, [contour], 0, (0, 0, 255), 5) 
    
        
        # finding center point of shape 
        M = cv2.moments(contour) 
        x = 0 #int(M['m10'])
        y = 0 #int(M['m01'])
        if M['m00'] != 0.0: 
            x = int(M['m10']/M['m00']) 
            y = int(M['m01']/M['m00'])

            # putting shape name at center of each shape 
        if len(approx) == 3: 
            cv2.putText(img, 'Triangle', (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
    
        elif len(approx) == 4: 
            cv2.putText(img, 'Quadrilateral', (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
    
        elif len(approx) == 5: 
            cv2.putText(img, 'Pentagon', (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
    
        elif len(approx) == 6: 
            cv2.putText(img, 'Hexagon', (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2) 
    
        elif len(approx) == 8: 
            cv2.putText(img, 'Octagon', (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else: 
            cv2.putText(img, 'Circle', (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        #''' 
  
# displaying the image after drawing contours 
cv2.imshow('shapes', img) 
  
cv2.waitKey(0) 
cv2.destroyAllWindows() 