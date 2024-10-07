import cv2
import numpy as np 
from matplotlib import pyplot as plt
#Start of object Detections:
#print("Starting")
img = cv2.imread('C:/Users/andyj/OneDrive/Desktop/Quanser/road.png')
#print("loaded image")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # converting image into grayscale image 
#print("made image grey")
#img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  #converts img into RGB instead of BGR

#stop_data = cv2.CascadeClassifier('stop_data.xml')  
#found = stop_data.detectMultiScale(gray, minSize =(20, 20))
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)  
# Don't do anything if there's no object
amount_found = len(faces)
  
if amount_found != 0:
    for (x, y, width, height) in faces:
          
        # We draw a rectangle around the object
        cv2.rectangle(img, (x, y), 
                      (x + height, y + width), 
                      (255, 0, 0), 2)   #(0, 255, 0), 5) could be this too
          
# Creates the environment of 
# the picture and shows it
cv2.imshow('Detected Faces', img)
cv2.waitKey(0)
cv2.destroyAllWindows()