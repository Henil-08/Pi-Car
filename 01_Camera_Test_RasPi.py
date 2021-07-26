import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)

try:
    while True:
        _, frame = cap.read()

        cv.imshow('OrgVid', frame) 

        key = 27
        if(cv.waitKey(1) & 0xFF == key):
            break
    
finally:
    cap.release()
    cv.destroyAllWindows()