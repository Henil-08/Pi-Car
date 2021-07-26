import numpy as np
import cv2 as cv

def nothing(x):
    pass

_INITIAL_WIDTH = 320
_INITIAL_HEIGHT = 240

cap = cv.VideoCapture(0)
cap.set(3, _INITIAL_WIDTH)
cap.set(4, _INITIAL_HEIGHT)

# cv.namedWindow('Tracking')

# cv.createTrackbar('LH', 'Tracking', 0, 255, nothing)
# cv.createTrackbar('LS', 'Tracking', 0, 255, nothing)
# cv.createTrackbar('LV', 'Tracking', 0, 255, nothing)

# cv.createTrackbar('UH', 'Tracking', 0, 255, nothing)
# cv.createTrackbar('US', 'Tracking', 0, 255, nothing)
# cv.createTrackbar('UV', 'Tracking', 0, 255, nothing)

while True:
    isSuccess, frame = cap.read()
    frame = cv.resize(frame, (340, 240))
    
    if isSuccess:
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # l_h = cv.getTrackbarPos('LH', 'Tracking')
        # l_s = cv.getTrackbarPos('LS', 'Tracking')
        # l_v = cv.getTrackbarPos('LV', 'Tracking') 

        # u_h = cv.getTrackbarPos('UH', 'Tracking')
        # u_s = cv.getTrackbarPos('US', 'Tracking')
        # u_v = cv.getTrackbarPos('UV', 'Tracking')

        l_b = np.array([30, 20, 20])
        u_b = np.array([85, 255, 255])

        mask = cv.inRange(hsv, l_b, u_b)
        res = cv.bitwise_and(frame, frame, mask=mask)

        cv.imshow("Frame", frame)
        cv.imshow("Mask", hsv)
        cv.imshow("Res", res)

        key = cv.waitKey(1) & 0xFF

        if key == 27:
            break
    else:
        print("No Frames Read!")
        break

cap.release()
cv.destroyAllWindows()