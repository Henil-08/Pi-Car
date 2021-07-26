import cv2 as cv
import numpy as np
import logging as log
from math import tan, atan, sqrt
from imutils.video import FPS
from time import time
from RPiCar import RPiCar 

_SHOW_IMAGE = False

PiCar = RPiCar()
PiCar.setup()
        
class LaneFollower():

    def __init__(self, car = None):
        log.info('Creating a LaneFollower...')
        self.current_steering_angle = 90
        self.car = car
        

    def follow_lane(self, frame):
        show_image("OrigiVid", frame)

        lane_lines, frame = detect_lines(frame)
        final_frame = self.steer(frame, lane_lines)

        return final_frame


    def steer(self, frame, lane_lines):
        log.info('steering...')
        if len(lane_lines) == 0:
            log.error('No lane lines detected, Trying to find Lane Lines.')
            return frame
        
        num_of_lane_lines = len(lane_lines)

        new_steering_angle = compute_steering_angle(frame, lane_lines)
        self.current_steering_angle = stabilizing_steering_angle(self.current_steering_angle, new_steering_angle, num_of_lane_lines)

        if self.car is True:
            PiCar.servo(self.current_steering_angle)

        current_heading_frame = display_heading_line(frame, self.current_steering_angle)
        show_image("Heading", current_heading_frame)

        return current_heading_frame


    def check_for_lanes(self, car, frame):
        '''
        This Function checks for few final lane lines at slow speed and for 5 seconds 
        If Lane Lines are detected at this step then the code continues
        If No Lane Lines are detected at this point then the code stops
        '''
        car.motor(20)
        car.servo(self.current_steering_angle)

        end_time = time() + 5

        while True:
            if time() > end_time:
                break

            found_lane_lines, found_frame =  detect_lines(frame)

        return found_lane_lines, found_frame

        

        
############################
# Frame Processing Steps
############################

def detect_lines(frame):
    log.debug('Detecting Lane Lines...')

    edge = edgeDetect(frame)
    show_image("Edge", edge)

    cropped_edge = region_of_interest(edge)
    show_image("ROI", cropped_edge, show=True)

    line_segments = detect_line_segments(cropped_edge)
    line_image = display_lines(frame, line_segments, linewidth=5)
    show_image("Line Image", line_image, show=True)

    lane_lines = average_slope_intercept(frame, line_segments)
    final_image = display_lines(frame, lane_lines)
    show_image("Final Image", final_image)      

    return lane_lines, final_image


def edgeDetect(frame):
    # Filtering White from the lane Lines
    # gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # mask = cv.GaussianBlur(gray_frame, (5, 5), 0)

    # Just for Example
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    show_image("hsv", hsv, show=True)
    lower_blue = np.array([40, 5, 0])
    upper_blue = np.array([75, 255, 255])
    mask = cv.inRange(hsv, lower_blue, upper_blue)
    show_image("Mask", mask, show=True)
    
    # Edge Detection
    edge_frame = cv.Canny(mask, 50, 150)
    
    return edge_frame


def region_of_interest(canny):
    height, width = canny.shape
    mask = np.zeros_like(canny)

    # Setting Coordinates of Bottom Half of the screen as Polygon
    polygon = np.array([[
        (0, height * 1/2), 
        (width, height * 1/2), 
        (width, height), 
        (0, height)
    ]], np.int32)
    
    cv.fillPoly(mask, polygon, 255)
    show_image("Mask", mask)
    masked_image = cv.bitwise_and(mask, canny)

    return masked_image


def detect_line_segments(cropped_canny):
    # Setting Rho, Angle and MinThreshold for HoughLine Transform
    rho = 1 # It is precision in pixel, 1 represents 1 pixel of precision
    angle = np.pi/180 # it is Degree in radian, np.pi/180 represents 1 degree
    minThreshold = 20 # Minimum number of votes to be considered as a line

    line_segments = cv.HoughLinesP(
        cropped_canny, 
        rho, angle, 
        minThreshold, 
        np.array([]), 
        minLineLength=10, 
        maxLineGap=5
    )

    if line_segments is not None:
        for line_segment in line_segments:
            log.info('detected line_segment:')
            log.info("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments


def average_slope_intercept(frame, line_segments):
    '''
    This Function is used to combine the total line segments to one or two lane lines
    If the Slopes of all the line segment are < 0: Then we have detected a Left Lane Line
    If the Slopes of all the line segment are > 0: Then we have detected a Right Lane Line
    '''

    lane_lines = []
    if line_segments is None:
        log.warning("No Lane Lines detected")
        return lane_lines

    _, width, _ = frame.shape
    left_fit = []
    right_fit = []

    # Defining the Boundary of the Left and Right region
    boundary = 3/5
    left_region_boundary = width * (1 - boundary) # Left Lane line should be on left 2/3 of the screen; region ends at width * 2 / 3
    right_region_boundary = width * boundary # Right Lane Line should be on right 2/3 of the screen; region starts at width * 1 / 3

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            # Eliminating Vertical lines as their slopes = infinite
            if x1 ==x2: 
                log.info("Vertical Line Detected (Slope = Infinity): %s" % line_segment)
                continue

            parameter = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameter[0]
            intercept = parameter[1]

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0: # If only left lanes exist append in Lane Lines
        left_line = make_coordinates(frame, left_fit_average)
        lane_lines.append(left_line)

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0: # If only right lanes exist append in Lane Lines
        right_line = make_coordinates(frame, right_fit_average)
        lane_lines.append(right_line)

    log.info('lane lines: %s' % lane_lines)  # Displaying the lane lines

    return lane_lines


def compute_steering_angle(frame, lane_lines):
    '''
    This Function finds the steering angle based on the Coordinates of the lane lines
    The only assumption is that the camera is pointing the dead center of the car
    '''

    if len(lane_lines) == 0:
        log.warning("No Lane Lines Found, Do Nothing!")
        return 90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        log.info("Only one lane Line Detected, Following it %s" % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_offset_percentage = 0.00 # 0.00: camera pointing to center; +0.03: camera pointing to right; -0.03: camera pointing to left

        mid = int(width / 2 * (1 + camera_offset_percentage))
        x_offset = (left_x2 + right_x2)/2 - mid
    
    y_offset = int(height / 2)
    
    '''
    Steering angle is basically the angle at which the car turns
    It is calculated by first calculating middle line i.e (x1=mid, y1=height) and (x2=(left_x2+right_x2)/2, y2=height/2)
    Now after finding the x and y offsets of the line we can determine the angle by using Tan inverse of x_offest/yoffset
    Note: the angle determined is respect to the vertical line passing through the center
    '''
    
    angle_to_mid_radian = atan(x_offset / y_offset) # Angle to Center Vertical Line (in Radian) 
    angle_to_mid_degree = int(angle_to_mid_radian * 180.0 / np.pi) # Angle to Center Vertical Line (in Degree) 
    steering_angle = 90 - angle_to_mid_degree # Final Steering Angle Needed by the car (90-angle because my servo is horizontally flipped)


    # My Servo Limitation is from 60 to 120 else it will exert more torque 
    if steering_angle >= 120:
        steering_angle = 120
    elif steering_angle <= 60:
        steering_angle = 60

    #log.debug('new steering angle: %s' % steering_angle)
    # print(f"The New Steering Angle is {steering_angle}")
    return steering_angle


def stabilizing_steering_angle(
    curr_steering_angle, 
    new_steering_angle, 
    num_of_lane_lines, 
    max_angle_deviation_for_two_lines = 10, 
    max_ngle_deviation_for_one_line = 1):

    '''
    This Function Stablizes the Steering angle so that the car doesn't abruptly change the Angles
    If new_steering_angle is greater than max_angle_deviation only turn by max_angle_deviation
    Else if new_steering_angle is less than max_angle_deviation turn by new_steering_angle
    '''

    if num_of_lane_lines ==1:
        # If only one lane line is detected we cannot deviate much as there is no precision
        max_angle_deviation = max_ngle_deviation_for_one_line
    
    else:
        # If two lane lines are detected, we can deviate more as there is precision in turning
        max_angle_deviation = max_angle_deviation_for_two_lines

    # Calculating the Angle of Deviation 
    angle_deviation = new_steering_angle - curr_steering_angle

    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
    
    else:
        stabilized_steering_angle = new_steering_angle

    log.debug('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    # print(f"Proposed angle: {new_steering_angle}, Stabilized Angle: {stabilized_steering_angle}")
    return stabilized_steering_angle



############################
# Utlity Functions
############################

def make_coordinates(frame, line_parameters):
    height, width, _ = frame.shape
    slope, intercept = line_parameters

    # Taking the coordinates of the Line
    y1 = height # bottom of the frame
    y2 = int(height * 1 / 2) # Middle of the frame

    # Bounding the Coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))

    # Returning as 2D Array to make function general
    return [[x1, y1, x2, y2]] 


def display_lines(frame, lines, linecolor = (0, 255, 0), linewidth = 10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv.line(line_image, (x1, y1), (x2, y2), linecolor, linewidth)

    line_image = cv.addWeighted(frame, 0.8, line_image, 1, 1)

    return line_image
    

def display_heading_line(frame, steering_angle, line_color = (0, 0, 255), line_width = 5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    '''
    This Function figures out the heading line (line at which the car is headed) from steering angle
    The (x1, y1) coordinate of the line is middle bottom of the frame (width / 2, height)
    The (x2, y2) can be obtained using tan()
    '''

    '''
    Note: The steering angle of:
    0-90: Turn Right
    90: Go Straight
    91-180: Turn Left
    '''

    # This is done because my servo is different
    reversed_steering_angle = 180 - steering_angle

    steering_angle_radian = reversed_steering_angle / 180 * np.pi # Converting Degree to radian as tan() accepts radian

    # Finding Coordinates
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / tan(steering_angle_radian))
    y2 = int(height / 2)

    cv.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def show_image(title, frame, show=_SHOW_IMAGE):
    # This function is for ease; It simultaneaously turns all cv.imshow() ON/OFF
    if show:
        cv.imshow(title, frame)


############################
# Testing Functions
############################

def test_photo(photo):
    # Creating Lane Follower Object
    lane_follower = LaneFollower()

    try: 
        frame = cv.imread(photo)
        final_image = lane_follower.follow_lane(frame)
        show_image("Final Image", final_image, show=True)
    finally:
        cv.waitKey(0)  
        cv.destroyAllWindows()


def test_video(video):
       
    # Creating Lane Follower Object
    lane_follower = LaneFollower(car=False)

    # Starting the FPS counter
    fps = FPS().start()

    cap = cv.VideoCapture(video)

    fourCC = cv.VideoWriter_fourcc(*'mpeg')
    out = cv.VideoWriter(video + '_final.mp4', fourCC, 60.0, (640, 480))

    try:
        i=1
        while cap.isOpened():
            isSuccess, frame = cap.read()
             
            if not isSuccess:
                print("Video Ended!")
                break
            
            frame = cv.resize(frame, (640, 480))
            show_image("OriginalVid", frame, show=True)

            final_frame = lane_follower.follow_lane(frame)

            if len(final_frame) == None:
                print("No Lane Lines Detected, Ending Video")
                break

            out.write(final_frame)             
            show_image("Final Video", final_frame, show=True)
            
            fps.update()
            i += 1 

            key = cv.waitKey(1) & 0xFF  
            if key == 27:
                break
            

    finally:
        fps.stop()
        print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
        
        # PiCar.cleanup()
        cap.release()
        # out.release()
        cv.destroyAllWindows()


def test_webcam(src=0, _INITIAL_SPEED=0):
    
    # Creating RPiCar Object
    PiCar.motor(speed=_INITIAL_SPEED)

    width = 320
    height = 240

    # Creating Lane Follower Object
    lane_follower = LaneFollower(car=True)

    # Starting the FPS counter
    fps = FPS().start()

    cap = cv.VideoCapture(src)
    cap.set(3, width)
    cap.set(4, height)

    #fourCC = cv.VideoWriter_fourcc(*'mpeg')
    #out = cv.VideoWriter('Webcam_final.mp4', fourCC, 60.0, (340, 240))

    try:
        i=1
        while cap.isOpened():
            isSuccess, frame = cap.read()
             
            if not isSuccess:
                print("Video Ended!")
                break
           
            # frame = cv.resize(frame, (340, 240))
            show_image("OriginalVid", frame, show=False)

            final_frame = lane_follower.follow_lane(frame)

            #if len(final_frame) == None:
             #   print("No Lane Lines Detected, Ending Video!")
              #  PiCar.cleanup()

            # out.write(final_frame)
            show_image("Final Video", final_frame, show=True)

            fps.update()
            i += 1 

            key = cv.waitKey(1) & 0xFF  
            if key == 27:
                break
            

    finally:
        fps.stop()
        print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
        print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
        
        PiCar.cleanup()
        cap.release()
        # out.release()
        cv.destroyAllWindows()


if __name__ == '__main__':
    log.basicConfig(level=log.ERROR)
    
    # test_photo('Data/test_image.jpg')
    # test_video('Data/Output.avi')
    test_video('Data/Track Test/track_test_01.mp4')
    # test_webcam()

