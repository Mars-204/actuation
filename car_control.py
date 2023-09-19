# Import modules
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import os
import numpy as np
import math
import threading
import queue
from enum import Enum, auto
import warnings
from CarAPI import CarApi
import imutils
import sys
import importlib.util
from client_demo_main import reciever_server


# ------ CONSTANTS ------ #
# Constants
C_ANGLE_NOT_CALC = -1000

# -- Configuration values
# Movement
CAR_SPEED_CAP = 70                 # 85 Max speed of the car (% upper saturation)
SERVO_PERC_COEF = 1.05               # Coeficient of servo percentage increase (increase turning steepness)
MOTOR_MIN_PERC = 40                 # Minimum percentual speed of the motor
MOTOR_SPEED_TURN_COEF = 0.5         # Coeficient of motor percentage decrease by servo percentage (lower is faster)
SERVO_LPF_COEF = 0.2                # LPF coeficient for smooth servo control
MIN_DISTANCE_CM = 35                # Threshold for ultrasonic sensor at which the vehicle stops
# Image processing
BINARY_THRESH = 150                 # Threshold for image binarization
TEMPL_MATCH_THRESH = 0.7            # Threshold for template matching
IMG_BOTTOM_KEEP_PORTION = 2/5       # Portion of how much of the image from the bottom stays (to remove bumper)
CAMERA_BRIGHTNESS = 50
CAMERA_CONTRAST = 50
CAMERA_EXPOSURE = 0


# Only for template matching configuration
MATCH_EDGED_IMG = False             # Perform template matching on image converted to edges
NUM_MATCH_SCALES = 5                # Number of scales between 20% and 100% of the camera image on which template is matched
TEMPLATE_PATH_STOP = "/home/pi/Desktop/signs_detection/img/template_stop.png"
TEMPLATE_PATH_SPEED = "/home/pi/Desktop/signs_detection/img/template_speed.png"
TEMPLATE_PATH_ARROW = "/home/pi/Desktop/signs_detection/img/template_arrow.png"

SHOW_BINARY = False
SHOW_EDGES = False

# ------ CLASSES ------ #
class QueueItemType(Enum):
    ServoPerc = auto()
    UserQuit = auto()
    ImgCamera = auto()
    ImgLanes = auto()
    ImgEdges = auto()
    ImgBinary = auto()
    ImgSigns = auto()
    SignStop = auto()
    SignEv = auto()
    SignLimit30 = auto()
    SignTr1 = auto()
    SignTr2 = auto()
    SignTr3 = auto()
    SignTr4 = auto()
    SignTr5 = auto()
    SignTr6 = auto()
    SignTr7 = auto()
    SignTr8 = auto()
    SignTr9 = auto()
    SignCarA = auto()
    SignCarB = auto()
    SignCarC = auto()
    SignCarD = auto()
    SignCarE = auto()
    SignCarF = auto()
    
def getQueueObj(label): 
    labelObjDict = {
        "car-a": QueueItemType.SignCarA,
        "car-b": QueueItemType.SignCarB,
        "car-c": QueueItemType.SignCarC,
        "car-d": QueueItemType.SignCarD,
        "car-e": QueueItemType.SignCarE,
        "car-f": QueueItemType.SignCarF,
        "limit-30": QueueItemType.SignLimit30,
        "stop": QueueItemType.SignStop,
        "ev": QueueItemType.SignEv,
        "tr-1": QueueItemType.SignTr1,
        "tr-2": QueueItemType.SignTr2,
        "tr-3": QueueItemType.SignTr3,
        "tr-4": QueueItemType.SignTr4,
        "tr-5": QueueItemType.SignTr5,
        "tr-6": QueueItemType.SignTr6,
        "tr-7": QueueItemType.SignTr7,
        "tr-8": QueueItemType.SignTr8,
        "tr-9": QueueItemType.SignTr9
    }
    return labelObjDict[label]
        

# ------ GLOBAL VARIABLES ------ #
_q_binary_image = queue.Queue()
# _q_lanes_detection = queue.Queue()
_s_lanes_detection = []
_q_driving_control = queue.Queue()
_q_signs_detection = queue.Queue()
_q_gui = queue.Queue()
_q_edges_image = queue.Queue()


# ------ FUNCTIONS: LANES DETECTION ------ #
def execLanesDetection():
    # Initialize camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.brightness = CAMERA_BRIGHTNESS
    camera.contrast = CAMERA_CONTRAST
    camera.exposure_compensation = CAMERA_EXPOSURE

    camera.framerate = 30
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # Let camera warm up
    time.sleep(0.2)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # --- Image acquisition
        # Copy the captured image and clear the buffer
        img_camera = frame.array
        rawCapture.truncate(0)

        # Crop the image off the bumber part
        height, width, _ = img_camera.shape
        img_crop = img_camera[:(height-190), :]
        
        # Send image to signs detection
        if _q_signs_detection.qsize() == 0:
            _q_signs_detection.put_nowait([QueueItemType.ImgCamera, img_crop])

        # Additionally crop the image off outmost sides
        #img_crop = img_crop[:, 50:(width-50)]


        try:
            # Process image to find lanes
            lane_lines = detectLanes(img_crop)

            # Check that the found lanes are reasonable
            if lane_lines is None or (len(lane_lines) < 1 or len(lane_lines) > 2):
                raise Exception()

            # Calculate the steering angle and servo percentage
            steering_angle = calcSteeringAngle(img_crop, lane_lines)
            if steering_angle == C_ANGLE_NOT_CALC:
                raise Exception()

            # Recalculate angle to servo percentage
            servo_perc = (steering_angle/90 * 100) * SERVO_PERC_COEF
            servo_perc = min(max(servo_perc, -100), 100)

            # Visualize the navigation
            img_lanes = drawLines(img_crop, lane_lines)
            img_result = drawHeadingLine(img_lanes, steering_angle)

            # Send the result to driving control
            _q_driving_control.put_nowait([QueueItemType.ServoPerc, servo_perc])
        except:
            img_result = img_crop

        _s_lanes_detection.append(img_result)

        # Check for the user-quit request
        '''
        if _q_lanes_detection.qsize() > 0:
            qitem = _q_lanes_detection.get()
            if qitem[0] is QueueItemType.UserQuit:
                break
        '''
    camera.close()



def detectLanes(img_crop):
    # --- Image preprocessing - edge detection
    # Convert the image grayscale, binarize it and detect sharp edges
    img_gray = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
    img_binary = cv2.threshold(img_gray, BINARY_THRESH, 255, cv2.THRESH_BINARY)[1]
    img_edges = cv2.Canny(img_binary, 200, 400)
    
    if (SHOW_BINARY):
        # Send image to binary gui
        if _q_binary_image.qsize() == 0:
            _q_binary_image.put_nowait([QueueItemType.ImgBinary, img_binary])

    if (SHOW_EDGES): 
        # Send image to binary gui
        if _q_edges_image.qsize() == 0:
            _q_edges_image.put_nowait([QueueItemType.ImgEdges, img_edges])

    # --- Image preprocessing - partial image focus/suppression
    # Define the suppressed range as a polygon
    height, width = img_edges.shape
    polygon = np.array([[
        (0, height * (1-IMG_BOTTOM_KEEP_PORTION)),
        (width, height * (1-IMG_BOTTOM_KEEP_PORTION)),
        (width, height),
        (0, height)
    ]], np.int32)

    # Create a suppresion mask and AND the image with it
    mask = np.zeros_like(img_edges)
    cv2.fillPoly(mask, polygon, 255)
    img_edges_focused = cv2.bitwise_and(img_edges, mask)


    # --- Image processing - detection of lines
    # Use Hough transform to collect continuous line segments
    rho = 1  # radius resolution
    theta = np.pi/180  # angle resolution
    line_threshold = 10  # characteristic setting of Hough transform - minimum number of intersections
    line_segments = cv2.HoughLinesP(img_edges_focused, rho, theta, line_threshold, minLineLength=30, maxLineGap=5)


    # --- Evaluating lines to identify driving lanes
    # Check that there is data to process
    if line_segments is None:
        return []

    # Define a boundary for positions of left and right lane
    _, width, _ = img_crop.shape
    boundary_coef = 1/3
    left_region_boundary = width * (1 - boundary_coef)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary_coef # right lane line segment should be on left 2/3 of the screen

    # Iterate over all line segments
    left_fit = []
    right_fit = []
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            # Skip a line that is too horizontal
            if abs(y2 - y1) < 20:
                continue
            # Fit the segment with the line equation
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            # Rate the line's direction and location
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    # Calculate average lanes for left and right
    lane_lines = []
    if len(left_fit) > 0:
        left_fit_average = np.average(left_fit, axis=0)
        lane_lines.append(makeLinePoints(img_crop, left_fit_average))

    if len(right_fit) > 0:
        right_fit_average = np.average(right_fit, axis=0)
        lane_lines.append(makeLinePoints(img_crop, right_fit_average))

    return lane_lines


def calcSteeringAngle(img_crop, lane_lines):
    # Define some dimensional values
    height, width, _ = img_crop.shape
    x_offset = 0
    y_offset = int(height / 2)
    mid = int(width/2)

    # Calculate x and y dimensions for one or two lanes
    if len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    elif len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        x_offset = (left_x2 + right_x2) / 2 - mid
        if left_x2 > right_x2:
            return C_ANGLE_NOT_CALC
    
    # Finally calculate
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)

    if angle_to_mid_deg > 90 or angle_to_mid_deg < -90:
        angle_to_mid_deg = C_ANGLE_NOT_CALC

    return angle_to_mid_deg


def makeLinePoints(image, line):
    height, width, _ = image.shape
    y1 = height  # bottom of the image
    y2 = int(y1 * 1 / 2)  # make points from middle of the image down

    slope, intercept = line
    # bound the coordinates within the image
    try:
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    except OverflowError:
        x1 = 0
        x2 = width
        
    return [[x1, y1, x2, y2]]


def drawLines(image, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(image, 0.8, line_image, 1, 1)
    return line_image


def drawHeadingLine(image, steering_angle, line_color = (0, 0, 255), line_width = 5):
    heading_image = np.zeros_like(image)
    height, width, _ = image.shape
    
    steering_angle_radian = (steering_angle + 90) / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)
    
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(image, 0.8, heading_image, 1, 1)
    
    return heading_image


# ------ FUNCTIONS: NN SIGNS DETECTION ------ #

class NeuralCamera:
    def __init__(self):
        # self._camera    = PiCamera()
        self._image     = None
        self._nnImage   = None
        self._fpsCalc   = None

#    @property
#    def camera(self):
#        return self._camera
        
    @property
    def image(self):
        return self._image
        
    @image.setter
    def image(self, value):
        # self._image = cv2.flip(value.array, 0)
        # self._image = value.array
        # self._image = cv2.resize(value, (320, 145))
        self._image = value

    @property
    def nnImage(self):
        return self._nnImage
        
    @image.setter
    def nnImage(self, value):
        self._nnImage = value

    @property
    def fpsCalc(self):
        return self._fpsCalc
        
    @image.setter
    def fpsCalc(self, value):
        self._fpsCalc = float(value)

    def nnSetup(self): 
        # Neural Network Configuration
        MODEL_NAME         = "/home/pi/Desktop/Solution_task_1_2/MyModels/customNN3"
        GRAPH_NAME         = "detect_quant.tflite"
        LABELMAP_NAME      = "labelmap.txt"
        self._min_conf_threshold = float(0.45)
        self._imW = int(640)
        self._imH = int(290)
        # self._imW = int(320)
        # self._imH = int(145)

        # Import TensorFlow libraries
        pkg = importlib.util.find_spec('tflite_runtime')
        if pkg:
            from tflite_runtime.interpreter import Interpreter
        else:
            from tensorflow.lite.python.interpreter import Interpreter
            
        # Path to .tflite file, which contains the model that is used for object detection
        CWD_PATH     = os.getcwd()
        PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

        # Path to label map file
        PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

        # Load the label map
        with open(PATH_TO_LABELS, 'r') as f:
            self._labels = [line.strip() for line in f.readlines()]
            
        # Load the Tensorflow Lite model.
        self._interpreter = Interpreter(model_path=PATH_TO_CKPT)
        self._interpreter.allocate_tensors()

        # Get model details
        self._input_details  = self._interpreter.get_input_details()
        self._output_details = self._interpreter.get_output_details()
        self._height         = self._input_details[0]['shape'][1]
        self._width          = self._input_details[0]['shape'][2]

        self._floating_model = (self._input_details[0]['dtype'] == np.float32)

        input_mean = 127.5
        input_std = 127.5

        # Check output layer name to determine if this model was created with TF2 or TF1,
        # because outputs are ordered differently for TF2 and TF1 models
        outname = self._output_details[0]['name']

        if ('StatefulPartitionedCall' in outname): # This is a TF2 model
            self._boxes_idx, self._classes_idx, self._scores_idx = 1, 3, 0
        else: # This is a TF1 model
            self._boxes_idx, self._classes_idx, self._scores_idx = 0, 1, 2


    def setup(self):
#        self._camera.resolution = (360, 240)
#        self._camera.framerate = 40
#        self._camera.brightness = 50
#        self._camera.saturation = 50
#        self._camera.contrast = 40
        self.nnSetup()


    def recognizeObjects(self):
        
        # Acquire frame and resize to expected shape [1xHxWx3]
        frame         = self._image.copy()
        frame_rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (self._width, self._height))
        input_data    = np.expand_dims(frame_resized, axis=0)        

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self._floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        self._interpreter.set_tensor(self._input_details[0]['index'],input_data)
        self._interpreter.invoke()

        # retrieve detection results
        boxes   = self._interpreter.get_tensor(self._output_details[self._boxes_idx]['index'])[0]   # Bounding box coordinates of detected objects
        classes = self._interpreter.get_tensor(self._output_details[self._classes_idx]['index'])[0] # Class index of detected objects
        scores  = self._interpreter.get_tensor(self._output_details[self._scores_idx]['index'])[0]  # Confidence of detected objects

        self._nnImage = frame

        object_name = None

        # loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > self._min_conf_threshold) and (scores[i] <= 1.0)):
                
                # get bounding box coordinates and draw box
                ymin = int(max(1,(boxes[i][0]   * self._imH)))
                xmin = int(max(1,(boxes[i][1]   * self._imW)))
                ymax = int(min(self._imH,(boxes[i][2] * self._imH)))
                xmax = int(min(self._imW,(boxes[i][3] * self._imW)))
                cv2.rectangle(frame, (xmin,ymin), (xmax, ymax), (10, 255, 0), 2)

                # draw label
                object_name         = self._labels[int(classes[i])] # Look up object name from "labels" array using class index
                label               = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin          = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(self._nnImage, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(self._nnImage, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                
        # draw framerate
        cv2.putText(self._nnImage,'FPS: {0:.2f}'.format(self._fpsCalc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
        # self.showFrame()
        return self._nnImage, object_name

def execNNSignsDetection():     
    # initialize the camera and grab a reference to the raw camera capture
    myCamHandler = NeuralCamera()
    myCamHandler.setup()
    # rawCapture = myCamHandler.capture()
    # myCam = myCamHandler.camera
    
    # Initialize frame rate calculation
    myCamHandler.fpsCalc = 1
    freq = cv2.getTickFrequency()
    
    while True:
        # Dequeue an item
        qitem = _q_signs_detection.get()


        # Queue item is an image to process
        if qitem[0] is QueueItemType.ImgCamera:
            # Cut out only the region of interest (upper left corner)
            img = qitem[1]
            height, width, _ = img.shape
            # print("height= " + str(height)) 
            # print("width= " + str(width))
            # img = img[:int(height/2), :int(width/2)]

            # Start timer (for calculating frame rate)
            t1 = cv2.getTickCount()
           
            myCamHandler.image = img
            image = myCamHandler.image

            resImg, objectLabel = myCamHandler.recognizeObjects()
            
            t2    = cv2.getTickCount()
            time1 = (t2-t1)/freq
            myCamHandler.fpsCalc= 1/time1  
            
            if (objectLabel is not None):
                _q_driving_control.put([getQueueObj(objectLabel)])
            # Send image to GUI
            _q_gui.put_nowait([QueueItemType.ImgSigns, resImg])
            
        # Queue item is user-initiated quit
        elif qitem[0] is QueueItemType.UserQuit:
            break


'''
# ------ FUNCTIONS: SIGNS DETECTION ------ #
def execSignsDetection():
    # Load templates of signs
    img_template_stop = cv2.imread(TEMPLATE_PATH_STOP) if os.path.exists(TEMPLATE_PATH_STOP) else None
    img_template_speed = cv2.imread(TEMPLATE_PATH_SPEED) if os.path.exists(TEMPLATE_PATH_SPEED) else None
    img_template_arrow = cv2.imread(TEMPLATE_PATH_ARROW) if os.path.exists(TEMPLATE_PATH_ARROW) else None

    while True:
        # Dequeue an item
        qitem = _q_signs_detection.get()

        # Queue item is an image to process
        if qitem[0] is QueueItemType.ImgCamera:
            # Cut out only the region of interest (upper left corner)
            img = qitem[1]
            height, width, _ = img.shape
            img = img[:int(height/2), :int(width/3)]

            # Apply template matching
            if img_template_stop is not None:
                if matchTemplate(img, img_template_stop, TEMPL_MATCH_THRESH, NUM_MATCH_SCALES, MATCH_EDGED_IMG):
                    _q_driving_control.put([QueueItemType.SignStop])
            if img_template_speed is not None:
                if matchTemplate(img, img_template_speed, TEMPL_MATCH_THRESH, NUM_MATCH_SCALES, MATCH_EDGED_IMG):
                    _q_driving_control.put([QueueItemType.SignSpeed])
            if img_template_arrow is not None:
                if matchTemplate(img, img_template_arrow, TEMPL_MATCH_THRESH, NUM_MATCH_SCALES, MATCH_EDGED_IMG):
                    _q_driving_control.put([QueueItemType.SignArrow])

            # Send image to GUI
            _q_gui.put_nowait([QueueItemType.ImgSigns, img])

        # Queue item is user-initiated quit
        elif qitem[0] is QueueItemType.UserQuit:
            break



def matchTemplate(img_camera, img_template, threshold, num_scales, edged=False):
    templ_size_X, templ_size_Y = img_template.shape[:2]
    best_res = None
    # Loop over scales of the camera image
    for scale in np.linspace(0.8, 1.2, num_scales)[::-1]:
        # Resize the image and save the ratio
        img_camera_resized = imutils.resize(img_camera, width=int(img_camera.shape[1] * scale))
        ratio = img_camera.shape[1] / float(img_camera_resized.shape[1])

        # Stop if resized image is smaller than template
        if img_camera_resized.shape[0] < templ_size_X or img_camera_resized.shape[1] < templ_size_Y:
            break
        
        # Perform matching by OpenCV
        if edged:
            match_res = cv2.matchTemplate(convertToEdges(img_camera_resized), convertToEdges(img_template), cv2.TM_CCOEFF_NORMED)
        else:
            match_res = cv2.matchTemplate(img_camera_resized, img_template, cv2.TM_CCOEFF_NORMED)
        (_, val, _, loc) = cv2.minMaxLoc(match_res)

        # If new maximum is found, update the result
        if best_res is None or val > best_res[0]:
            best_res = (val, loc, ratio)

    # If the results is good enough, mark it by rectangle
    (val, loc, ratio) = best_res
    if val > threshold:
        match_start_X, match_start_Y = (int(loc[0] * ratio), int(loc[1] * ratio))
        match_end_X, match_end_Y = (int((loc[0] + templ_size_X) * ratio), int((loc[1] + templ_size_Y) * ratio))
        
        cv2.rectangle(img_camera, (match_start_X, match_start_Y), (match_end_X, match_end_Y), (0, 0, 255), 2)
        cv2.putText(img_camera, "{:.1f}".format(val), (match_start_X, match_start_Y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        return True
    else:
        return False

'''
def convertToEdges(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.Canny(img, 50, 200)
    return img




# ------ FUNCTIONS: DRIVING CONTROL ------ #
def execDrivingControl():
    # Initialize variables
    car_api = CarApi()
    global CAR_SPEED_CAP
    servo_perc_prev = 0
    time_release_stop = 2
    time_release_exit = 2

    # Variables to execute action
    isDetectionUpdate = False
    isStop = False
    isExit = False
    exit_turn_angle = 100
    exit_motor_power = MOTOR_MIN_PERC


    # Start distance measurement
    car_api.startContDistMeas()

    # Set servo to straight position
    car_api.setSteeringAngle(0)

    while True:

        # car_clint_detect
        # Dequeue an item
        qitem = _q_driving_control.get()

        # Queue item is user-initiated quit
        if qitem[0] is QueueItemType.UserQuit:
            break

        # If there is an obstacle or STOP sign, stay still
        # if car_api.getLastDistance() < MIN_DISTANCE_CM or time.perf_counter() < time_release_stop:
        #     car_api.setMotorPower(0)
        #     continue

        # If there is a command from server, update isStop and isExit (exit at EV sign)
        # --Receiving stop command queue--
        # if qitem[0] is QueueItemType.StopCommand:
        #   isStop = True
        # --Receiving EV sign (exit) command queue--
        # if qitem[0] is QueueItemType.ExitCommand:
        #   isExit = True

        # Queue item is servo percentage
        if qitem[0] is QueueItemType.ServoPerc:
            # If there's command from server to stop
            if isStop:
                car_api.setSteeringAngle(0)
                car_api.setMotorPower(0)
                time.sleep(time_release_stop)
                isStop = False
            # Normal operation: updating steering angle and motor power based the detection algorithm
            else:
                # Use LPF to smooth the percentage
                servo_perc = SERVO_LPF_COEF*servo_perc_prev + (1 - SERVO_LPF_COEF)*qitem[1]
                servo_perc_prev = servo_perc
                car_api.setSteeringAngle(servo_perc)

                # Calculate the motor percentual power
                motor_perc = min(100 - abs(servo_perc)*MOTOR_SPEED_TURN_COEF + MOTOR_MIN_PERC, CAR_SPEED_CAP)
                car_api.setMotorPower(motor_perc)

        # Queue item is a sign
        elif qitem[0] is QueueItemType.SignEv:
            # If there's command from server to exit the circuit at EV sign
            if isExit:
                # Turn the car
                car_api.setSteeringAngle(exit_turn_angle)
                car_api.setMotorPower(exit_motor_power)
                time.sleep(time_release_exit)
                # Go straight exiting the track
                car_api.setSteeringAngle(0)
                car_api.setMotorPower(exit_motor_power)
                time.sleep(time_release_exit)
                isExit = False
                # print("EV Sign")
            else:
                # print("EV Sign")
                pass
        elif qitem[0] is QueueItemType.SignStop:
            # time_release_stop = time.perf_counter() + 4  # Wait for X seconds
            pass
            # print("STOP")
        elif qitem[0] is QueueItemType.SignLimit30:
            # CAR_SPEED_CAP = 60  # Limit max speed
            # print("LIMIT30")
            pass
        elif qitem[0] is QueueItemType.SignTr1:
            pass
            # print("TRACK POS 1")
        elif qitem[0] is QueueItemType.SignTr2:
            # print("TRACK POS 2")
            pass
        elif qitem[0] is QueueItemType.SignTr3:
            # print("TRACK POS 3")
            pass 
        elif qitem[0] is QueueItemType.SignTr4:
            # print("TRACK POS 4")
            pass
        elif qitem[0] is QueueItemType.SignTr5:
            #print("TRACK POS 5")
            pass
        elif qitem[0] is QueueItemType.SignTr6:
            # print("TRACK POS 6")              
            pass
        elif qitem[0] is QueueItemType.SignTr7:
            # print("TRACK POS 7")
            pass
        elif qitem[0] is QueueItemType.SignTr8:
            # print("TRACK POS 8")
            pass
        elif qitem[0] is QueueItemType.SignTr9:
            # print("TRACK POS 9")
            pass
        elif qitem[0] is QueueItemType.SignCarA:
            # print("FRONT CAR A")                
            pass
        elif qitem[0] is QueueItemType.SignCarB:
            # print("FRONT CAR B")                            
            pass
        elif qitem[0] is QueueItemType.SignCarC:
            # print("FRONT CAR C")        
            pass
        elif qitem[0] is QueueItemType.SignCarD:
            # print("FRONT CAR D")        
            pass
        elif qitem[0] is QueueItemType.SignCarE:
            # print("FRONT CAR E")        
            pass 
        elif qitem[0] is QueueItemType.SignCarF:
            # print("FRONT CAR F")        
            pass


    # Leave the actuators in a neutral state
    car_api.setMotorPower(0)
    car_api.setSteeringAngle(0)
    car_api.stopContDistMeas()




# ------ FUNCTIONS: GUI ------ #
def execGui():
    print("Press q to quit")
    while True:
        # Dequeue an item
        qitem = _q_gui.get()

        # Queue item is an image
        if qitem[0] is QueueItemType.ImgSigns:
            cv2.namedWindow("Signs detection")
            cv2.moveWindow("Signs detection",  0, 0)
            cv2.imshow("Signs detection", qitem[1])
        
        
        if (_s_lanes_detection):
            slanes = _s_lanes_detection.pop()
            cv2.namedWindow("Lanes detection")
            cv2.moveWindow("Lanes detection", 680, 0)
            cv2.imshow("Lanes detection",slanes) 
            _s_lanes_detection.clear()
        
        if(SHOW_BINARY):
            qitemBinary = _q_binary_image.get()
            if qitemBinary[0] is QueueItemType.ImgBinary:
                cv2.namedWindow("Binary detection")
                cv2.moveWindow("Binary detection", 0, 500)
                cv2.imshow("Binary detection", qitemBinary[1])

        if(SHOW_EDGES):
            qitemEdges = _q_edges_image.get()
            if qitemEdges[0] is QueueItemType.ImgEdges: 
                cv2.namedWindow("Edge detection")
                cv2.moveWindow("Edge detection", 680, 500)								
                cv2.imshow("Edge detection", qitemEdges[1])

        # Watch for the user-initiated quit
        key = cv2.waitKey(1)
        if key == ord("q"):
            _q_driving_control.put([QueueItemType.UserQuit])
            _q_signs_detection.put([QueueItemType.UserQuit])
            # _q_lanes_detection.put([QueueItemType.UserQuit])
            print("Quitting")
            break

    cv2.destroyAllWindows()



# ------ MAIN CODE ------ #
warnings.filterwarnings("ignore")

thread_lanes_detection = threading.Thread(target=execLanesDetection)
thread_lanes_detection.start()

# thread_signs_detection = threading.Thread(target=execSignsDetection)
thread_signs_detection = threading.Thread(target=execNNSignsDetection)
thread_signs_detection.start()

thread_driving_control = threading.Thread(target=execDrivingControl)
thread_driving_control.start()

thread_recieve_server = threading.Thread(target=reciever_server)
thread_recieve_server.start()

thread_gui = threading.Thread(target=execGui)
thread_gui.start()
