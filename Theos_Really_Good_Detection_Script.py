#!python3
######## Webcam Object Detection Using Tensorflow-trained Classifier #########
#
# Author: Evan Juras
# Date: 9/28/19
# Description:
# This program uses a TensorFlow Lite object detection model to perform object
# detection on an image or a folder full of images. It draws boxes and scores
# around the objects of interest in each image.
#
# This code is based off the TensorFlow Lite image classification example at:
# https://github.com/tensorflow/tensorflow/blob/master/tensorflow/lite/examples/python/label_image.py
#
# I added my own method of drawing boxes and labels using OpenCV.
#
# ----------------------------------------------------------------------------
#
# Modified By: Dillon Wall
# dillon.wall@oit.edu
# Date: 4/26/2020
#
#
# We can use the C++ Python API to embed a python interpreter into C++ which will allow us to get the return value
# from a specific python function, and therefore run inference on single images using this script
#
# I refactored the code to be completely wrapped in functions so that we can call image processing on demand
#       to evaluate an image and get a value returned to C++
#
# Modified By: Theodor Giles
# farmergilest@outlook.com
# First Modified: 8/3/20
# Last Edited 8/5/20
#
#
# Distance calculation and integration into other programs/classes/handlers for running the RoboSub
# Some standardization and future algorithms possibly not using the Tensorflow AI
#

# use newer python

# Import packages *********************************************************************************
import os
# import argparse
# from typing import List
import math
import cv2
import numpy as np
import sys
import time
from threading import Thread
import importlib.util


# import tensorflow as tf


# Define VideoStream class to handle streaming of video from webcam in separate processing thread Source - Adrian
# Rosebrock, PyImageSearch: https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/
class VideoStream:
    """Camera object that controls video streaming from the Picamera"""

    def __init__(self, resolution=(640, 480)):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3, resolution[0])
        ret = self.stream.set(4, resolution[1])

        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

        # Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
        # Start the thread that reads frames from the video stream
        self.thread = Thread(target=self.stopped, args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # Return the most recent frame
        return self.frame

    def stop(self):
        # Indicate that the camera and thread should be stopped
        self.stopped = True


class Detector:
    frame_rate_calc = 0.0
    freq = 0.0
    width = 0.0
    height = 0.0

    # *************************************************************************************************
    # Initialize environment variables and set things up for image processing.
    # Returns: None
    # Thread Safety: None
    # *************************************************************************************************

    def __init__(self, model_dir, show_images=False, resolution='640x480', graph='model.tflite',
                 labelmap_name='labelmap.txt',
                 threshold=0.5, edgetpu=False):
        self.Directory = os.getcwd()
        resW, resH = resolution.split('x')
        self.imageWidth, self.imageHeight = int(resW), int(resH)

        self.Focal_LENGTH = 3.6  # mm
        self.Model_NAME = 'model.tflite'
        self.ShowImages = show_images
        self.Graph_NAME = graph
        LABELMAP_NAME = labelmap_name
        self.min_conf_threshold = float(threshold)
        # Get path to current working directory
        self.CWD_PATH = os.getcwd()
        pkg = importlib.util.find_spec('tensorflow')
        if pkg is None:
            from tflite_runtime.interpreter import Interpreter
            if edgetpu:
                from tflite_runtime.interpreter import load_delegate
        else:
            from tensorflow.lite.python.interpreter import Interpreter
            if edgetpu:
                from tensorflow.lite.python.interpreter import load_delegate

        # If using Edge TPU, assign filename for Edge TPU model
        if edgetpu:
            # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
            if self.Graph_NAME == 'detect.tflite':
                self.Graph_NAME = 'edgetpu.tflite'

        # Path to .tflite file, which contains the model that is used for object detection
        self.Checkpoint_PATH = os.path.join(self.CWD_PATH, self.Model_NAME, self.Graph_NAME)

        # Path to label map file
        self.Labels_PATH = os.path.join(self.CWD_PATH, self.Model_NAME, self.Graph_NAME)

        # Load the label map
        with open(self.Labels_PATH, 'r') as f:
            self.LabelsTF = [line.strip() for line in f.readlines()]

        # Have to do a weird fix for label map if using the COCO "starter model" from
        # https://www.tensorflow.org/lite/models/object_detection/overview
        # First label is '???', which has to be removed.
        if self.LabelsTF[0] == '???':
            del (self.LabelsTF[0])

        # Load the Tensorflow Lite model
        # If using Edge TPU, use special load_delegate argument
        if edgetpu:
            self.interpreter = Interpreter(model_path=self.Checkpoint_PATH,
                                           experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
            print(self.Checkpoint_PATH)
        else:
            self.interpreter = Interpreter(model_path=self.Checkpoint_PATH)

        self.interpreter.allocate_tensors()

        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        self.ObjectVector = [0.0, 0.0]

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

        # Initialize frame rate calculation
        self.frame_rate_calc = 1
        self.freq = cv2.getTickFrequency()

        # Initialize video stream
        self.videostream = VideoStream(resolution=(self.imageWidth, self.imageHeight)).start()
        time.sleep(1)

    # ************************************************************************************************* Processes a
    #
    # *************************************************************************************************
    def process_image(self, searchingfor=None):
        self.Target_NAME = searchingfor
        self.OffCenterX = 0
        self.OffCenterY = 0
        self.LateralDistanceMM = 0.0
        self.DistanceMM = 0.0
        self.FoundTarget = False
        input_mean = 127.5
        input_std = 127.5
        # Start timer (for calculating frame rate)
        self.t1 = cv2.getTickCount()

        # Grab frame from video stream
        frame1 = self.videostream.read()

        # Acquire frame and resize to expected shape [1xHxWx3]
        frame = frame1.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (self.imageWidth, self.imageHeight))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Retrieve detection results
        # Bounding box coordinates of detected objects
        self.Boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[
            0]
        # Class index of detected objects
        self.Classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        # Confidence of detected objects
        self.Scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
        # Total number of detected objects (inaccurate and not needed)
        self.NumDetected = self.interpreter.get_tensor(self.output_details[3]['index'])[0]

        if self.ShowImages:
            self.draw_detected_frame(frame)

        # checking for specific target
        if self.Target_NAME is not None:
            self.process_distance()
        else:
            self.LateralDistanceMM = 0
            self.DistanceMM = 0
            self.OffCenterX = 0
            self.OffCenterY = 0
            self.FoundTarget = False

    def getLatDistanceMM(self):
        return self.LateralDistanceMM

    def getObjectVector(self):
        return self.ObjectVector

    def draw_detected_frame(self, frame):
        # Loop over all detections and draw detection box if confidence is above minimum threshold
        cv2.namedWindow('Object Detector')

        for i in range(len(self.Scores)):
            if (self.Scores[i] > self.min_conf_threshold) and (self.Scores[i] <= 1.0):
                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image
                # dimensions, need to force them to be within image using max() and min()
                self.MinY = int(max(1, (self.Boxes[i][0] * self.imageHeight)))
                self.MinX = int(max(1, (self.Boxes[i][1] * self.imageWidth)))
                self.MaxY = int(min(self.imageHeight, (self.Boxes[i][2] * self.imageHeight)))
                self.MaxX = int(min(self.imageWidth, (self.Boxes[i][3] * self.imageWidth)))

                cv2.rectangle(frame, (self.MinX, self.MinY), (self.MaxX, self.MaxY), (10, 255, 0), 2)
                # Look up object name from "labels" array using class index
                object_name = self.LabelsTF[int(self.Classes[i])]
                # drawing distance between searchingobject
                # Draw label
                label = '%s: %d%%' % (object_name, int(self.Scores[i] * 100))  # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
                label_ymin = max(self.MinY, labelSize[1] + 10)  # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (self.MinX, label_ymin - labelSize[1] - 10),
                              (self.MinX + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255),
                              cv2.FILLED)  # Draw white box to put label text in
                cv2.putText(frame, label, (self.MinX, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0),
                            2)  # Draw label text

        # Draw frame rate in corner of frame
        cv2.putText(frame, 'FPS: {0:.2f}'.format(self.frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA)

        # All the results have been drawn on the frame, so it's time to display it.
        cv2.imshow('Object Detector', frame)
        # NOT IN A LOOP ANYMORE, JUST SHOW IMAGES AND LET USER CALL FINALIZE TO CLOSE
        # --# Press any key to continue to next image, or press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            self.finalize()
        # Calculate framerate
        self.t2 = cv2.getTickCount()
        self.time1 = (self.t2 - self.t1) / self.freq
        self.frame_rate_calc = 1 / self.time1

    def process_distance(self):
        self.OffCenterX = 0
        self.OffCenterY = 0
        self.LateralDistance = 0.0
        self.DistanceMM = 0.0
        self.FoundTarget = False
        for i in range(len(self.Scores)):
            if (self.Scores[i] > self.min_conf_threshold) and (self.Scores[i] <= 1.0):
                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image
                # dimensions, need to force them to be within image using max() and min()
                self.MinY = int(max(1, (self.Boxes[i][0] * self.imageHeight)))
                self.MinX = int(max(1, (self.Boxes[i][1] * self.imageWidth)))
                self.MaxY = int(min(self.imageHeight, (self.Boxes[i][2] * self.imageHeight)))
                self.MaxX = int(min(self.imageWidth, (self.Boxes[i][3] * self.imageWidth)))
                ClassIndex = int(self.Classes[i])
                object_name = self.LabelsTF[ClassIndex]  # Look up object name from "labels" array using class index
                if ClassIndex > 3:
                    self.TargetYSize = 800
                else:
                    self.TargetYSize = 200
                if object_name == self.Target_NAME:
                    FoundTarget = True
                    self.distance3()

    def distance3(self):
        THEODORS_NUMBER = 0.0516657316
        SizeX = (self.MaxX - self.MinX)
        SizeY = (self.MaxY - self.MinY)
        self.OffCenterX = int(self.MaxX - (SizeX / 2)) - int(self.imageWidth / 2)
        self.OffCenterY = int(self.MaxY - (SizeY / 2)) - int(self.imageHeight / 2)
        # lateral distance of camera from object
        self.LateralDistance = (self.TargetYSize * self.Focal_LENGTH) / SizeY
        self.ObjectVector[0] = math.atan(self.OffCenterY/self.LateralDistance) * (180/math.pi)
        self.ObjectVector[1] = math.atan(self.OffCenterX/self.LateralDistance) * (180/math.pi)
        # to mm
        self.LateralDistance = (self.LateralDistance / THEODORS_NUMBER) * 10

        # a side of mm travel laterally triangle
        TargetPX = SizeY / 2
        TargetMM = self.TargetYSize
        # ratio of pixel to mm
        MM__PX = TargetPX / TargetMM
        # b side of mm travel laterally triangle
        Bpx = math.sqrt(pow(self.OffCenterX, 2) + pow(self.OffCenterY, 2))
        Bmm = Bpx * MM__PX
        # c side of mm travel laterally triangle
        # true exact distance of camera from object, no matter
        # where it is on the plane
        self.DistanceMM = math.sqrt(pow(Bmm, 2) + pow(self.LateralDistance, 2))
        self.OffCenterXMM = self.OffCenterX * MM__PX
        self.OffCenterYMM = self.OffCenterY * MM__PX

    def finalize(self):
        # Clean up
        cv2.destroyAllWindows()
        self.videostream.stop()
        sys.exit()

# *************************************************************************************************
