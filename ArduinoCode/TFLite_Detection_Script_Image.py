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
# We can use the C++ Python API to embed a python interpreter into C++ which will allow us to 
#      get the return value from a specific python function, and therefore run inference on single images using this script
#
# I refactored the code to be completely wrapped in functions so that we can call image processing on demand 
#       to evaluate an image and get a value returned to C++
#



# Import packages *********************************************************************************
import os
import argparse
import cv2
import numpy as np
import sys
import glob
import importlib.util


# *************************************************************************************************

# *************************************************************************************************
# Initialize environment variables and set things up for single images to be able to be processed.
# Returns: None
# Thread Safety: None
# *************************************************************************************************
def init(model_dir, show_images=False, graph='detect.tflite', labelmap_name='labelmap.txt', threshold=0.5, edgetpu=False):
    # globals
    global CWD_PATH
    global width
    global height
    global floating_model
    global interpreter
    global input_details
    global output_details
    global SHOW_IMAGES
    
    
    MODEL_NAME = model_dir
    SHOW_IMAGES = show_images
    GRAPH_NAME = graph
    LABELMAP_NAME = labelmap_name
    min_conf_threshold = float(threshold)
    use_TPU = edgetpu
    
    # Import TensorFlow libraries 
    # If tensorflow is not installed, import interpreter from tflite_runtime, else import from regular tensorflow
    # If using Coral Edge TPU, import the load_delegate library
    pkg = importlib.util.find_spec('tensorflow')
    if pkg is None:
        from tflite_runtime.interpreter import Interpreter
        if use_TPU:
            from tflite_runtime.interpreter import load_delegate
    else:
        from tensorflow.lite.python.interpreter import Interpreter
        if use_TPU:
            from tensorflow.lite.python.interpreter import load_delegate
    
    # If using Edge TPU, assign filename for Edge TPU model
    if use_TPU:
        # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
        if (GRAPH_NAME == 'detect.tflite'):
            GRAPH_NAME = 'edgetpu.tflite'

    # Get path to current working directory
    CWD_PATH = os.getcwd()

    # Path to .tflite file, which contains the model that is used for object detection
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

    # Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

    # Load the label map
    with open(PATH_TO_LABELS, 'r') as f:
        labels = [line.strip() for line in f.readlines()]

    # Have to do a weird fix for label map if using the COCO "starter model" from
    # https://www.tensorflow.org/lite/models/object_detection/overview
    # First label is '???', which has to be removed.
    if labels[0] == '???':
        del(labels[0])

    # Load the Tensorflow Lite model.
    # If using Edge TPU, use special load_delegate argument
    if use_TPU:
        interpreter = Interpreter(model_path=PATH_TO_CKPT,
                                  experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
        print(PATH_TO_CKPT)
    else:
        interpreter = Interpreter(model_path=PATH_TO_CKPT)

    interpreter.allocate_tensors()

    # Get model details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]

    floating_model = (input_details[0]['dtype'] == np.float32)

    input_mean = 127.5
    input_std = 127.5
# *************************************************************************************************

# *************************************************************************************************
# Processes a single image with the specified file name
# Returns: A tuple of 3 lists:
#                   + boundingBoxes     : list (python array) of arrays size 4. 
#                                           These size 4 arrays have top,bottom,left,right coordinates respectively 
#                                           (I think they are percent based, so 0 to 1)
#                   + classifications   : list (python array) which contains strings of what TensorFlow classified the object as
#                   + accuracies        : list (python array) which contains accuracies (out of 100.0%) of how "certain" the model thinks it is.
#                                           It has a percentage for each classification totaling to 100%, but only outputs its most accurate prediction
#                                           (Also, I believe it doesnt display anything if the prediction is below 50% accurate)
#
#                   Note: These three arguments should be exactly the same length, and correspond exactly to each other with index
# Thread Safety: None
# *************************************************************************************************
def process_image(image):
    # Parse input image name and directory. 
    IM_NAME = image
    IM_DIR = None # This can be specified to process a directory of images, but for now we will only do one at a time

    # If both an image AND a folder are specified, throw an error
    if (IM_NAME and IM_DIR):
        print('Error! Please only use the --image argument or the --imagedir argument, not both. Issue "python TFLite_detection_image.py -h" for help.')
        finalize()
        sys.exit()

    # If neither an image or a folder are specified, default to using 'test1.jpg' for image name
    if (not IM_NAME and not IM_DIR):
        IM_NAME = 'test1.jpg'

    # Define path to images and grab all image filenames
    if IM_DIR:
        PATH_TO_IMAGES = os.path.join(CWD_PATH,IM_DIR)
        images = glob.glob(PATH_TO_IMAGES + '/*')

    elif IM_NAME:
        PATH_TO_IMAGES = os.path.join(CWD_PATH,IM_NAME)
        images = glob.glob(PATH_TO_IMAGES)

    # NO LONGER LOOP SINCE SINGLE IMAGE
    #--# Loop over every image and perform detection
    #--for image_path in images:
    image_path = images[0]

    if image_path is not None:
        # Load image and resize to expected shape [1xHxWx3]
        image = cv2.imread(image_path)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        imH, imW, _ = image.shape 
        image_resized = cv2.resize(image_rgb, (width, height))
        input_data = np.expand_dims(image_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()

        # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[0]['index'])[0] # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[1]['index'])[0] # Class index of detected objects
        scores = interpreter.get_tensor(output_details[2]['index'])[0] # Confidence of detected objects
        #num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)

        if SHOW_IMAGES:
            draw_detected_image(image, imH, imW, boxes, classes, scores)

        return boxes, classes, scores

    # else
    print('Error! No image found...')
    finalize()
    sys.exit()
# *************************************************************************************************

# *************************************************************************************************
# This function uses the OpenCV library to show the image specified with its corresponding bounding
#   boxes, classes, and scores shown on the image.
# This takes in the image, image height, image width, bounding box coordinate list, class list, and
#   score list.
# Returns: None
# Thread Safety: None
# *************************************************************************************************
def draw_detected_image(image, imH, imW, boxes, classes, scores):
    # Loop over all detections and draw detection box if confidence is above minimum threshold
    for i in range(len(scores)):
        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1,(boxes[i][0] * imH)))
            xmin = int(max(1,(boxes[i][1] * imW)))
            ymax = int(min(imH,(boxes[i][2] * imH)))
            xmax = int(min(imW,(boxes[i][3] * imW)))
            
            cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

            # Draw label
            object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
            label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(image, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
            cv2.putText(image, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

    # All the results have been drawn on the image, now display the image
    cv2.imshow('Object detector', image)

    # NOT IN A LOOP ANYMORE, JUST SHOW IMAGES AND LET USER CALL FINALIZE TO CLOSE
    #--# Press any key to continue to next image, or press 'q' to quit
    #--if cv2.waitKey(0) == ord('q'):
    #--    break


def finalize():
    # Clean up
    cv2.destroyAllWindows()
