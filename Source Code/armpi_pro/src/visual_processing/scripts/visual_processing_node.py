#!/usr/bin/python3
# coding=utf8
# Date:2022/05/30
import sys
import cv2
import time
import math
import rospy
import numpy as np
from armpi_pro import Misc
from armpi_pro import apriltag
from threading import RLock,Timer
from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image
from visual_processing.msg import Result
from visual_processing.srv import SetParam


lock = RLock()

image_sub = None
publish_en = True
size_s = (160, 120)
size_m = (320, 240)
__isRunning = False
target_type = 'None'
target_color = 'None'
color_range_list = None
pub_time = time.time()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


# Threshold
conf_threshold = 0.6
# Mdodel position
configFile = "/home/ubuntu/armpi_pro/src/face_detect/scripts/models/deploy.prototxt"
modelFile = "/home/ubuntu/armpi_pro/src/face_detect/scripts/models/res10_300x300_ssd_iter_140000_fp16.caffemodel"
net = cv2.dnn.readNetFromCaffe(configFile, modelFile)


# Find the biggest contour
# the parameter is the list of coutours to be compared
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # loop all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:  # Only when the area is greater than 300, the biggest contour can take effect to filter the interference 
                area_max_contour = c
    return area_max_contour, contour_area_max  # return the biggest contour

# face recognition function
def face_detect(img):
    global pub_time
    global publish_en
    
    msg = Result()
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    blob = cv2.dnn.blobFromImage(img_copy, 1, (140, 140), [104, 117, 123], False, False)
    net.setInput(blob)
    detections = net.forward() #calculation recognition
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold:
            #convert the coordinates of the recognized face into the coordnates before scaling.
            x1 = int(detections[0, 0, i, 3] * img_w)
            y1 = int(detections[0, 0, i, 4] * img_h)
            x2 = int(detections[0, 0, i, 5] * img_w)
            y2 = int(detections[0, 0, i, 6] * img_h)             
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2) #frame the recognized face.
            
            msg.center_x = int((x1 + x2)/2)
            msg.center_y = int((y1 + y2)/2)
            msg.data = round(confidence, 2)
            publish_en = True
        
        if publish_en:
            if (time.time()-pub_time) >= 0.06:
                result_pub.publish(msg)  # publish result
                pub_time = time.time()
                
            if msg.data == 0:
                publish_en = False
                result_pub.publish(msg) 
                
    return img


# detect Apriltag fucntion
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltag_Detect(img):
    global pub_time
    global publish_en
    
    msg = Result()
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size_m, interpolation=cv2.INTER_NEAREST)
    gray = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)
    
    if len(detections) != 0:
        for i, detection in enumerate(detections):              
            corners = np.rint(detection.corners)  # get four corners
            for i in range(4):
                corners[i][0] = int(Misc.map(corners[i][0], 0, size_m[0], 0, img_w))
                corners[i][1] = int(Misc.map(corners[i][1], 0, size_m[1], 0, img_h))
                
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)
            tag_family = str(detection.tag_family, encoding='utf-8')  #  get tag_family
            tag_id = int(detection.tag_id)        # get tag_id
            
            object_center_x = int(Misc.map(detection.center[0], 0, size_m[0], 0, img_w))
            object_center_y = int(Misc.map(detection.center[1], 0, size_m[1], 0, img_h))  # centre point            
            object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # calculate the rotation angle
            cv2.putText(img, str(tag_id), (object_center_x - 10, object_center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 255], 2)
            
        msg.center_x = object_center_x
        msg.center_y = object_center_y
        msg.angle = object_angle
        msg.data = tag_id
        publish_en = True 
    
    if publish_en:
        if (time.time()-pub_time) >= 0.06:
            result_pub.publish(msg)  # publish result
            pub_time = time.time()
            
        if msg.data == 0:
            publish_en = False
            result_pub.publish(msg)
                
    return img
    

# [ROI, weight]
roi = [(50, 70,  0, 160, 0.2),
       (80, 90,  0, 160, 0.3),
       (100, 115,  0, 160, 0.5)]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]
roi_h_list = [roi_h1, roi_h2, roi_h3]
# line dection function
def line_detect(img, color):
    global pub_time
    global publish_en
    global color_range_list
    
    if color == 'None':
        return img
    
    n = 0
    line_width = 0
    msg = Result()
    area_max = 0
    weight_sum = 0
    centroid_x_sum = 0
    area_max_contour = 0
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size_s, interpolation=cv2.INTER_NEAREST)
    #frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3) 
    #Divide the image into upper, middle and lower parts, so the processing speed will be faster and more accurate
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1       
        blobs = frame_resize[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # convert image into LAB space
        if color in color_range_list:
            color_range = color_range_list[color]
            frame_mask = cv2.inRange(frame_lab, tuple(color_range['min']), tuple(color_range['max']))  # Bitwise operation operates on the original image and mask.
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))  # erode
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))  # dilate
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find contour
            area_max_contour, area_max = getAreaMaxContour(contours)  # find the biggest contour
        if area_max > 50:  # there is the biggest contour
            rect = cv2.minAreaRect(area_max_contour)  # The minimum enclosing rectangle
            box = np.int0(cv2.boxPoints(rect))        # The four vertices of the smallest enclosing rectangle
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                box[i][0] = int(Misc.map(box[i][0], 0, size_s[0], 0, img_w))
                box[i][1] = int(Misc.map(box[i][1], 0, size_s[1], 0, img_h))
            cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  #draw a rectangle composed by four points
            #Get the opposite corners of a rectangle
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt2_x, pt2_y = box[1, 0], box[1, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            center_x = int((pt1_x + pt3_x) / 2) #center point
            center_y = int((pt1_y + pt3_y) / 2)
            line_width = int(abs(pt1_x - pt2_x))
            cv2.circle(img, (center_x, center_y), 5, (0,0,0), -1) # draw center point
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]
    if weight_sum is not 0:
        #get the final center point
        msg.data = line_width
        msg.center_y = center_y
        msg.center_x = int(centroid_x_sum / weight_sum)
        cv2.circle(img, (msg.center_x, msg.center_y), 10, (0,0,255), -1)# draw center point
        publish_en = True
    
    if publish_en:
        if (time.time()-pub_time) >= 0.06:
            result_pub.publish(msg)  # publish result
            pub_time = time.time()
            
        if msg.data == 0:
            publish_en = False
            result_pub.publish(msg)
    
    return img

# single oolor detection
def color_detect(img, color):
    global pub_time
    global publish_en
    global color_range_list
    
    if color == 'None':
        return img
    
    msg = Result()
    area_max = 0
    area_max_contour = 0
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size_m, interpolation=cv2.INTER_NEAREST)
    frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # convert image into LAB space

    if color in color_range_list:
        color_range = color_range_list[color]
        frame_mask = cv2.inRange(frame_lab, tuple(color_range['min']), tuple(color_range['max']))  # Bitwise operation operates on the original image and mask.
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))          # erode
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))            # dilate
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]         # find contour
        area_max_contour, area_max = getAreaMaxContour(contours)                                   # find the biggest contour

        if area_max > 200:  # find the biggest area
            (centerx, centery), radius = cv2.minEnclosingCircle(area_max_contour)  # Get the smallest circumcircle
            msg.center_x = int(Misc.map(centerx, 0, size_m[0], 0, img_w))
            msg.center_y = int(Misc.map(centery, 0, size_m[1], 0, img_h))
            msg.data = int(Misc.map(radius, 0, size_m[0], 0, img_w))
            cv2.circle(img, (msg.center_x, msg.center_y), msg.data+5, range_rgb[color], 2)
            publish_en = True
        
        if publish_en:
            if (time.time()-pub_time) >= 0.06:
                result_pub.publish(msg)  # publish result 
                pub_time = time.time()
                
            if msg.data == 0:
                publish_en = False
                result_pub.publish(msg)
        
    return img

# multi-color detection
def colors_detect(img, color_list):
    global pub_time
    global publish_en
    global color_range_list
    
    if color_list == 'RGB' or color_list == 'rgb':
        color_list = ('red','green','blue')
    else:
        return img
    
    msg = Result()
    msg.data = 0
    color_num = 0
    max_area = 0
    color_area_max = None
    areaMaxContour_max = 0
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    frame_resize = cv2.resize(img_copy, size_m, interpolation=cv2.INTER_NEAREST)
    frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # convert image into LAB space
    
    for color in color_list:
        if color in color_range_list:
            color_range = color_range_list[color]
            frame_mask = cv2.inRange(frame_lab, tuple(color_range['min']), tuple(color_range['max']))  # Bitwise operation operates on the original image and mask.
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))          # erode
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))            # dilate
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]         # find contour
            areaMaxContour, area_max = getAreaMaxContour(contours)                                   # find the biggest contour
            if areaMaxContour is not None:
                if area_max > max_area:#find the biggest area
                    max_area = area_max
                    color_area_max = color
                    areaMaxContour_max = areaMaxContour

    if max_area > 200:  # there is the biggest area
        (centerx, centery), radius = cv2.minEnclosingCircle(areaMaxContour_max)  # Get the smallest circumcircle
        msg.center_x = int(Misc.map(centerx, 0, size_m[0], 0, img_w))
        msg.center_y = int(Misc.map(centery, 0, size_m[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size_m[0], 0, img_w))
        rect = cv2.minAreaRect(areaMaxContour_max)   #Minimum circumscribed rectangle
        box = np.int0(cv2.boxPoints(rect))           #The four vertices of the smallest circumscribed rectangle
        msg.angle = int(math.degrees(math.atan2(box[1][1] - box[0][1], box[1][0] - box[0][0])))
        
        cv2.circle(img, (msg.center_x, msg.center_y), radius+5, range_rgb[color_area_max], 2)
        
        if color_area_max == 'red':  #the maximum color area is red 
            msg.data = 1
        elif color_area_max == 'green':  #the maximum color area is green
            msg.data = 2
        elif color_area_max == 'blue':  #the maximum color area is blue
            msg.data = 3
            
        publish_en = True
            
    if publish_en:
        if (time.time()-pub_time) >= 0.06:
            result_pub.publish(msg)  # publish result
            pub_time = time.time()
            
        if msg.data == 0:
            publish_en = False
            result_pub.publish(msg)
        
    return img

# camera image node callback function
def image_callback(ros_image):
    global lock
    global target_type 
    global target_color
    
    # convert the customization image meassage into image
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame_result = cv2_img
    with lock:
        if __isRunning:
            # switch the image processing mode
            if target_type == 'face':
                frame_result = face_detect(cv2_img)
            elif target_type == 'apriltag': 
                frame_result = apriltag_Detect(cv2_img)
            elif target_type == 'line':
                frame_result = line_detect(cv2_img,target_color)
            elif target_type == 'color': 
                frame_result = color_detect(cv2_img,target_color)
            elif target_type == 'colors': 
                frame_result = colors_detect(cv2_img,target_color)
            
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


# initialization call
def init():
    global publish_en
    global target_color
    global color_range_list
    
    publish_en = True
    target_color = 'None'
    rospy.loginfo("visual processing Init")
    color_range_list = rospy.get_param('/lab_config_manager/color_range_list', {})  # get lab range from ros param server

image_sub_st = False
def enter_func(msg):
    global lock
    global image_sub
    global image_sub_st
    
    rospy.loginfo("enter visual processing")
    init()
    with lock:
        if not image_sub_st:
            image_sub_st = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
            
    return [True, 'enter']

def exit_func(msg):
    global lock
    global image_sub
    global image_sub_st
    global __isRunning
    global target_color
    
    rospy.loginfo("exit visual processing")
    with lock:
        __isRunning = False
        target_color = 'None'
        try:
            if image_sub_st:
                image_sub_st = False
                image_sub.unregister()
        except BaseException as e:
            rospy.loginfo('%s', e)
        
    return [True, 'exit']

def start_running():
    global lock
    global __isRunning
    
    rospy.loginfo("start running visual processing")
    with lock:
        __isRunning = True

def stop_running():
    global lock
    global __isRunning
    
    rospy.loginfo("stop running visual processing")
    with lock:
        __isRunning = False

def set_running(msg):
    global target_type 
    global target_color
    
    rospy.loginfo("%s", msg)
    init()
    if msg.type:
        target_type = msg.type
        target_color = msg.color
        start_running()
    else:
        target_color = 'None'
        stop_running()
        
    return [True, 'set_running']


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('visual_processing', log_level=rospy.DEBUG)
    # communication service
    image_pub = rospy.Publisher('/visual_processing/image_result', Image, queue_size=1)
    result_pub = rospy.Publisher('/visual_processing/result', Result, queue_size=1) 
    enter_srv = rospy.Service('/visual_processing/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/visual_processing/exit', Trigger, exit_func)
    running_srv = rospy.Service('/visual_processing/set_running', SetParam, set_running)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()
