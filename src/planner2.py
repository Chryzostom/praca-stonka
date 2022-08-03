#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import math as m
from cv_bridge import CvBridge, CvBridgeError

def obj_image(image):
    kernel = np.ones((4, 4), np.uint8)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur_image = cv2.GaussianBlur(gray_image, (7, 7), 0)
    canny_image = cv2.Canny(blur_image, 50, 150)
    dilated_img = cv2.dilate(canny_image, kernel, iterations=3)
    return dilated_img

def destination_point(image, objects):
    contours, _ = cv2.findContours(objects, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    points = []
    for contour in contours:
        (x, y, w, h) = cv2.boundingRect(contour)
        if cv2.contourArea(contour) < 3000:
            continue
        else:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)
            points.append((int(x+w/2), int(y+h/2)))
    last_point = points[-1]
    cv2.circle(image, last_point, 2, (0, 0, 255), 5)
    return last_point

def central_point(image):
    y = image.shape[0]
    x = image.shape[1]
    point = (int(x/2), int(y/2))
    cv2.circle(image, point, 2, (0, 255, 0), 5)
    return point

def diff(image, current_pos, destination):
    x_now = current_pos[0]
    y_now = current_pos[1]
    x_dest = destination[0]
    y_dest = destination[1]
    yaw = 270
    fi = cv2.fastAtan2(y_dest - y_now, x_dest - x_now)
    dist = abs(m.sqrt(((x_dest - x_now)**2)+((y_dest - y_now)**2)))
    test_linear = 2 * dist
    test_angular = 3 * (fi - yaw)
    print(test_linear)

def image_processing(image):
    try:
        brige = CvBridge()
        img = brige.imgmsg_to_cv2(image, "bgr8")
        obj_img = obj_image(img)
        dest_point = destination_point(img, obj_img)
        cent_point = central_point(img)
        diff(img, cent_point, dest_point)
        return img
    except CvBridgeError as e:
        print(e)

def imgCallback(data):
    frame = image_processing(data)
    cv2.imshow('img-window', frame)
    cv2.waitKey(1)

def main():
    rospy.init_node('planner')
    img_sub = rospy.Subscriber("/agribot/front_camera/image_raw", Image, imgCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
else:
    pass