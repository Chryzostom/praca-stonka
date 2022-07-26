#!/usr/bin/env python

from turtle import delay
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import math as m
from cv_bridge import CvBridge, CvBridgeError
import sys
from geometry_msgs.msg import Twist
from vision_crop.srv import start, startResponse
import time

flag = 0 #flaga dotyczaca startu programu
sim = 0 #jesli 0 to kamera, jesli 1 to symulacja 

class planner:
    def __init__(self):
        if sim == 0:
            self.img_sub = rospy.Subscriber("/cam/image_raw", Image, self.imgCallback)
        else:
            self.img_sub = rospy.Subscriber("/agribot/front_camera/image_raw", Image, self.imgCallback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def imgCallback(self, data):
        frame = self.image_processing(data)
        cv2.imshow('img-window', frame)
        cv2.waitKey(1)

    def image_processing(self, image):
        try:
            brige = CvBridge()
            img = brige.imgmsg_to_cv2(image, "bgr8")
            if sim == 0:
                left_right_image = np.split(img, 2, axis=1)
                img = left_right_image[0]
            obj_img = self.obj_image_hsv(img) #dla hsv
            #obj_img = self.obj_image_canny(img) #dla canny
            dest_point = self.destination_point(img, obj_img)
            robot_point = self.robot_point(img)
            self.controller(robot_point, dest_point)
            self.draw_line(img, robot_point, dest_point)
            return img
        except CvBridgeError as e:
            print(e)

    def obj_image_hsv(self, image):
        kernel = np.ones((4, 4), np.uint8)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv[..., 1] = hsv[..., 1]*2.4
        hsv[..., 2] = hsv[..., 2]*0.6
        l_b = np.array([12, 21, 64])
        u_b = np.array([84, 255, 255])
        obj_img = cv2.inRange(hsv, l_b, u_b)
        dilated_img = cv2.dilate(obj_img, kernel, iterations=3)
        return obj_img

    def obj_image_canny(self, image):
        kernel = np.ones((4, 4), np.uint8)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur_image = cv2.GaussianBlur(gray_image, (7, 7), 0)
        canny_image = cv2.Canny(blur_image, 50, 150)
        dilated_img = cv2.dilate(canny_image, kernel, iterations=3)
        return dilated_img

    def destination_point(self, image, objects):
        contours, _ = cv2.findContours(objects, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        points = []
        xi = image.shape[1]
        for contour in contours:
            (x, y, w, h) = cv2.boundingRect(contour)
            if (cv2.contourArea(contour) < 5000):# or (cv2.contourArea(contour) > 100000):
                continue
            else:
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                points.append((int(x+w/2), int(y+h/2)))
        if len(points) == 0:
            pass
        else:
            for point in reversed(points):
                if (point[0] < xi/2+150) and (point[0] > xi/2-150): #threshold na linie
                    last_point = point
                    break
                else:
                    last_point = points[-1]
                    continue
            cv2.circle(image, last_point, 2, (0, 0, 255), 5)
            return last_point

    def robot_point(self, image):
        y = image.shape[0]
        x = image.shape[1]
        point = (int(x/2), int(y/2)+300) #obnizenie punktu w dol
        cv2.circle(image, point, 2, (0, 255, 255), 5)
        return point
        
    def draw_line(self, image, current_pos, destination):
        if current_pos == None or destination == None:
            pass
        else:
            cv2.line(image, current_pos, destination, (255, 0, 0), 2)
    
    def publish(self, linear, angular):
        move = Twist()
        move.linear.x = linear
        move.angular.z = angular
        self.vel_pub.publish(move)

    def controller(self, current_pos, destination):
        if current_pos == None or destination == None or flag == 0:
            vel_linear = 0.0
            vel_angular = 0.0
        else:
            x_now = current_pos[0]
            y_now = current_pos[1]
            x_dest = destination[0]
            y_dest = destination[1]
            yaw = 270
            fi = cv2.fastAtan2(y_dest - y_now, x_dest - x_now)
            gamma = fi - yaw
            dist = m.sqrt(((x_dest - x_now)**2)+((y_dest - y_now)**2))
            if dist > 50:
                vel_linear = 0.001 * dist
                vel_angular = (-0.01) * (fi - yaw)
            else:
                vel_linear = 0.0
                vel_angular = 0.0
        self.publish(vel_linear, vel_angular)
        print("predkosc liniowa: {}, predkosc katowa: {}".format(vel_linear, vel_angular))
        #time.sleep(0.1)

def handle_start(req):
    global flag
    res = startResponse()
    flag = req.start
    return res

def main(args):
    rospy.init_node('planner')
    rate = rospy.Rate(10)
    service = rospy.Service('start', start, handle_start)
    p = planner()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
else:
    pass
