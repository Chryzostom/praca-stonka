#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import math as m
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from vision_crop.srv import start, startResponse

flag = 0 #flaga dotyczaca startu programu
sim = 1 #jesli 0 to kamera, jesli 1 to symulacja 

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

            canny_img = self.canny(img)
            lines = cv2.HoughLinesP(
                canny_img, 
                1, 
                np.pi/180, 
                60, 
                np. array([]), 
                minLineLength=150,  
                maxLineGap=700
                )
            self.display_lines(img, lines)
            dest_point = self.make_point(img, lines)
            rob_point = self.robot_point(img)
            self.controller(rob_point, dest_point)
            self.draw_line(img, rob_point, dest_point)
            return img
        except CvBridgeError as e:
            print(e)

    def canny(self, image):
        kernel = (7, 7)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur_image = cv2.GaussianBlur(gray_image, kernel, 0)
        canny_image = cv2.Canny(blur_image, 50, 150)
        return canny_image

    def display_lines(self, image, Hlines):
        if Hlines is not None:
            for Hline in Hlines:
                x1, y1, x2, y2 = Hline.reshape(4)
                cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 5)

    def make_point(self, image, Hlines):
        x_val = []
        y_val = []
        if np.any(Hlines) == None:
            pass
        else:
            for Hline in Hlines:
                x1, y1, x2, y2 = Hline.reshape(4)
                x_val.append((x1, x2))
                y_val.append((y1, y2))
            x_avg = np.average(x_val, axis=0)
            y_avg = np.average(y_val, axis=0)
            x_point = int((x_avg[0] + x_avg[1])/2)
            y_point = int((y_avg[0] + y_avg[1])/2)
            point = (x_point, y_point)
            cv2.circle(image, point, 2, (0, 0, 255), 5)
            return point

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
            cv2.line(image, current_pos, destination, (255, 0, 255), 2)

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
                vel_angular = (-0.03) * (fi - yaw)
            else:
                vel_linear = 0.0
                vel_angular = 0.0
        self.publish(vel_linear, vel_angular)
        print("predkosc liniowa: {}, predkosc katowa: {}".format(vel_linear, vel_angular))

def handle_start(req):
    global flag
    res = startResponse()
    flag = req.start
    return res

def main(args):
    rospy.init_node('planner')
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