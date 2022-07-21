#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

def canny(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur_image = cv2.GaussianBlur(gray_image, (7, 7), 0)
    canny_image = cv2.Canny(blur_image, 50, 150)
    return canny_image

def display_lines(image, Hlines):
    line_image = np.zeros_like(image)
    if Hlines is not None:
        for Hline in Hlines:
            x1, y1, x2, y2 = Hline.reshape(4)
            cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
    return line_image

def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = 0
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return (np.array([x1, y1, x2, y2]))

def average_slope_intercept(image, lines):
    line_fit = []
    if np.any(lines) == None:
        pass
    else:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            intercept = parameters[1]
            line_fit.append((slope, intercept))
        line_fit_average = np.average(line_fit, axis=0)
        line = make_coordinates(image, line_fit_average)
        return np.array([line])

def make_point(Hlines):
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
        return (x_point, y_point)

def display_point(image, point):
    point_image = np.zeros_like(image)
    if point is not None:
        cv2.circle(point_image, point, 2, (0, 0, 255), 5)
    return point_image

def find_contours(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    contours = cv2.findContours(gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0]
    return contours

def check_area(Hlines, contours):
    points = []
    if np.any(Hlines) == None:
        pass
    else:
        for Hline in Hlines:
            x1, y1, x2, y2 = Hline.reshape(4)
            line_points = [(x1, y1), (x2, y2)]
            points.append(line_points)
        for c in contours:
            for p in points:
                p1 = (int(p[0][0]), int(p[0][1]))
                p2 = (int(p[1][0]), int(p[1][1]))
                result1 = cv2.pointPolygonTest(c, p1, False)
                result2 = cv2.pointPolygonTest(c, p2, False)
                if (result1 == -1) or (result2 == -1):
                    return -1
                else:
                    pass
        return 1

def draw_area(image, flag=0):
    height = image.shape[0]
    width = image.shape[1]
    p1 = (int(width/2-120), 0) #TUTAJ SZEROKOSC PROSTOKATA
    p2 = (int(width/2+120), height)
    rec_image = np.zeros_like(image)
    if flag == 1:
        cv2.rectangle(rec_image, p1, p2, (0, 255, 0), 3)
    else:
        cv2.rectangle(rec_image, p1, p2, (0, 0, 255), 3)
    return rec_image

def publish_msg(command):
    msg = String()
    if command == None:
        pass
    else:
        msg.data = command
        print(msg.data)
        area_pub.publish(msg)

def control_logic(image, area_flag, point):
    if point == None:
        publish_msg('STOP')
    else:
        width = image.shape[1]
        if area_flag == -1:
            if(point[0] > width/2): #jesli jest po prawej
                publish_msg('P')
            elif(point[0] < width/2): #jesli jest po lewej
                publish_msg('L')
        else:
            publish_msg('GO')
        
def image_processing(image):
    try:
        brige = CvBridge()
        img = brige.imgmsg_to_cv2(image, "bgr8")

        global run_once
        global cnts
        if run_once == 0:
            rec_image = draw_area(img)
            cnts = find_contours(rec_image)
            run_once = 1

        canny_img = canny(img)
        lines = cv2.HoughLinesP(
            canny_img, 
            1, 
            np.pi/180, 
            55, 
            np. array([]), 
            minLineLength=150,  
            maxLineGap=700
            )
        #avg_line = average_slope_intercept(img, lines)
        line_image = display_lines(img, lines)
        point = make_point(lines)
        point_image = display_point(img, point)
        area_flag = check_area(lines, cnts)
        rec_image = draw_area(img, area_flag)
        combo_image = cv2.addWeighted(img, 0.8, line_image, 1, 1)
        combo_image2 = cv2.addWeighted(combo_image, 0.8, point_image, 1, 1)
        end_image = cv2.addWeighted(combo_image2, 0.8, rec_image, 1, 1)
        control_logic(end_image, area_flag, point)
        return end_image
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

area_pub = rospy.Publisher('/area', String, queue_size=10)
run_once = 0

if __name__ == "__main__":
    main()
else:
    pass