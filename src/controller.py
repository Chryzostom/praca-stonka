#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def publish_msg(value):
    msg = Twist()
    if value == None:
        pass
    else:
        msg.data = value
        print(msg.data)
        vel_pub.publish(msg)
    
def callback(msg):
    move = Twist()
    if msg.data == 'GO':
        print('Do przodu')
        move.linear.x = 0.1
        move.angular.z = 0
    elif msg.data == 'L':
        print('W lewo')
        move.linear.x = 0.04
        move.angular.z = 0.08
    elif msg.data == 'P':
        print('W prawo')
        move.linear.x = 0.04
        move.angular.z = -0.08
    else:
        print('Stop')
        move.linear.x = 0
        move.angular.z = 0
    vel_pub.publish(move)

def main():
    rospy.init_node('controller')
    area_sub = rospy.Subscriber('/area', String, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

if __name__ == "__main__":
    main()
else:
    pass