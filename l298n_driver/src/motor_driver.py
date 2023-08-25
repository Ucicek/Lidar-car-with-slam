#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class MotorController:
    def __init__(self):
        self.pub_left = rospy.Publisher('left_motor', Int16, queue_size=10)
        self.pub_right = rospy.Publisher('right_motor', Int16, queue_size=10)
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.callback)

    def callback(self, data):
        linear_speed = data.linear.x
        angular_speed = data.angular.z

        # convert linear_speed and angular_speed to left_motor_speed and right_motor_speed
        # your conversion logic here, which depends on your robot's design

        left_motor_speed = Int16()
        right_motor_speed = Int16()

        # assign values to left_motor_speed and right_motor_speed here based on your logic

        self.pub_left.publish(left_motor_speed)
        self.pub_right.publish(right_motor_speed)

if __name__ == '__main__':
    rospy.init_node('l298n_driver', anonymous=True)
    MotorController()
    rospy.spin()

