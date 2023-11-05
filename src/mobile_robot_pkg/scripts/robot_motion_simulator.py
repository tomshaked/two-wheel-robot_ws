#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class RobotMotionSimulator:
    def __init__(self):
        rospy.init_node('robot_motion_simulator')

        # Create a publisher to send velocity commands to the robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Simulate robot motion
            linear_vel = 0.2  # m/s
            angular_vel = 0.1  # rad/s

            # Publish velocity commands
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        motion_simulator = RobotMotionSimulator()
        motion_simulator.run()
    except rospy.ROSInterruptException:
        pass
