#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

csvFilename = "csvs/longmpc_friction0.8_0.5_angular1.csv"

class SIM_TEST:
    def __init__(self, csv_file_name):
        rospy.init_node("sim_test")
        self.time = -1
        rospy.Subscriber("/cmd_vel", Twist, self.record_vel)
        rospy.Subscriber("/odom", Odometry, self.record_odom)
        self.csv_file_name = csv_file_name
        with open(self.csv_file_name, 'w') as fd:
            pass
        rospy.spin()
    
    def record_vel(self, twist):
        if self.time != -1:
            lin_vel = twist.linear.x
            ang_vel = twist.angular.z
            with open(self.csv_file_name,'a') as fd:
                spamwriter = csv.writer(fd)
                row = [self.time, self.x, self.y, self.theta, lin_vel, ang_vel]
                spamwriter.writerow(row)
    
    def record_odom(self, odom):
        self.time = odom.header.stamp
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        quat = odom.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])



if __name__ == "__main__":
    csv_file_name = csvFilename
    sim_test = SIM_TEST(csv_file_name)
