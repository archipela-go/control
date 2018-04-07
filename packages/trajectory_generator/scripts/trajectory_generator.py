#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
from std_srvs.srv import Trigger

class trajectory_generator(object):
    def __init__(self):
        rospy.init_node('trajectory_generator')

        self.distance = 2.0 #m
        self.a_max = 0.5 # m / s^2
        self.v_max = 1.0 # m / s

        self.acc_time = self.v_max / self.a_max
        self.acc_dist = 0.5 * self.a_max * self.acc_time**2;

        if self.acc_dist >= self.distance/2.0:
            self.T = self.acc_time * 2.0
        else:
            self.T = self.acc_time * 2.0 + (self.distance - 2*self.acc_dist)/self.v_max

        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        self.imu_set = False
        self.imu_q = None

        self.pos_sub = rospy.Subscriber("/mavros/global_position/local", PoseWithCovarianceStamped, self.pos_cb)
        self.pos_set = False
        self.gps_pos = None
        self.gps_q = None

        self.gen_traj_srv = rospy.Service("generate", Trigger, self.gen_cb)
        self.generated = False
        self.coeffs_x = None
        self.coeffs_y = None

        self.execute_srv = rospy.Service("execute", Trigger, self.execute_cb)
        self.executing = False
        self.t0 = 0.0

        self.pos_pub = rospy.Publisher("position_cmd", PoseStamped, queue_size = 10)
        self.path_pub = rospy.Publisher("path", Path, queue_size=10)
    def imu_cb(self, data):
        q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        euler = euler_from_quaternion(q)
        yaw_q = quaternion_from_euler(0.0, 0.0, euler[2])

        self.imu_set = True
        self.imu_q = yaw_q

        if self.executing and self.imu_set and self.pos_set and self.generated:
            t = (rospy.Time.now() - self.t0).to_sec()
            print "Executing trajectory...t = " + str(t)
            if t >= self.T:
                t = self.T

            x_des = self.coeffs_x[0] + self.coeffs_x[1]*t + self.coeffs_x[2]*t**2 + self.coeffs_x[3]*t**3
            y_des = self.coeffs_y[0] + self.coeffs_y[1]*t + self.coeffs_y[2]*t**2 + self.coeffs_y[3]*t**3

            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = x_des
            msg.pose.position.y = y_des
            self.pos_pub.publish(msg)

    def pos_cb(self, data):
        q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        euler = euler_from_quaternion(q)
        yaw_q = quaternion_from_euler(0.0, 0.0, euler[2])

        self.gps_q = yaw_q

        pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        self.gps_pos = pos
        self.pos_set = True

    def gen_cb(self, data):
        if self.imu_set and self.pos_set:
            initial_vector = np.array([[self.distance], [0], [0], [1]])
            rotated_vector = np.matmul(quaternion_matrix(self.imu_q), initial_vector)

            start = np.array(self.gps_pos[0:-1])
            destination = rotated_vector[0:-2]

            A_x = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, self.T, self.T**2, self.T**3], [0, 1, 2*self.T, 3*self.T**2]])
            b_x = np.array([start[0], 0.0, destination[0], 0.0])

            coeffs_x = np.linalg.solve(A_x, b_x)

            A_y = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, self.T, self.T**2, self.T**3], [0, 1, 2*self.T, 3*self.T**2]])
            b_y = np.array([start[1], 0.0, destination[1], 0.0])

            coeffs_y = np.linalg.solve(A_y, b_y)

            self.coeffs_x = coeffs_x
            self.coeffs_y = coeffs_y

            myPath = Path()
            myPath.header.frame_id = 'map'
            pose_list = []
            for i in xrange(50):
                t = self.T * i / 50.0

                x_des = self.coeffs_x[0] + self.coeffs_x[1]*t + self.coeffs_x[2]*t**2 + self.coeffs_x[3]*t**3
                y_des = self.coeffs_y[0] + self.coeffs_y[1]*t + self.coeffs_y[2]*t**2 + self.coeffs_y[3]*t**3

                loc = PoseStamped()
                loc.header.frame_id = 'map'
                loc.pose.position.x = x_des
                loc.pose.position.y = y_des

                pose_list.append(loc)

            myPath.poses = pose_list
            self.path_pub.publish(myPath)


            print "T: " + str(self.T)
            print "coeffs_x: " + str(coeffs_x)
            print "coeffs_y: " + str(coeffs_y)

            self.generated = True

    def execute_cb(self, data):
        if self.executing:
            self.executing = False
        else:
            self.executing = True

        self.t0 = rospy.Time.now()


if __name__ == '__main__':
    node = trajectory_generator()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")
