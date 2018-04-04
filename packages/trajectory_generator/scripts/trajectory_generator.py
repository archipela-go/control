#!/usr/bin/env python

import math
import rospy
import tf
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class trajectory_generator(object):
    def __init__(self):
        rospy.init_node('trajectory_generator')
        self.waypoint_sub = rospy.Subscriber("/traj/waypoints", Path, self.waypoint_cb)

        self.positions = []

    def waypoint_cb(self, data):
        n_points = len(data.poses)

        print "Recieved: " + str(n_points) + " waypoints."

        positions = np.empty([n_points, 2])

        i = 0
        for pose in data.poses:
            positions[i, :] = [pose.pose.position.x, pose.pose.position.y]
            i += 1

        print "\n Waypoints:"
        print positions

        if not np.array_equal(self.positions, positions):
            self.positions = positions

if __name__ == '__main__':
    node = trajectory_generator()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")
