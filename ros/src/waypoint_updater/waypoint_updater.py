#!/usr/bin/env python

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.log_counter = 1

        self.last_velocity = 0.0
        rospy.Subscriber('/current_velocity',  TwistStamped, self.current_velocity_cb)

        # Subscribe to all the topics you need and initialize corresponding variables:
        self.last_pose = None
        self.last_waypoints = None
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.next_tl_idx = -1  # -1 means no known traffic light
        self.braking_for_tl_idx = -1  # stores the index of the traffic light we're currently braking for
        rospy.Subscriber('/traffic_waypoint',  Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.waypoints_2d = None
        self.waypoint_tree = None

        self.main_loop()

    def main_loop(self):
        rate = rospy.Rate(10)  # define a 10Hz timer, only dbw_node needs to run at 50Hz
        while not rospy.is_shutdown():
            # check that we received callbacks 'pose_cb' and 'waypoints_cb':
            if self.last_waypoints is not None and self.last_pose is not None:
                lane = Lane()
                lane.header.stamp = rospy.Time().now()
                lane.header.frame_id = '/world'

                # calculate next waypoint:
                next_wp = self.get_closest_waypoint(self.last_pose)
                # Get all following waypoints - this adjusts for traffic lights as well:
                lane.waypoints = self.get_next_waypoints(self.last_waypoints, next_wp, next_wp + LOOKAHEAD_WPS)

                self.final_waypoints_pub.publish(lane)

            rate.sleep()

    def pose_cb(self, msg):
        self.last_pose = msg

    def waypoints_cb(self, msg):
        # Note: according to the classroom this will be published once only!! (always use a copy of waypoints!)
        self.last_waypoints = msg.waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in msg.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.next_tl_idx = msg.data
        if self.next_tl_idx == -1:
            self.braking_for_tl_idx = self.next_tl_idx

    def current_velocity_cb(self, msg):
        self.last_velocity = msg.twist.linear.x  # in m/s

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, pose):
        # get closest waypoint:
        x = pose.pose.position.x
        y = pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        # Calculate projection of vector (closest_wp->current position) onto vector (prev_wp->closest_wp)
        # If projection is positive it means that that we passed closest_wp already!
        val = np.dot(cl_vect - prev_vect, pos_vect-cl_vect)
        if (val > 0):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def get_next_waypoints(self, waypoints, start_wp, end_wp):

        next_waypoints = waypoints[start_wp:end_wp]

        tl_wp = self.next_tl_idx
        if tl_wp >= 0 and tl_wp <= end_wp and tl_wp >= start_wp:
            self.log_counter = self.log_counter + 1

            temp = []
            for i, wp in enumerate(next_waypoints):
                p = Waypoint()
                p.pose = wp.pose
                stop_idx = max(tl_wp - start_wp - 2, 0)
                dist = self.distance(next_waypoints, i, stop_idx)
                vel = 1.0 * math.sqrt(1.0 * dist) - 0.0
                if vel < 1:
                    vel = 0

                self.set_waypoint_velocity(p, min(vel, self.get_waypoint_velocity(wp)))
                temp.append(p)
            next_waypoints = temp

        return next_waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
