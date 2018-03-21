#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint',  Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.last_pose = None
        self.last_waypoints = None

        rate = rospy.Rate(10)  # define a 10Hz timer
        while not rospy.is_shutdown():
            self.main_loop()  # execute main loop
            rate.sleep()      # waiting till the next time-slot

    def main_loop(self):
        # check that we received callbacks 'pose_cb' and 'waypoints_cb':
        if self.last_waypoints is not None and self.last_pose is not None:
            lane = Lane()
            lane.header.stamp = rospy.Time().now()
            lane.header.frame_id = '/world'

            # calculate next and all following waypoints:
            waypoints = self.last_waypoints.waypoints
            next_wp = self.get_closest_waypoint(self.last_pose, waypoints)
            lane.waypoints = self.get_next_waypoints(waypoints, next_wp, next_wp + LOOKAHEAD_WPS)

            self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.last_pose = msg

    def waypoints_cb(self, msg):
        self.last_waypoints = msg

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def sanitize_wp_index(self, wp_idx):
        return wp_idx % len(self.last_waypoints.waypoints)

    def raw_dist(self, wp1, wp2):
        return math.sqrt((wp1.x - wp2.x)**2 + (wp1.y - wp2.y)**2 + (wp1.z - wp2.z)**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.raw_dist(waypoints[self.sanitize_wp_index(wp1)].pose.pose.position,
                                  waypoints[self.sanitize_wp_index(i)].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint(self, pose, waypoints):
        # get closest waypoint:
        curr_dist_to_wp = lambda wp: self.raw_dist(pose.pose.position, wp.pose.pose.position)
        closest_wp = min(waypoints, key=curr_dist_to_wp)
        closest_idx = waypoints.index(closest_wp)  # TODO: there should be a better way to do this!?!

        # check if closest_wp is behind us:
        next_wp_idx = self.sanitize_wp_index(closest_idx+1)
        dist_between_next_wps = self.distance(waypoints, closest_idx, closest_idx+1)  # don't sanitize here
        dist_to_next_wp = curr_dist_to_wp(waypoints[next_wp_idx])

        # TODO: Will need to make this check more robust when we're deviating from the path.
        #       e.g. current position is exactly perpendicular next to clostest_wp
        return next_wp_idx if dist_to_next_wp < dist_between_next_wps else closest_idx

    def get_next_waypoints(self, waypoints, start_wp, end_wp):
        next_waypoints = []
        for idx in range(start_wp, end_wp):
            next_waypoints.append(waypoints[self.sanitize_wp_index(idx)])

        return next_waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
