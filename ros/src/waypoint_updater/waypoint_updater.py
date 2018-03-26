#!/usr/bin/env python

import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from copy import deepcopy

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

        # Get the max-velocity set in /ros/src/waypoint_loader/launch/waypoint_loader.launch
        self.max_velocity = rospy.get_param('/waypoint_loader/velocity', 40.0)  # in km/h

        self.last_velocity = self.max_velocity
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
        rate = rospy.Rate(10)  # define a 50Hz timer, according to the walkthrough the waypoint_follower runs at 30Hz
        while not rospy.is_shutdown():
            # check that we received callbacks 'pose_cb' and 'waypoints_cb':
            if self.last_waypoints is not None and self.last_pose is not None:
                lane = Lane()
                lane.header.stamp = rospy.Time().now()
                lane.header.frame_id = '/world'

                # calculate next and all following waypoints:
                waypoints = self.last_waypoints.waypoints
                next_wp = self.get_closest_waypoint(self.last_pose, waypoints)
                # Get all waypoints - this adjusts for traffic lights as well:
                lane.waypoints = self.get_next_waypoints(waypoints, next_wp, next_wp + LOOKAHEAD_WPS)

                # rospy.logwarn('Next wp: {} out of {}'.format(next_wp, len(waypoints)))

                self.final_waypoints_pub.publish(lane)

            rate.sleep()

    def pose_cb(self, msg):
        self.last_pose = msg

    def waypoints_cb(self, msg):
        # Note: according to the classroom this will be published once only!! (always use a copy of waypoints!)
        self.last_waypoints = msg
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in msg.waypoints]
            self.waypoint_tree = KDTree( self.waypoints_2d )

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

    def get_waypoint_velocity(self, waypoints, waypoint):
        return waypoints[waypoint].twist.twist.linear.x

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
        #curr_dist_to_wp = lambda wp: self.raw_dist(pose.pose.position, wp.pose.pose.position)
        #closest_wp = min(waypoints, key=curr_dist_to_wp)
        #closest_idx = waypoints.index(closest_wp)  # TODO: there should be a better way to do this!?!

        # check if closest_wp is behind us:
        #next_wp_idx = self.sanitize_wp_index(closest_idx+1)
        #dist_between_next_wps = self.distance(waypoints, closest_idx, closest_idx+1)  # don't sanitize here
        #dist_to_next_wp = curr_dist_to_wp(waypoints[next_wp_idx])

        # TODO: Will need to make this check more robust when we're deviating from the path.
        #       e.g. current position is exactly perpendicular next to clostest_wp
        #return next_wp_idx if dist_to_next_wp < dist_between_next_wps else closest_idx

        x = pose.pose.position.x
        y = pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect-cl_vect)
        if (val > 0):
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx


    def check_need_brake(self, waypoints, start_wp, end_wp):
        """ Returns (bool, new end_waypoint) if we need to brake and where to stop completely """
        need_brake = False
        if self.next_tl_idx >= 0:
            tl_idx = self.next_tl_idx if self.next_tl_idx >= start_wp else (self.next_tl_idx + len(waypoints))
            curr_vel = self.last_velocity  # cached for printing same value

            # TODO: this is a very bad heuristic: Maybe something like the 'torque' value in twist_controller.py
            brake_dist = curr_vel * 4  # as measured: reduced speed by 20 m/s within 80 waypoints
            max_brake_dist = max(5, int(1.4 * brake_dist))  # don't brake before that value (but at least 5wps before)
            min_brake_dist = int(0.25 * brake_dist)  # don't brake after that value (e.g. yellow light at full speed)

            within_brake_window = (tl_idx - start_wp) >= min_brake_dist and (tl_idx - start_wp) <= max_brake_dist

            # Note: Once we started braking for a traffic light we always want to continue doing so.
            #       Unless we reduced the speed a lot already then allow for closing the gap if outside brake-window:
            if (curr_vel > 5.0 and self.next_tl_idx == self.braking_for_tl_idx) or within_brake_window:
                end_wp = tl_idx + 1  # range function below should use 'end_wp' itself as well
                need_brake = True
                self.braking_for_tl_idx = self.next_tl_idx  # remember that we started braking for this traffic light

            # rospy.logwarn('idx: {0} before: {1}, cur_vel: {2:5.2f} -> brake: {3} , max/min: [{4},{5}]'.format(
            #     start_wp, tl_idx, curr_vel, int(need_brake), max_brake_dist, min_brake_dist))

        return need_brake, end_wp


    def get_next_waypoints(self, waypoints, start_wp, end_wp):

        # Check if we need to brake and determine the new end_waypoint in that case:
        need_brake, tl_wp = self.check_need_brake(waypoints, start_wp, end_wp)

        # TODO: reactivate, deactivated for testing
        need_brake = True

        tl_wp = self.next_tl_idx

        next_waypoints = waypoints[start_wp:end_wp]
        
        if need_brake and tl_wp >= 0 and tl_wp <= end_wp and tl_wp >= start_wp:
            show_debug = False
            if self.log_counter > 10:
                self.log_counter = 0
                show_debug = True
                #rospy.logwarn('start_wp: {0} tl_wp: {1}, end_wp: {2}'.format(start_wp, tl_wp, end_wp))
            self.log_counter = self.log_counter + 1

            temp = []
            for i, wp in enumerate(next_waypoints):
                p = Waypoint()
                p.pose = wp.pose
                stop_idx = max( tl_wp - start_wp - 2, 0 )
                dist = self.distance(next_waypoints, i, stop_idx)
                vel = 2.8 * math.sqrt( 1.5 * dist ) - 2
                # some more try's
                #vel2 = ((dist + 1)/60.0) * wp.twist.twist.linear.x
                #vel3 = (1.0 / 4.0) * math.sqrt( 0.2 * (dist) ) * wp.twist.twist.linear.x;
                #vel3 = vel3 - (0.1 * wp.twist.twist.linear.x)
                #if show_debug:
                #    rospy.logwarn('    vel: {0}  vel2: {3}  vel3: {4}  dist: {1}  wp.twist.twist.linear.x: {2}'.format(vel, dist, wp.twist.twist.linear.x, vel2, vel3))
                #    show_debug = False
                #vel = vel3
                if vel < 1:
                    vel = 0
                    show_debug = False
                p.twist.twist.linear.x = min( vel, wp.twist.twist.linear.x)
                temp.append( p )
            next_waypoints = temp

        return next_waypoints

        # need_brake, end_wp = self.check_need_brake(waypoints, start_wp, end_wp)

        # # NOTE: we need to take copies of /base_waypoints as they will be published exactly once!
        # next_waypoints = []
        # for idx in range(start_wp, end_wp):
        #     next_waypoints.append(deepcopy(waypoints[self.sanitize_wp_index(idx)]))

        # curr_vel = self.last_velocity  # cached for whole method

        # if need_brake:
        #     if start_wp == end_wp:
        #         # wait until we get a green light:
        #         self.set_waypoint_velocity(next_waypoints, 0, 0.0)
        #     else:
        #         num_wps = len(next_waypoints)
        #         # adjust all velocities to end up at 0 at the last waypoint:
        #         for i, wp in enumerate(next_waypoints):
        #             adj = curr_vel/4.0  # adjustment for very high speeds
        #             self.set_waypoint_velocity(next_waypoints, (num_wps - 1) - i, max(0, i * curr_vel / num_wps - adj))

        # return next_waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
