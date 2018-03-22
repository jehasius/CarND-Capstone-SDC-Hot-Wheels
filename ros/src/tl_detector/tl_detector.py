#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

from bisect import bisect_left


class Position:
    """ simple class that has a x and a y value """
    def __init__(self, given):
        self.x = given[0]
        self.y = given[1]

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        # all traffic_light indexes will be calculated only once in process_traffic_lights():
        self.config = yaml.load(rospy.get_param("/traffic_light_config"))
        self.all_tl_indexes = None

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # rospy.logwarn('got traffic lights of length: {}'.format(len(msg.lights)))
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint_idx(self, position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            position: (x,y) position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        # TODO: maybe replace? It's almost a copy from the waypoint_updater code
        if self.waypoints is not None:
            raw_dist = lambda wp1, wp2: math.sqrt((wp1.x - wp2.x) ** 2 + (wp1.y - wp2.y) ** 2)
            curr_dist_to_wp = lambda wp: raw_dist(position, wp.pose.pose.position)
            closest_wp = min(self.waypoints, key=curr_dist_to_wp)
            return self.waypoints.index(closest_wp)  # TODO: there should be a better way to do this!?!

        return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        if self.all_tl_indexes is None and self.waypoints is not None:
            stop_line_positions = self.config['stop_line_positions']
            self.all_tl_indexes = []
            for stop_line in stop_line_positions:
                wp_idx = self.get_closest_waypoint_idx(Position(stop_line))
                self.all_tl_indexes.append(wp_idx % len(self.waypoints))
            self.all_tl_indexes = sorted(self.all_tl_indexes)  # sort it for binary search!
            rospy.logwarn('all_tl_indexes: {}'.format(self.all_tl_indexes))


        #TODO find the closest visible traffic light (if one exists)


        # THIS USES A FAKE TRAFFIC LIGHT FINDER IN THE SIMULATOR
        # Please replace this as soon as we have a real detector:

        if self.pose is not None and self.all_tl_indexes is not None and self.lights is not None:
            car_idx = self.get_closest_waypoint_idx(self.pose.pose.position)
            # find next traffic light with binary search:
            res = bisect_left(self.all_tl_indexes, car_idx)
            tl_idx = self.all_tl_indexes[res] if res < len(self.all_tl_indexes) else self.all_tl_indexes[0]

            # TODO: the following doesn't work properly when looping around the track!
            if abs(tl_idx - car_idx) < 50:
                # find that traffic light state in data given by /vehicle/traffic_lights:
                min_idx_distance = len(self.waypoints) + 1
                current_state = TrafficLight.UNKNOWN
                for tl in self.lights:
                    idx = self.get_closest_waypoint_idx(tl.pose.pose.position)
                    idx = idx -30  # light-position is behind stop-position
                    # rospy.logwarn('idx: {}  state: {}'.format(idx, tl.state))
                    if tl_idx >= idx and (tl_idx - idx) < min_idx_distance:
                        min_idx_distance = (tl_idx - idx)
                        current_state = tl.state

                rospy.logwarn('curr idx: {}  Next tl: {} state: {}'.format(car_idx, tl_idx, current_state))
                return tl_idx, current_state


        # Please use this as soon as we have a real detector:
        # if light:
        #     state = self.get_light_state(light)
        #     return light_wp, state

        # self.waypoints = None  ## why would we reset static data?!?
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
