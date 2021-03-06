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
import yaml
import math

import cv2
import os
import rospkg


STATE_COUNT_THRESHOLD = 2


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.has_image = False
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = None
        self.slps = []

        self.listener = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.waypoints = rospy.wait_for_message('/base_waypoints', Lane).waypoints  # Only get base_waypoints once
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        # List of positions that correspond to the line to stop in front of a given intersection.
        for slp in self.config['stop_line_positions']:
            tl = TrafficLight()
            tl.pose.pose.position.x, tl.pose.pose.position.y, tl.pose.pose.position.z = slp[0], slp[1], 0
            self.slps.append(self.get_closest_waypoint(tl.pose.pose.position))

        rospy.Subscriber('/image_color', Image, self.image_cb)

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
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        if not self.lights or not self.waypoints or not self.pose:
            pass

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        # Does the detected state not match the previous state?
        if self.state != state:
            # Yes - they're different.  Reset the counter.
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            # Elseif the state count is above the persistence threshold
            self.last_state = self.state

            light_wp = light_wp
            if state == TrafficLight.UNKNOWN or state == TrafficLight.GREEN:
                light_wp = -1

            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            position (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        dist = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        return min(xrange(len(self.waypoints)),
                   key=lambda p: dist(position, self.waypoints[p].pose.pose.position))

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location
        Args:
            point_in_world (Point): 3D location of a point in the world
        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image
        """
        # fx = self.config['camera_info']['focal_length_x']
        # fy = self.config['camera_info']['focal_length_y']
        # image_width = self.config['camera_info']['image_width']
        # image_height = self.config['camera_info']['image_height']
        #
        # # get transform between pose of camera and world frame
        # trans = None
        # try:
        #     now = rospy.Time.now()
        #     self.listener.waitForTransform("/base_link",
        #                                    "/world", now, rospy.Duration(1.0))
        #     (trans, rot) = self.listener.lookupTransform("/base_link",
        #                                                  "/world", now)
        #
        # except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        #     rospy.logerr("Failed to find camera to map transform")

        # TODO Use transform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return x, y

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        if not self.has_image:
            return TrafficLight.RED

        # fixing convoluted camera encoding...
        if hasattr(self.camera_image, 'encoding'):
            if self.camera_image.encoding == '8UC3':
                self.camera_image.encoding = "rgb8"
        else:
            self.camera_image.encoding = 'rgb8'
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        if self.listener is not None:

            # x, y = self.project_to_image_plane(light.pose.pose.position)

            # TODO use light location to zoom in on traffic light in image

            # Get classification:
            pred = self.light_classifier.get_classification(cv_image)

            # # Here we automatically store wrongly classified images into a sub-folder for later re-training:
            # if pred != light.state:  # NOTE: this doesn't work for no-light predictions!
            #     image_path = rospkg.RosPack().get_path('tl_detector') \
            #                  + '/light_classification/Traffic_light_sim_only/aaa_new_samples/'
            #     folder = {TrafficLight.RED: 'red/',
            #               TrafficLight.YELLOW: 'yellow/',
            #               TrafficLight.GREEN: 'green/',
            #               TrafficLight.UNKNOWN: 'none/'}
            #     filepath = image_path + folder[light.state]
            #     path, dirs, files = os.walk(filepath).next()
            #     filename = filepath + 'wrong_' + str(len(files)).zfill(5) + '.png'
            #     RGB_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            #     cv2.imwrite(filename, RGB_img)
            #     rospy.logwarn("Light state: %s,   Light detected: %s", light.state, pred)

            return pred

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if self.pose and self.waypoints:
            car_position = self.get_closest_waypoint(self.pose.pose.position)

            # Finding the closest visible traffic light (if one exists)
            func = lambda p: self.slps[p] - car_position if self.slps[p] >= car_position \
                else len(self.waypoints) + self.slps[p] - car_position
            idx = min(xrange(len(self.slps)), key=func)
            light_wp = self.slps[idx]

            # Only start processing lights if we're close enough:
            if light_wp >= car_position and (light_wp - car_position) <= 150:
                light = self.lights[idx]
                state = self.get_light_state(light)
                return light_wp, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
