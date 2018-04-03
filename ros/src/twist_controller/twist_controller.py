import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704  # in m/s

PID_P = 0.3
PID_I = 0.1
PID_D = 0.2
PID_MIN = 0  # Minimum throttle value
#PID_MAX = 1.0  # Maximum throttle value for simulator
PID_MAX = 0.6  # Maximum throttle value due to more sensitive behavior on Carla (measured against reference rosbag)
PID_LIMIT_INTEGRAL = 1


class Controller(object):
    def __init__(self, *args, **kwargs):

        # Setup PID controller:
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.pid = PID(PID_P, PID_I, PID_D, PID_LIMIT_INTEGRAL, PID_MIN, PID_MAX)

        # Setup low pass filter
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        # Setup YawController:
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        min_speed = 1.0

        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # From https://discussions.udacity.com/t/412339 :
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.torque = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius

        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.last_time = rospy.get_time()

    def control(self, dbw_enabled, last_velocity, last_twist_cmd):

        throttle = brake = steering = 0.0

        if dbw_enabled:

            # track real time delta:
            current_time = rospy.get_time()
            time_delta = current_time - self.last_time
            self.last_time = current_time

            # abs because the target velocity from the waypoint_follower seems to flip
            # between positive and negative velocities on large deviations
            target_velocity = abs(last_twist_cmd.twist.linear.x)

            current_vel = self.vel_lpf.filt(last_velocity.twist.linear.x)
            
            error = target_velocity - current_vel

            throttle = self.pid.step(error, time_delta)
            # from walkthrough:
            if throttle < 0.05 and error < 1:
                throttle = 0.0
                decel = max(error, self.decel_limit)
                brake = abs(decel) * self.torque
                # # to prevent windup
                # self.pid.reset()

            # hold the car in place when stopped
            if current_vel < 1 and target_velocity < 0.001:
                throttle = 0
                brake = 400
                # to prevent windup
                self.pid.reset()

            if brake < self.brake_deadband:
                brake = 0

            steering = self.yaw_controller.get_steering(target_velocity,
                                                        last_twist_cmd.twist.angular.z,
                                                        last_velocity.twist.linear.x)

        else:  # dbw not enabled
            self.pid.reset()

        return throttle, brake, steering
