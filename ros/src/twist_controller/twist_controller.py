import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704  # in m/s


# TODO: from the project submission page:
# Be sure to check that this is working by testing with different values for kph velocity parameter in
# /ros/src/waypoint_loader/launch/waypoint_loader.launch


# TODO: Parameters need tuning!!
# Right now they're veeery fuzzy, but smooth!
# E.g. try max speed==140km/h -> It stays within the three lanes, but not very close to target.
PID_P = 0.5
PID_I = 0.001
PID_D = 0.1


class Controller(object):
    def __init__(self, *args, **kwargs):

        # Setup PID controller:
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        self.pid = PID(PID_P, PID_I, PID_D, decel_limit, accel_limit)

        # Setup YawController:
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        self.yaw_controller = YawController(wheel_base, steer_ratio, 1.0, max_lat_accel, max_steer_angle)

        # From https://discussions.udacity.com/t/412339 :
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.torque = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius

        # TODO: how to use brake_deadband properly?
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)

    def control(self, dbw_enabled, last_velocity, last_twist_cmd):

        throttle = brake = steering = 0.0

        if dbw_enabled:

            # TODO: track real time delta!
            time_delta = 0.02  # assumes 50Hz

            # TODO: do we need to use linear.y as well to get a proper error?
            error = last_twist_cmd.twist.linear.x - last_velocity.twist.linear.x

            # TODO: should we use the given LowPassFilter here?
            throttle = self.pid.step(error, time_delta)

            # From classroom -> '5.DBW Node':
            # Note that throttle values passed to publish should be in the range 0 to 1, throttle of 1 means full
            # Brake values passed to publish should be in units of torque (N*m).
            if throttle < 0.0:
                brake = -throttle * self.torque
                throttle = 0.0

            # TODO: should we use the given LowPassFilter here?
            # TODO: do we need to calculate velocities with linear.y as well?
            steering = self.yaw_controller.get_steering(last_twist_cmd.twist.linear.x,
                                                        last_twist_cmd.twist.angular.z,
                                                        last_velocity.twist.linear.x)

        else:  # dbw not enabled
            self.pid.reset()

        return throttle, brake, steering
