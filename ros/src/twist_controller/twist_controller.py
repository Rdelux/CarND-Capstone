import rospy

from pid import PID
from PID_steering import PID_steering
from yaw_controller import YawController
from lowpass import LowPassFilter
import math
import numpy as np
from scipy import interpolate
import tf

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit,
                wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        
        #self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        kp = 0.3
        ki = 0
        kd = 0.5
        mn = -1.7
        mx = 1.7
        self.yaw_controller1 = PID_steering(kp, ki, kd, mn, mx)

        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)
        self.steer_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, \
                base_lane, car_coordintes, isTrafficLightAhead):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.yaw_controller1.reset()
            return 0., 0., 0.

        ################################
        CTE = 0
        if (car_coordintes != None and base_lane != None):
            car = car_coordintes.pose.position
            orient = car_coordintes.pose.orientation
            euler = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            car_yaw = euler[2]
            if(car_yaw < 0):
                car_yaw = car_yaw + (2 * math.pi)
            x = []
            y = []
            carx = car.x
            cary = car.y
            rospy.loginfo("caryaw %f, %f, %f", car.x, car.y,car_yaw)
            index = 0
            pt1 = base_lane.waypoints[index].pose.pose.position
            orient = base_lane.waypoints[index].pose.pose.orientation
            euler = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            pt1yaw = euler[2]
            rospy.loginfo("pt1yaw %f, %f, %f", pt1.x, pt1.y, pt1yaw)

            for i in range(len(base_lane.waypoints)):
                pos = base_lane.waypoints[i].pose.pose.position
                x_origin = pos.x - car.x
                y_origin = pos.y - car.y

                x_rotated = x_origin*math.cos(-car_yaw)-y_origin*math.sin(-car_yaw)
                y_rotated = x_origin*math.sin(-car_yaw)+y_origin*math.cos(-car_yaw)

                x.append(x_rotated)
                y.append(y_rotated)

            p = interpolate.interp1d(x, y, kind="cubic", fill_value="extrapolate")
            CTE = -p(0)

        ################################

        current_vel = self.vel_lpf.filt(current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        steering = self.yaw_controller1.step(CTE, sample_time)
        #steering = self.steer_lpf.filt(steering)

        rospy.loginfo("steering  %f", steering)
        rospy.loginfo("-------------->")
        #throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        throttle = 0.42

        if(isTrafficLightAhead == True):
            throttle = 0.1
        if(isTrafficLightAhead == True and current_vel > 4.47):
            throttle = 0
            brake = 200

        if linear_vel == 0.:
            throttle = 0
            brake = 1500
        elif vel_error < 0:
            throttle = 0
            brake = 50

        # A hack for a simulator bug
        # Just slow the car down between these points
        if(car.x > 1000 and car.x < 1400):
            if(car.y >2920 and car.y < 2980):
                throttle = .15
        if(car.x > 730 and car.y > 1128 and car.y < 1134):
            throttle = .15

        if(len(base_lane.waypoints) < 20):
            rospy.loginfo("stopping")
            throttle = 0
            brake = 3500
        return throttle, brake, steering

