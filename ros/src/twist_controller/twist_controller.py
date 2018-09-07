import rospy

from pid import PID
from PID_steering import PID_steering
from lowpass import LowPassFilter
import math
from scipy import interpolate
import tf
from styx_msgs.msg import NavType

STOPPING_THRESHOLD = 20

class Controller(object):
    def __init__(self, wheel_base, steer_ratio):
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.42
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        kp = 0.3
        ki = 0
        kd = 0.5
        mn = -1.7
        mx = 1.7
        self.PID_controller = PID_steering(kp, ki, kd, mn, mx)

        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio

        self.last_time = rospy.get_time()

    def PID(self, car_coordintes, base_lane, sample_time):
        CTE = 0
        steering = 0
        if (car_coordintes != None and base_lane != None and len(base_lane.waypoints) > 0):
            car = car_coordintes.pose.position
            orient = car_coordintes.pose.orientation
            euler = tf.transformations.euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            car_yaw = euler[2]
            if(car_yaw < 0):
                car_yaw = car_yaw + (2 * math.pi)
            x = []
            y = []
            
            for i in range(len(base_lane.waypoints)):
                pos = base_lane.waypoints[i].pose.pose.position
                x_origin = pos.x - car.x
                y_origin = pos.y - car.y

                x_rotated = x_origin*math.cos(-car_yaw)-y_origin*math.sin(-car_yaw)
                y_rotated = x_origin*math.sin(-car_yaw)+y_origin*math.cos(-car_yaw)

                x.append(x_rotated)
                y.append(y_rotated)

            p = interpolate.interp1d(x, y, kind="quadratic", fill_value="extrapolate")
            CTE = -p(0)
            steering = self.PID_controller.step(CTE, sample_time)
        return steering

    def wp_follower(self, linear_vel, angular_vel):
        from math import atan
        r = linear_vel/angular_vel
        angle = math.atan(self.wheel_base/r)
        steering = angle * self.steer_ratio
        return steering
    
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel, \
                base_lane, car_coordintes, isTrafficLightAhead, navtype):
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            self.PID_controller.reset()
            return 0., 0., 0.

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        steering = 0
        if(navtype == NavType.PID):
            steering = self.PID(car_coordintes, base_lane, sample_time)
        else:
            steering = self.wp_follower(linear_vel, angular_vel)
            global STOPPING_THRESHOLD
            STOPPING_THRESHOLD = 4

        current_vel = self.vel_lpf.filt(current_vel)
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        brake = 0
        throttle = 0
        throttle = self.throttle_controller.step(vel_error, sample_time)

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
        car = car_coordintes.pose.position
        if(car is not None):
            if(car.x > 1000 and car.x < 1400):
                if(car.y >2920 and car.y < 2980):
                    throttle = .15

            #End of the highway track
            if(car.x > 730 and car.y > 1128 and car.y < 1134):
                throttle = .10

        if(len(base_lane.waypoints) < STOPPING_THRESHOLD):
            throttle = 0
            brake = 10000
        return throttle, brake, steering

