import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self,params):
        self.params = params
        #self.speed_pid = PID(11.0, 0.02, 0.3, -params.accel_limit, params.accel_limit)
        self.speed_pid = PID(6.0, 0.5, 0.3, -params.accel_limit, params.accel_limit)
        self.tau = 3 #.5
        self.ts = 1 #.02
        self.lpf = LowPassFilter(self.tau, self.ts)

        self.yaw_controller = YawController(params.wheel_base,
                                            params.steer_ratio,
                                            params.min_speed,
                                            params.max_lat_accel,
                                            params.max_steer_angle)

        self.s_lpfilter = LowPassFilter(self.tau, self.ts)
        self.t_lpfilter = LowPassFilter(self.tau, self.ts)

    def reset(self):
        self.speed_pid.reset()

    def control(self, twist_cmd, current_velocity, dt, dbw_enabled ):

        linear_vel = abs(twist_cmd.twist.linear.x)
        angular_vel = twist_cmd.twist.angular.z
        vel_cte = linear_vel - current_velocity.twist.linear.x

        predictive_steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_velocity.twist.linear.x)
        predictive_steer = self.s_lpfilter.filt(predictive_steer )

        acceleration = self.speed_pid.step(vel_cte, dt)
        acceleration = self.t_lpfilter.filt(acceleration)

        # PID not to accumulate error under manual control
        if dbw_enabled:
            self.reset()

        brake = 0.0
        throttle = 0.0

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            if abs(acceleration) < self.params.brake_deadband:
                acceleration = 0.0
            total_mass = self.params.vehicle_mass + self.params.fuel_capacity * GAS_DENSITY
            brake = -acceleration * total_mass * self.params.wheel_radius

        return throttle, brake, predictive_steer
