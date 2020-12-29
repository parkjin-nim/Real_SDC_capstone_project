from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, 
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,max_steer_angle):
        
        # set steering pid controller with minimum speed 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        # set throttling pid controller with min. 0, max. 0.2 
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.
        mx = 0.2
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        # set lowpath filter
        tau = 0.5
        ts  = .02
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass    = vehicle_mass
        self.fuel_capacity   = fuel_capacity
        self.brake_deadband  = brake_deadband
        self.decel_limit     = decel_limit
        self.accel_limit     = accel_limit
        self.wheel_radius    = wheel_radius
        
        self.last_time = rospy.get_time()


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        
        # if dbw_enabled is disable, pid controller.
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.# Return throttle, brake, steer
        
        current_vel = self.vel_lpf.filt(current_vel)
        
        # get steering pid controller's output
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time  = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake    = 0
        
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake    = 700 #400
        
        # if car is faster than the goal, vel_error is neg.
        elif throttle < .1 and vel_error < 0: 
            throttle = 0
            # decel_limit is -5
            decel = max(vel_error, self.decel_limit)
            # brake unit is in torque(N*m)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
            
        return throttle, brake, steering
