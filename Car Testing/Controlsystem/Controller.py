import time
import VelocityProfile
# from cameraComputeDC import camera_get_dc


class ThrottleController:
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.pwm = 15.0
        self.xd = 0
        self.vd = 0
        self.x = 0
        self.x_previous = 0
        self.v = 0
        self.last_update = 0  # Initialize with 0
        self.delta_t = 0  # Initialize delta_t
        self.position_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.velocity_profile = VelocityProfile.VelocityProfile



    def update_pwm(self, start_time, velocity_profile):
        current_time = time.time()  # Convert to milliseconds
        # Get current position
        # self.x = self.readEncoders        # RESTORE THIS
        self.x = 2
        #Calculate current position

        # Get desired position
        self.xd = velocity_profile.get_desired_position(start_time)
        # Calculate delta t
        self.delta_t = (time.time() - self.last_update) 
        # Calculate velocity
        if self.delta_t != 0:
            print(self.x)
            print(self.x_previous)
            self.v = (self.x - self.x_previous) / self.delta_t
        #Get desired velocity
        self.vd = velocity_profile.get_desired_velocity(start_time)
        # Calculate Errors
        self.position_error = self.xd - self.x
        self.integral_error += self.position_error
        self.integral_error = max(-1000, min(1000, self.integral_error)) # Prevent integral windup #Choose different values maybe
        self.derivative_error = self.vd - self.v
        # Calculate next PWM value
        self.pwm = self.Kp * self.position_error + self.Ki * self.integral_error + self.Kd * self.derivative_error
        # Constrain between 10 and 20% DC
        self.pwm = max(10, min(20, self.pwm))  
        
        # Update previous variables
        self.last_update = time.time()
        self.x_previous = self.x
        # Add delay
        time.sleep(0.01)

    def readEncoders(self): # Need to write that
        self.x
        return int(self.x)


    def reset(self):
        self.x = 0
        self.xd = 0
        self.x_previous = 0
        self.v = 0
        self.vd = 0
        self.pwm = 0
        self.position_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.last_update = 0
        self.delta_t = 0 



class SteeringController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pwm = 15.0
        self.delta_t = 0
        self.last_updated = 0
        self.desired_dist_to_centerline = 0
        self.current_dist_to_centerline = 0
        self.previous_dist_to_centerline = 0
        self.error_dist_to_centerline = 0
        self.derivative_error_dist_to_centerline = 0
        self.integral_error_dist_to_centerline = 0
        
    def update_pwm(self, desired_dist_to_centerline, actual_dist_to_center):
        # Get current time 
        current_time = time.time()
        # Get desired steering angle
        self.desired_dist_to_centerline = desired_dist_to_centerline
        # Get current steering angle (m and b from MATLAB file "CarSteeringModel" relating PWM signal and steering angle centerline)   
        #    
        self.current_dist_to_centerline = -1 * actual_dist_to_center     #camera value


        # Update delta T
        self.delta_t = time.time() - self.last_updated
        # Calculate errors
        self.error_dist_to_centerline = self.desired_dist_to_centerline - self.current_dist_to_centerline
        if self.delta_t != 0:
            self.derivative_error_dist_to_centerline = (self.current_dist_to_centerline - self.previous_dist_to_centerline) / self.delta_t
        self.integral_error_dist_to_centerline += self.error_dist_to_centerline
        # Update PWM
        self.pwm = 15 + self.kp * self.error_dist_to_centerline + self.ki * self.integral_error_dist_to_centerline + self.kd * self.derivative_error_dist_to_centerline

        print(f"Integral: {self.ki * self.integral_error_dist_to_centerline}")
        print(f"Proportional: {self.kp * self.error_dist_to_centerline}")
        # Contrain PWM
        self.pwm = max(10,min(self.pwm, 20))
        
        # Update previous variables 
        self.previous_dist_to_centerline = self.current_dist_to_centerline
        self.last_updated = time.time()
        # Add delay
        time.sleep(0.01)

    def reset(self):

        self.pwm = 15
        self.delta_t = 0
        self.last_updated = 0
        self.desired_dist_to_centerline = 0
        self.current_dist_to_centerline = 0
        self.previous_dist_to_centerline = 0
        self.error_dist_to_centerline = 0
        self.derivative_error_dist_to_centerline = 0
        self.integral_error_dist_to_centerline = 0