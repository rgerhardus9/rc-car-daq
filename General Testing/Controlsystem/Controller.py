import time
import VelocityProfile

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
        self.desired_steering_angle = 0
        self.current_steering_angle = 0
        self.previous_steering_angle = 0
        self.steering_angle_error = 0
        self.steering_angle_derivative_error = 0
        self.steering_angle_integral_error = 0
        
    def update_pwm(self, desired_steeringAngle):
        # Get current time 
        current_time = time.time()
        # Get desired steering angle
        self.desired_steering_angle = desired_steeringAngle
        # Get current steering angle (m and b from MATLAB file "CarSteeringModel" relating PWM signal and steering angle centerline)
        self.current_steering_angle = 5.9435 * self.pwm - 89.1515
        # Update delta T
        self.delta_t = time.time() - self.last_updated
        # Calculate errors
        self.steering_angle_error = self.desired_steering_angle - self.current_steering_angle
        if self.delta_t != 0:
            self.steering_angle_derivative_error = (self.current_steering_angle - self.previous_steering_angle) / self.delta_t
        self.steering_angle_integral_error += self.steering_angle_error
        # Update PWM
        self.pwm = self.kp * self.steering_angle_error + self.ki * self.steering_angle_integral_error + self.kd * self.steering_angle_derivative_error
        # Contrain PWM
        self.pwm = max(10,min(self.pwm, 20))
        
        # Update previous variables 
        self.previous_steering_angle = self.current_steering_angle
        self.last_updated = time.time()
        # Add delay
        time.sleep(0.01)

    def reset(self):

        self.pwm = 15
        self.delta_t = 0
        self.last_updated = 0
        self.desired_steering_angle = 0
        self.current_steering_angle = 0
        self.previous_steering_angle = 0
        self.steering_angle_error = 0
        self.steering_angle_derivative_error = 0
        self.steering_angle_integral_error = 0