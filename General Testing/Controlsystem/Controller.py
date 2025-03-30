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
        self.last_update = time.monotonic()
        self.delta_t = 0  # Initialize delta_t
        self.proportional_error = 0
        self.proportional_error_previous = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.velocity_profile = VelocityProfile.VelocityProfile


    def update_pwm(self, velocity_profile, start_time):
        # Get current position
        # self.x = self.getCurrentPosition        # RESTORE THIS
        self.x = 0
        #Calculate current position

        # Get desired position
        self.xd = velocity_profile.get_desired_position(start_time)
        # Calculate delta t in seconds
        self.delta_t = (time.monotonic() - self.last_update)         
        print(f"Delta T: {self.delta_t:.2f} ")      
        # Calculate velocity
        if self.delta_t != 0:
            self.v = (self.x - self.x_previous) / self.delta_t
        #Get desired velocity
        self.vd = velocity_profile.get_desired_velocity(start_time)
        # Calculate Errors
        # P
        self.proportional_error = self.vd - self.v
        # I
        self.integral_error += self.proportional_error
        self.integral_error = max(-1000, min(1000, self.integral_error)) # Prevent integral windup #Choose different values maybe
        # D
        if self.delta_t > 0:
            self.derivative_error = (self.proportional_error - self.proportional_error_previous) / self.delta_t
        # Calculate next PWM value
        self.pwm = self.Kp * self.proportional_error + self.Ki * self.integral_error + self.Kd * self.derivative_error
        # Constrain between 10 and 20% DC
        self.pwm = max(15, min(20, self.pwm))  
        
        # Update previous variables
        self.last_update = time.monotonic()
        self.x_previous = self.x
        self.proportional_error_previous = self.proportional_error
        # # Add delay
        time.sleep(0.01)

    def getCurrentPosition(self): # Need to write that
        self.x
        return int(self.x)


    def reset(self):
        self.x = 0
        self.xd = 0
        self.x_previous = 0
        self.v = 0
        self.vd = 0
        self.pwm = 15
        self.proportional_error = 0
        self.integral_error = 0
        self.derivative_error = 0
        self.last_update = time.monotonic()
        self.delta_t = 0 
    



class SteeringController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pwm = 15.0
        self.delta_t = 0
        self.last_updated = time.monotonic()
        self.desired_steering_angle = 0
        self.current_steering_angle = 0
        self.previous_steering_angle = 0
        self.proportional_error = 0
        self.proportional_error_previous = 0
        self.derivative_error = 0
        self.integral_error = 0
        
    def update_pwm(self, desired_steeringAngle):
        # Get current time 
        current_time = time.monotonic()
        # Get desired steering angle
        self.desired_steering_angle = desired_steeringAngle
        # Get current steering angle (m and b from MATLAB file "CarSteeringModel" relating PWM signal and steering angle centerline)
        self.current_steering_angle = 5.9435 * self.pwm - 89.1515
        # Calculate delta T in seconds
        self.delta_t = time.monotonic() - self.last_updated
        print(f"Delta T: {self.delta_t:.2f} ")  
        # Calculate errors
        # P
        self.proportional_error = self.desired_steering_angle - self.current_steering_angle
        print(f"Proportional error: {self.proportional_error}")
        # I
        self.integral_error += self.proportional_error
        #self.integral_error = max((10 / self.ki) , min((20 / self.ki), self.integral_error))
        print(f"Integral error: {self.integral_error}")
        if self.delta_t > 0:
        # D
            self.derivative_error = (self.proportional_error - self.proportional_error_previous) / self.delta_t
            print(f"Derivative error: {self.derivative_error}")
        # Update PWM
        self.pwm = self.kp * self.proportional_error + self.ki * self.integral_error + self.kd * self.derivative_error
        print()
        print(f"Proportional Part: {self.proportional_error * self.kp}")
        print(f"Integral Part: {self.integral_error * self.ki}")
        print(f"Derivative Part: {self.derivative_error * self.kd}")
        print()

        # Contrain PWM
        self.pwm = max(10,min(self.pwm, 20))
        
        # Update previous variables 
        self.previous_steering_angle = self.current_steering_angle
        self.proportional_error_previous = self.proportional_error
        self.last_updated = time.monotonic()
        # # Add delay
        time.sleep(0.01)

    def reset(self):

        self.pwm = 15
        self.delta_t = 0
        self.last_updated = time.monotonic()
        self.desired_steering_angle = 0
        self.current_steering_angle = 0
        self.previous_steering_angle = 0
        self.proportional_error = 0
        self.derivative_error = 0
        self.integral_error = 0