#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h> // For usleep function

#define V_STEADY 20.5  // Max velocity (m/s)
#define ACCEL 10.25     // Max acceleration (m/s^2)
#define D_TOTAL 82.0    // Total displacement (m)

double PWM_signal_throttle = 15;
double PWM_signal_steering = 15;

typedef struct {
    double Kp, Ki, Kd;
    double pwm;
    double xd, vd;
    double x, x_previous;
    double v;
} ThrottleController;

typedef struct {
    double kp, ki, kd;
    double pwm;
    double desired_steering_angle;
    double current_steering_angle;
} SteeringController;

void main() {
    ThrottleController throttleController = {1.0, 1.0, 1.0, 0, 0, 0, 0, 0};
    SteeringController steeringController = {0.061388, 0.682021, 0.0, 15, 15};
    
    double start_time = time(NULL);
    
    while (throttleController.x <= D_TOTAL) {
        PWM_signal_throttle = throttleController.pwm;
        PWM_signal_steering = steeringController.pwm;
        
        printf("Desired Position: %.2f m\n", throttleController.xd);
        printf("Current Position: %.2f m\n", throttleController.x);
        printf("Desired Velocity: %.2f m/s\n", throttleController.vd);
        printf("Current Velocity: %.2f m/s\n", throttleController.v);
        printf("\n"); // Blank line
        printf("Desired Steering Angle: %.2f degrees\n", steeringController.desired_steering_angle);
        printf("Current Steering Angle: %.2f degrees\n", steeringController.current_steering_angle);
        
        usleep(100000); // 100 ms delay
    }
}
