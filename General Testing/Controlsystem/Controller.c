#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h> // For usleep function

typedef struct {
    double Kp, Ki, Kd;
    double pwm;
    double xd, vd;
    double x, x_previous;
    double v;
    double last_update;
    double deltaT;
    double position_error, integral_error, derivative_error;
} ThrottleController;

typedef struct {
    double kp, ki, kd;
    double pwm;
    double deltaT, last_updated;
    double desired_steering_angle;
    double current_steering_angle;
    double previous_steering_angle;
    double steering_angle_error;
    double steering_angle_derivative_error;
    double steering_angle_integral_error;
} SteeringController;

// Function prototypes
double get_desired_position(double start_time);
double get_desired_velocity(double start_time);
double readEncoders();
double get_current_time();

void throttle_update_pwm(ThrottleController *tc, double start_time) {
    double current_time = get_current_time();
    
    tc->x = readEncoders();
    tc->xd = get_desired_position(start_time);
    tc->deltaT = current_time - tc->last_update;

    if (tc->deltaT != 0)
        tc->v = (tc->x - tc->x_previous) / tc->deltaT;

    tc->vd = get_desired_velocity(start_time);
    tc->position_error = tc->xd - tc->x;
    tc->integral_error += tc->position_error;

    // Prevent integral windup
    if (tc->integral_error > 1000) tc->integral_error = 1000;
    if (tc->integral_error < -1000) tc->integral_error = -1000;

    tc->derivative_error = tc->vd - tc->v;
    tc->pwm = tc->Kp * tc->position_error + tc->Ki * tc->integral_error + tc->Kd * tc->derivative_error;

    // Constrain between 10 and 20% DC
    if (tc->pwm > 20) tc->pwm = 20;
    if (tc->pwm < 10) tc->pwm = 10;

    tc->last_update = get_current_time();
    tc->x_previous = tc->x;
    
    usleep(10000); // 10 ms delay
}

void throttle_reset(ThrottleController *tc) {
    tc->x = tc->xd = tc->x_previous = 0;
    tc->v = tc->vd = 0;
    tc->pwm = 0;
    tc->position_error = tc->integral_error = tc->derivative_error = 0;
    tc->last_update = 0;
    tc->deltaT = 0;
}

void steering_update_pwm(SteeringController *sc, double desired_steeringAngle, double current_steering_angle) {
    double current_time = get_current_time();
    
    sc->desired_steering_angle = desired_steeringAngle;
    sc->current_steering_angle = current_steering_angle;
    sc->deltaT = current_time - sc->last_updated;
    sc->steering_angle_error = sc->desired_steering_angle - sc->current_steering_angle;

    if (sc->deltaT != 0)
        sc->steering_angle_derivative_error = (sc->current_steering_angle - sc->previous_steering_angle) / sc->deltaT;

    sc->steering_angle_integral_error += sc->steering_angle_error;
    sc->pwm = sc->kp * sc->steering_angle_error + sc->ki * sc->steering_angle_integral_error + sc->kd * sc->steering_angle_derivative_error;

    // Constrain PWM
    if (sc->pwm > 17) sc->pwm = 17;
    if (sc->pwm < 13) sc->pwm = 13;

    sc->previous_steering_angle = sc->current_steering_angle;
    sc->last_updated = get_current_time();
    
    usleep(10000); // 10 ms delay
}

void steering_reset(SteeringController *sc) {
    sc->pwm = 15;
    sc->deltaT = sc->last_updated = 0;
    sc->desired_steering_angle = sc->current_steering_angle = sc->previous_steering_angle = 0;
    sc->steering_angle_error = sc->steering_angle_derivative_error = sc->steering_angle_integral_error = 0;
}

// Dummy implementations for external functions
double get_desired_position(double start_time) {
    return 10.0; // Example value
}

double get_desired_velocity(double start_time) {
    return 2.0; // Example value
}

double readEncoders() {
    return rand() % 100; // Example random encoder value
}

double get_current_time() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

int main() {
    ThrottleController tc = {1.0, 0.1, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    SteeringController sc = {1.0, 0.1, 0.05, 15, 0, 0, 0, 0, 0, 0, 0, 0};

    double start_time = get_current_time();

    for (int i = 0; i < 100; i++) {
        throttle_update_pwm(&tc, start_time);
        steering_update_pwm(&sc, 10.0, 5.0);
        printf("Throttle PWM: %.2f, Steering PWM: %.2f\n", tc.pwm, sc.pwm);
        usleep(50000); // 50 ms delay
    }

    return 0;
}
