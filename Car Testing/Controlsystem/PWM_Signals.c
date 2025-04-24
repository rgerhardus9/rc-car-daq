#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>

#define PIN_THROTTLE 13
#define PIN_STEERING 2
#define FREQUENCY 100

void setupPWM(int pin, int dutyCycle) {
    pinMode(pin, PWM_OUTPUT);
    pwmWrite(pin, dutyCycle);
}

void mainLoop() {
    int dutyCycleThrottle = 15;
    int dutyCycleSteering = 15;

    printf("Generating PWM on GPIO %d with %dHz and %d%% duty cycle.\n", PIN_THROTTLE, FREQUENCY, dutyCycleThrottle);
    setupPWM(PIN_THROTTLE, dutyCycleThrottle);
    sleep(1);
    
    printf("Full Reverse\n");
    setupPWM(PIN_THROTTLE, 10);
    sleep(5);
    
    printf("Generating PWM on GPIO %d with %dHz and %d%% duty cycle.\n", PIN_STEERING, FREQUENCY, dutyCycleSteering);
    setupPWM(PIN_STEERING, dutyCycleSteering);
    sleep(1);
    
    printf("Full Reverse\n");
    setupPWM(PIN_STEERING, 10);
    sleep(5);
}

int main() {
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "Failed to initialize GPIO\n");
        return 1;
    }
    
    mainLoop();
    return 0;
}