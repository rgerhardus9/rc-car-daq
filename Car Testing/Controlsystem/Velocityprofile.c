#include <stdio.h>
#include <time.h>
#include <math.h>

typedef struct {
    double dTotal;
    double vsteady;
    double accel;
    double t1;
    double tsteady;
    double t2;
    double tfinal;
} VelocityProfile;

// Function to initialize the VelocityProfile structure
void initVelocityProfile(VelocityProfile *vp, double D_TOTAL, double V_STEADY, double ACCEL) {
    vp->dTotal = D_TOTAL;
    vp->vsteady = V_STEADY;
    vp->accel = ACCEL;
    vp->t1 = V_STEADY / ACCEL;
    vp->tsteady = (D_TOTAL / V_STEADY) - (V_STEADY / ACCEL);
    vp->t2 = (V_STEADY / ACCEL) + vp->tsteady;
    vp->tfinal = 2 * (V_STEADY / ACCEL) + vp->tsteady;
}

// Function to get the desired velocity
double getDesiredVelocity(VelocityProfile *vp, double start_time) {
    double now = (clock() / (double)CLOCKS_PER_SEC) - start_time;
    double vd = 0;

    if (now > vp->tfinal) {
        vd = 0;
    } else if (now >= vp->t2) {
        vd = vp->vsteady - vp->accel * (now - vp->t2);
    } else if (now >= vp->t1) {
        vd = vp->vsteady;
    } else if (now >= 0) {
        vd = vp->accel * now;
    } else {
        vd = 0;
    }

    printf("vd: %f\n", vd);
    return vd;
}

// Function to get the desired position
double getDesiredPosition(VelocityProfile *vp, double start_time) {
    double now = (clock() / (double)CLOCKS_PER_SEC) - start_time;
    double xd = 0;

    if (now > vp->tfinal) {
        xd = vp->dTotal;
    } else if (now >= vp->t2) {
        xd = vp->dTotal - 0.5 * vp->accel * pow((vp->tfinal - now), 2);
    } else if (now >= vp->t1) {
        xd = 0.5 * vp->accel * pow(vp->t1, 2) + vp->vsteady * (now - vp->t1);
    } else if (now >= 0) {
        xd = 0.5 * vp->accel * pow(now, 2);
    } else {
        xd = 0;
    }
    
    return xd;
}