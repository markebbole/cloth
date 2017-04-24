#include "simparameters.h"

SimParameters::SimParameters()
{
    simRunning = false;
    timeStep = .01;

    activeForces = F_GRAVITY | F_COLLISION_REPULSION | F_STRETCH | F_SHEAR | F_BEND;

    clothWidth = 6;

    springStiffness = 1000.;
    shearStiffness = 100.;
    bendStiffness = 0.000001;
    dampingStiffness = 0.1;
    clothSideLen = .3;
    gravityC = 9.8;
    clothDensity = 1.;

}
