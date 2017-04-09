#include "simparameters.h"

SimParameters::SimParameters()
{
    simRunning = false;
    timeStep = 60;
    NewtonMaxIters = 2000;
    NewtonTolerance = 1e-8;

    activeForces = F_GRAVITY | F_PENALTY | F_IMPULSE | F_FRACTURE;

    penaltyStiffness = 1e16;
    CoR = 0.2;

}
