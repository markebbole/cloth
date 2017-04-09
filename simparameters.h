#ifndef SIMPARAMETERS_H
#define SIMPARAMETERS_H

struct SimParameters
{
    SimParameters();

    const static int F_GRAVITY = 1;
    const static int F_PENALTY = 2;
    const static int F_IMPULSE = 4;
    const static int F_FRACTURE = 8;

    bool simRunning;
    double timeStep;
    double NewtonTolerance;
    int NewtonMaxIters;

    int activeForces;

    double penaltyStiffness;
    double CoR;

};

#endif // SIMPARAMETERS_H
