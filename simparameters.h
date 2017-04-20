#ifndef SIMPARAMETERS_H
#define SIMPARAMETERS_H

struct SimParameters
{
    SimParameters();

    const static int F_GRAVITY = 1;
    const static int F_COLLISION_REPULSION = 2;
    const static int F_STRETCH = 4;
    const static int F_SHEAR = 8;
    const static int F_BEND = 16;


    bool simRunning;
    double timeStep;

    double springStiffness;
    double shearStiffness;
    double bendStiffness;
    double dampingStiffness;

    double clothSideLen;

    int activeForces;
    int clothWidth;

};

#endif // SIMPARAMETERS_H
