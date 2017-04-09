#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <set>
#include <QMutex>
#include "simparameters.h"
#include <QGLWidget>
#include "distance.h"

class RigidBodyTemplate;
class RigidBodyInstance;

typedef Eigen::Triplet<double> Tr;

struct SimParameters;
struct Collision;

class Simulation
{
public:
    Simulation(const SimParameters &params);
    ~Simulation();

    void takeSimulationStep();
    void initializeGL();

    void renderObjects();
    void clearScene();    

    Eigen::Vector3d getBodyPosition(int body);
    double getBodyBoundingRadius(int body);

    //double relativeVelocity(int, int, int, int);
    //Eigen::VectorXd calculateDG(int b1, int b2, int v, int b2TetIndex);

private:
    void loadSphere();

    void computeForces(Eigen::VectorXd &Fc, Eigen::VectorXd &Ftheta);

    const SimParameters &params_;
    QMutex renderLock_;

    double time_;

    std::vector<RigidBodyTemplate *> templates_;
    std::vector<RigidBodyInstance *> bodies_;
};

#endif // SIMULATION_H
