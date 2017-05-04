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
#include "obstacle.h"

// class RigidBodyTemplate;
// class RigidBodyInstance;
class ClothTemplate;
class ClothInstance;

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
    void loadScene(int sceneId);    

private:
    void loadSphere();

    void computeForces(Eigen::VectorXd &Fc, Eigen::VectorXd &Ftheta);
    void loadDefaultScene();
    void loadScene1();
    void loadScene2();
    void loadScene3();
    void deleteEverything();
    
    void makeCloth(Eigen::Matrix3d& rotation, Eigen::Vector3d trans);

    //void handleCollisions(std::set<Collision>& collisions, )

    const SimParameters &params_;
    QMutex renderLock_;

    double time_;

    std::vector<ClothTemplate *> cloth_templates_;
    std::vector<ClothInstance *> cloths_;

    std::vector<Obstacle*> obstacles_;
};

#endif // SIMULATION_H
