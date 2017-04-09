#ifndef RIGIDBODYINSTANCE_H
#define RIGIDBODYINSTANCE_H

#include <Eigen/Core>
#include <list>
#include <vector>

class RigidBodyTemplate;
struct AABBNode;

class RigidBodyInstance
{
public:
    RigidBodyInstance(const RigidBodyTemplate &rbtemplate, const Eigen::Vector3d &c, const Eigen::Vector3d &theta, double density);
    ~RigidBodyInstance();

    void render();  

    int time;  

    Eigen::Vector3d c;
    Eigen::Vector3d theta;

    Eigen::Vector3d cvel;
    Eigen::Vector3d w;

    double density;
    Eigen::Vector3d color;

    AABBNode *AABB;

    const RigidBodyTemplate &getTemplate() const {return rbtemplate_;}

private:
    const RigidBodyTemplate &rbtemplate_;
};

#endif // RIGIDBODYINSTANCE_H
