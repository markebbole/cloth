#ifndef RIGIDBODYTEMPLATE_H
#define RIGIDBODYTEMPLATE_H

#include <string>
#include <Eigen/Core>
#include <set>

class SignedDistanceField;

class RigidBodyTemplate
{
public:
    RigidBodyTemplate(const std::string &meshFilename, double scale);
    RigidBodyTemplate(const Eigen::MatrixX3d &verts, const Eigen::MatrixX4i &tets);

    ~RigidBodyTemplate();

    double getVolume() const {return volume_;}
    const Eigen::Matrix3d getInertiaTensor() const {return inertiaTensor_;}    

    double getBoundingRadius() const {return radius_;}
    const Eigen::MatrixX3d &getVerts() const {return V;}
    const Eigen::MatrixX4i &getTets() const {return T;}
    const Eigen::MatrixX3i &getFaces() const {return F;}   
    //const Eigen::VectorXd &getVertexDistances() const { return vdf; }
    const Eigen::MatrixX2i &getDualEdges() const {return dualEdges;}
    Eigen::Vector3d com_;

private:
    RigidBodyTemplate(const RigidBodyTemplate &other);
    RigidBodyTemplate &operator=(const RigidBodyTemplate &other);

    void initialize();

    void computeFaces();
    void computeVolume();
    Eigen::Vector3d computeCenterOfMass();
    void computeInertiaTensor();

    Eigen::MatrixX3d V;
    Eigen::MatrixX4i T;
    Eigen::MatrixX3i F;
    Eigen::MatrixX2i dualEdges;
    //Eigen::VectorXd vdf;

    double volume_;
    double radius_;
    Eigen::Matrix3d inertiaTensor_;

};

#endif // RIGIDBODYTEMPLATE_H
