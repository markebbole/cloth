#ifndef CLOTHINSTANCE_H
#define CLOTHINSTANCE_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <list>
#include <vector>
#include "vectormath.h"
#include <QGLWidget>
#include "clothtemplate.h"
#include <Eigen/Geometry>
#include <iostream>
#include "collisiondetection.h"
#include "simparameters.h"

using namespace Eigen;
using namespace std;
class ClothTemplate;
struct AABBNode;
struct SimParameters;

class ClothInstance
{
public:
    ClothInstance(const ClothTemplate &ctemplate, Eigen::VectorXd x_init, const SimParameters &params);
    ~ClothInstance();

    void render();  

    int time;  

    Eigen::VectorXd x;
    Eigen::VectorXd v;

    Eigen::Vector3d color;

    AABBNode *AABB;

    const ClothTemplate &getTemplate() const {return ctemplate_;}
    void computeForces(VectorXd& F_el, VectorXd& F_d, SparseMatrix<double>& dFdx, SparseMatrix<double>& dFdv);

private:
    void computeShearForce(VectorXd& F, SparseMatrix<double> dFdx, vector < Tr > & dFdv);
    const SimParameters &params_;
    const ClothTemplate &ctemplate_;
};

#endif // CLOTHINSTANCE_H
