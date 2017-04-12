#include "simulation.h"
#include <QGLWidget>
#include "simparameters.h"
#include <iostream>
#include <Eigen/Geometry>
#include <QDebug>
#include "rigidbodytemplate.h"
#include "rigidbodyinstance.h"
#include "clothtemplate.h"
#include "clothinstance.h"
#include "vectormath.h"
#include <Eigen/Dense>
#include <set>
#include <map>
#include <unordered_map>
#include "collisiondetection.h"
#include <utility>
#include <algorithm>

const double PI = 3.1415926535898;

using namespace Eigen;
using namespace std;

Simulation::Simulation(const SimParameters &params) : params_(params), time_(0)
{
}

Simulation::~Simulation()
{
    for(vector<RigidBodyTemplate *>::iterator it = templates_.begin(); it != templates_.end(); ++it)
    {
        delete *it;
    }
    for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
    {
        delete *it;
    }
}

void Simulation::initializeGL()
{    
}

void Simulation::clearScene()
{
    renderLock_.lock();

    for(vector<RigidBodyTemplate *>::iterator it = templates_.begin(); it != templates_.end(); ++it)
    {
        delete *it;
    }
    for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
    {
        delete *it;
    }

    templates_.clear();
    bodies_.clear();

    // double testRadius = 1.;
    // RigidBodyTemplate *testTemplate = new RigidBodyTemplate(string("resources/sphere.tet"), testRadius);
    // templates_.push_back(testTemplate);

    // Vector3d center(0,0,0);
    // Vector3d orient(0,0,0);
    // double density = 1.;
    // RigidBodyInstance *testInst = new RigidBodyInstance(*testTemplate, center, orient, density);
    // testInst->color = Vector3d(0., 0.6, 1.);
    // bodies_.push_back(testInst);



    //cloth

    VectorXd clothVerts(3*4);
    MatrixX3i clothFaces(2, 3);
    clothFaces.row(0) = Vector3i(0, 1, 2);
    clothFaces.row(1) = Vector3i(1, 2, 3);

    clothVerts.segment<3>(0) = Vector3d(0., 0., 0.);
    clothVerts.segment<3>(3) = Vector3d(1., 0., 0.);
    clothVerts.segment<3>(6) = Vector3d(0., 1., 0.);
    clothVerts.segment<3>(9) = Vector3d(1., 1., 0.);
    
    
    

    ClothTemplate *clothTemplate = new ClothTemplate(clothVerts, clothFaces, 1.);

    ClothInstance* clothInst = new ClothInstance(*clothTemplate, clothVerts);

    cloth_templates_.push_back(clothTemplate);
    cloths_.push_back(clothInst);

    renderLock_.unlock();
}

void Simulation::renderObjects()
{
    if(renderLock_.tryLock())
    {
        for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
        {
            (*it)->render();
        }

        for(vector<ClothInstance *>::iterator it = cloths_.begin(); it != cloths_.end(); ++it)
        {
            (*it)->render();
        }



        renderLock_.unlock();
    }

}


struct comp_imp {
    bool operator() (const Vector2i& a, const Vector2i& b) const {
        return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]);
    }
};


void Simulation::takeSimulationStep()
{   
    time_ += params_.timeStep;

    SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > solver;


    for(vector<ClothInstance *>::iterator it = cloths_.begin(); it != cloths_.end(); ++it)
    {

        ClothInstance *cloth = *it;

        VectorXd F(cloth->x.size());
        VectorXd dFdx(cloth->x.size());
        VectorXd dFdv(cloth->x.size());

        F.setZero();
        dFdx.setZero();
        dFdv.setZero();


        //LHS of delta v update.
        SparseMatrix<double> I(cloth->x.size(), cloth->x.size());
        I.setIdentity();

        SparseMatrix<double> invMass = cloth->getTemplate().getInvMass();

        double h = params_.timeStep;

        SparseMatrix<double> A = I - h * invMass * dFdv - h * h * invMass * dFdx;

        VectorXd b = h * invMass * (F + h * dFdx * cloth->v);

        solver.compute(A);

        VectorXd delta_v = solver.solve(b);

        cloth->x += h * (cloth->v + delta_v);
        cloth->v += delta_v;

    }

    /*VectorXd cForce;
    VectorXd thetaForce;
    computeForces(cForce, thetaForce);

    set<Collision> collisions;
    collisionDetection(bodies_, collisions);


    for(int bodyidx=0; bodyidx < (int)bodies_.size(); bodyidx++)
    {        
        RigidBodyInstance &body = *bodies_[bodyidx];
        Matrix3d Mi = body.getTemplate().getInertiaTensor();

        body.cvel += params_.timeStep*cForce.segment<3>(3*bodyidx)/body.density/body.getTemplate().getVolume();

        Vector3d newwguess(body.w);
        Matrix3d &Roldtheta = Roldthetas[bodyidx];

        int iter = 0;
        for(iter=0; iter<params_.NewtonMaxIters; iter++)
        {
            Matrix3d Dw1 = -VectorMath::TMatrix(params_.timeStep*newwguess).inverse() * VectorMath::TMatrix(-body.theta);
            Matrix3d Dw2 = VectorMath::TMatrix(-params_.timeStep*body.w).inverse() * VectorMath::TMatrix(-body.theta);
            Matrix3d DRw = VectorMath::DrotVector(-body.theta, newwguess);
            Matrix3d Rnewtheta = VectorMath::rotationMatrix(body.theta);
            Vector3d fval = body.density * Dw1.transpose()*Rnewtheta*Mi*Rnewtheta.transpose()*newwguess;
            fval += -params_.timeStep*body.density * DRw.transpose() * Mi * Rnewtheta.transpose() * newwguess;
            fval += body.density * Dw2.transpose() * Roldtheta * Mi * Roldtheta.transpose() * body.w;
            fval += params_.timeStep*thetaForce.segment<3>(3*bodyidx);

            if(fval.norm() / body.density / Mi.trace() <= params_.NewtonTolerance)
                break;

            Matrix3d Df = body.density * Dw1.transpose()*Rnewtheta*Mi*Rnewtheta.transpose();//+ -params_.timeStep*body.density * DRw.transpose() * Mi * Rnewtheta.transpose();

            Vector3d deltaw = Df.inverse() * (-fval);
            newwguess += deltaw;
        }
        cout << "Converged in " << iter << " Newton iterations" << endl;
        body.w = newwguess;

    }*/
}

void Simulation::computeForces(VectorXd &Fc, VectorXd &Ftheta)
{
    Fc.resize(3*bodies_.size());
    Ftheta.resize(3*bodies_.size());
    Fc.setZero();
    Ftheta.setZero();    

    double G = 6.67408e-11;

    if(params_.activeForces & params_.F_GRAVITY)
    {
        for(int i=0; i<(int)bodies_.size(); i++)
        {
            for(int j=i+1; j<(int)bodies_.size(); j++)
            {
                Vector3d diff = bodies_[j]->c - bodies_[i]->c;
                double r = diff.norm();
                double m1 = bodies_[i]->density * bodies_[i]->getTemplate().getVolume();
                double m2 = bodies_[j]->density * bodies_[j]->getTemplate().getVolume();
                Fc.segment<3>(3*i) += G*m1*m2/r/r/r * diff;
                Fc.segment<3>(3*j) -= G*m1*m2/r/r/r * diff;
            }
        }
    }
}

Eigen::Vector3d Simulation::getBodyPosition(int body)
{
    if(body >= 0 && body < (int)bodies_.size())
        return bodies_[body]->c;
    return Vector3d(0,0,0);
}

double Simulation::getBodyBoundingRadius(int body)
{
    if(body >= 0 && body < (int)bodies_.size())
    {
        return bodies_[body]->getTemplate().getBoundingRadius();
    }
    return 1.0;
}
