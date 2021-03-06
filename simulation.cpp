﻿#include "simulation.h"
#include <QGLWidget>
#include "simparameters.h"
#include <iostream>
#include <Eigen/Geometry>
#include <QDebug>
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
#include <string>
#include "exact-ccd/rootparitycollisiontest.h"
#include <fstream>
#include <iostream>
#include <sstream>
const double PI = 3.1415926535898;

using namespace Eigen;
using namespace std;

Simulation::Simulation(const SimParameters &params) : params_(params), time_(0)
{
}

Simulation::~Simulation()
{
    /*for(vector<RigidBodyTemplate *>::iterator it = templates_.begin(); it != templates_.end(); ++it)
    {
        delete *it;
    }
    for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
    {
        delete *it;
    }*/
}

void Simulation::initializeGL()
{    
}



void LoadObj(const std::string& file, MatrixX3d& verts,
             MatrixX3i& tris) {
  std::ifstream in(file);
  
  Vector3d vertex;
  Vector3i face_indices;
  vector<Vector3d> vertices;
  vector<Vector3i> indices;
  // glm::vec4 vertex = glm::vec4(0.0, 0.0, 0.0, 1.0);
  // glm::uvec3 face_indices = glm::uvec3(0, 0, 0);
  while (in.good()) {
    char c = in.get();
    switch (c) {
      case 'v':
        in >> vertex[0] >> vertex[1] >> vertex[2];
        vertices.push_back(vertex);
        break;
      case 'f':
        in >> face_indices[0] >> face_indices[1] >> face_indices[2];
        face_indices -= Vector3i(1, 1, 1);
        indices.push_back(face_indices);
        break;
      default:
        break;
    }
  }
  in.close();


  verts.resize(vertices.size(), 3);
  for(int i = 0; i < verts.rows(); ++i) {
    verts.row(i) = vertices[i];
  }

  tris.resize(indices.size(), 3);
  for(int i = 0; i < tris.rows(); ++i) {
    tris.row(i) = indices[i];
  }

}



void Simulation::clearScene()
{
    renderLock_.lock();

    for(vector<ClothInstance *>::iterator it = cloths_.begin(); it != cloths_.end(); ++it)
    {
        delete *it;
    }

    for(vector<ClothTemplate *>::iterator it = cloth_templates_.begin(); it != cloth_templates_.end(); ++it)
    {
        delete *it;
    }

    for(vector<Obstacle *>::iterator it = obstacles_.begin(); it != obstacles_.end(); ++it)
    {
        delete *it;
    }

    cloths_.clear();
    cloth_templates_.clear();
    obstacles_.clear();

    //cloth
    int vW = params_.clothWidth;
    int vH = params_.clothWidth;
    double spaceWidth = params_.clothSideLen;
    double spaceHeight = params_.clothSideLen;

    VectorXd clothVerts(3*vW*vH);
    MatrixX3i clothFaces((vW-1)*(vH-1) * 2, 3);

    cout << clothFaces.rows() << endl;

    cout << "what" << endl;
    for(int i = 0; i < vW; ++i) {
        for(int j = 0; j < vH; ++j) {
            Vector3d v(i*spaceWidth, j*spaceHeight,0.);
            clothVerts.segment<3>(3*vH*i + 3*j) = v;
        }
    }
    int z = 0;
    for(int i = 0; i < vW-1; ++i) {
        for(int j = 0; j < vH-1;++j) {
            int v0 = vH*i + j;
            int v1 = vH*i + j+1;
            int v2 = vH*(i+1) + j;
            Vector3i face(v0, v2, v1);
            clothFaces.row(z++) = face;
            v0 = v1;
            v1 = v2;
            v2 = vH*(i+1) + j + 1;
            Vector3i face2(v0, v1, v2);
            clothFaces.row(z++) = face2;
        }
    }

    cout << clothVerts << endl;

    cout << clothFaces << endl;


    /*clothFaces.row(0) = Vector3i(0, 1, 2);
    clothFaces.row(1) = Vector3i(1, 2, 3);

    clothVerts.segment<3>(0) = Vector3d(0., 0., 0.);
    clothVerts.segment<3>(3) = Vector3d(1., 0., 0.);
    clothVerts.segment<3>(6) = Vector3d(0., 1., 0.);
    clothVerts.segment<3>(9) = Vector3d(1., 1., 0.);
    */
    cout << "WHAT" << endl;

    ClothTemplate *clothTemplate = new ClothTemplate(clothVerts, clothFaces, params_.clothDensity);

    ClothInstance* clothInst = new ClothInstance(*clothTemplate, clothVerts, params_);

    cloth_templates_.push_back(clothTemplate);
    cloths_.push_back(clothInst);

    double dx = -1;
    double dy = 1.;
    double dz = -.3;
    for(int i = 0; i < (int)clothInst->x.size()/3; ++i) {
        clothInst->x.segment<3>(3*i) = Vector3d(clothInst->x(3*i) + dx, clothInst->x(3*i+2) + dy, clothInst->x(3*i+1) + dz);
    }


    clothInst->AABB = buildAABB(clothInst);

    //make a floor obstacle
    MatrixX3i tris2(2, 3);

    MatrixX3d verts2(4, 3);

    LoadObj("resources/bunny.obj", verts2, tris2);

    for(int i = 0; i < verts2.rows(); ++i) {
        verts2.row(i) = 6 * verts2.row(i) + RowVector3d(0., -1., 0.);
    }


    MatrixX3i tris(2, 3);

    MatrixX3d verts(4, 3);
    verts.row(0) = Vector3d(-30., -2., -30.);
    verts.row(1) = Vector3d(30., -2., -30.);
    verts.row(2) = Vector3d(30., 0., 30.);
    verts.row(3) = Vector3d(-30., -2., 30.);

    tris.row(0) = Vector3i(0, 1, 2);
    tris.row(1) = Vector3i(0, 3, 2);

    Obstacle* o = new Obstacle(verts2, tris2);
    o->AABB = buildObstacleAABB(o);

    Obstacle* o2 = new Obstacle(verts, tris);
    o2->AABB = buildObstacleAABB(o2);

    obstacles_.push_back(o);
    obstacles_.push_back(o2);

    renderLock_.unlock();
}

void Simulation::renderObjects()
{
    if(renderLock_.tryLock())
    {
        /*for(vector<RigidBodyInstance *>::iterator it = bodies_.begin(); it != bodies_.end(); ++it)
        {
            (*it)->render();
        }*/

        for(vector<ClothInstance *>::iterator it = cloths_.begin(); it != cloths_.end(); ++it)
        {
            (*it)->render();
        }

        for(vector<Obstacle *>::iterator it = obstacles_.begin(); it != obstacles_.end(); ++it)
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



void getRepulsion(Collision& coll, double invMassOfPoint) {

}


void Simulation::takeSimulationStep()
{   
    time_ += params_.timeStep;

    SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > solver;

    for(vector<ClothInstance *>::iterator it = cloths_.begin(); it != cloths_.end(); ++it)
    {

        ClothInstance *cloth = *it;

        VectorXd F_el(cloth->x.size());
        VectorXd F_d(cloth->x.size());
        vector<Tr> dFdxCoeffs;
        vector<Tr> dFdvCoeffs;

        // SparseMatrix<double> dFdx(cloth->x.size(), cloth->x.size());
        // dFdx.setZero();
        // SparseMatrix<double> dFdv(cloth->x.size(), cloth->x.size());
        // dFdv.setZero();




        F_el.setZero();
        F_d.setZero();

        cloth->computeForces(F_el, F_d, dFdxCoeffs, dFdvCoeffs);
        VectorXd F = F_el + F_d;

        SparseMatrix<double> dFdx(cloth->x.size(), cloth->x.size());
        dFdx.setFromTriplets(dFdxCoeffs.begin(), dFdxCoeffs.end());

        SparseMatrix<double> dFdv(cloth->x.size(), cloth->x.size());
        dFdv.setFromTriplets(dFdvCoeffs.begin(), dFdvCoeffs.end());



        double h = params_.timeStep;
        SparseMatrix<double> invMass = cloth->getTemplate().getInvMass();


        //delta v update
        SparseMatrix<double> I(cloth->x.size(), cloth->x.size());
        I.setIdentity();

        //cout << I << endl;

        //SparseMatrix<double> invMass = cloth->getTemplate().getInvMass();

        //double h = params_.timeStep;

        SparseMatrix<double> A = I - h * invMass * dFdv - h * h * invMass * dFdx;

        VectorXd b = h * invMass * F + h * h * invMass * dFdx * cloth->v;

        solver.compute(A);

        VectorXd delta_v = solver.solve(b);

        /*VectorXd oldq = q;
        q += params_.timeStep*v;
        computeForce(q, oldq, F);
        v += params_.timeStep*Minv*F;*/

        // cloth->x += h * cloth->v;
        // cloth->v += h * invMass * F;


        set<Collision> collisions;
        set<Collision> collisions2;
        set<Collision> collisions3;

        VectorXd prevX = cloth->x;
        VectorXd prevV = cloth->v;

        VectorXd candidateX = cloth->x + h * (cloth->v + delta_v);
        VectorXd v_avg = (candidateX - prevX) / params_.timeStep;
        VectorXd candidateV = v_avg;//cloth->v + delta_v;



        
        collisions3.clear();

        double repulsionReduce = .25;

        obstacleCollisions(cloth, obstacles_, collisions3);

        if(collisions3.size() > 0) {
            cout << "OBSTACLE COLL: " << collisions3.size() << endl;
        }


        for(auto it = collisions3.begin(); it != collisions3.end(); ++it) {



            Collision coll = *it;

            if(coll.pointIndex == -1) {
                double magPoint = coll.rel_velocity / 2.;

                double magTri = 2* magPoint / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));


                Vector3i tri = cloth->getTemplate().getFaces().row(coll.triIndex).transpose();
                //double m1, m2, m3;
                double invm1 = invMass.coeffRef(3*tri[0], 3*tri[0]);
                double invm2 = invMass.coeffRef(3*tri[1], 3*tri[1]);
                double invm3 = invMass.coeffRef(3*tri[2], 3*tri[2]);

                if(coll.rel_velocity < 0) {
                    candidateV.segment<3>(3*tri[0]) += .3*repulsionReduce*coll.bary(0) * (magTri * invm1) * coll.n_hat;
                    
                    candidateV.segment<3>(3*tri[1]) += .3*repulsionReduce*coll.bary(1) * (magTri *invm2) * coll.n_hat;
                    candidateV.segment<3>(3*tri[2]) += .3*repulsionReduce*coll.bary(2) * (magTri * invm3) * coll.n_hat;

                }

                Vector3d triPointVel = coll.bary(0) * candidateV.segment<3>(3*tri[0]) 
                    + coll.bary(1) *candidateV.segment<3>(3*tri[1]) 
                    + coll.bary(2) *candidateV.segment<3>(3*tri[2]);

                double rel_velocity = -coll.n_hat.dot(triPointVel);
                

                double d = .1 - (obstacles_[coll.obstacleIndex]->V.row(coll.obstaclePointIndex).transpose() 
                    - coll.bary(0)*cloth->x.segment<3>(3*tri[0])
                    - coll.bary(1)*cloth->x.segment<3>(3*tri[1])
                    - coll.bary(2)*cloth->x.segment<3>(3*tri[2])).dot(coll.n_hat);
                if(rel_velocity < .1*d/params_.timeStep) {
                    double I_r_mag = -min(params_.timeStep * 1000. * d,  (.1*d/params_.timeStep - coll.rel_velocity));
                    double I_r_mag_interp = 2*I_r_mag / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));
                    candidateV.segment<3>(3*tri[0]) += repulsionReduce * coll.bary(0) * (I_r_mag_interp * invm1) * coll.n_hat;
                    candidateV.segment<3>(3*tri[1]) += repulsionReduce * coll.bary(1) * (I_r_mag_interp * invm2) * coll.n_hat;
                    candidateV.segment<3>(3*tri[2]) += repulsionReduce * coll.bary(2) * (I_r_mag_interp * invm3) * coll.n_hat;
                }

            } else {
                double invMPoint = invMass.coeffRef(3*coll.pointIndex, 3*coll.pointIndex);
                double magPoint = 0.;
                double m = 1.;
                if(invMPoint > 0) {
                    m = 1. / invMPoint;
                    magPoint = m * coll.rel_velocity / 2.;

                }

                double magTri = 2* magPoint / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));

                Vector3i tri = obstacles_[coll.obstacleIndex]->F.row(coll.triIndex).transpose();
                //double m1, m2, m3;
              
                if(coll.rel_velocity < 0) {

                    candidateV.segment<3>(3*coll.pointIndex) -= .3*repulsionReduce* (magTri / m) * coll.n_hat;
                    
                }

                //check rel velocity again

                double rel_velocity = coll.n_hat.dot(candidateV.segment<3>(3*coll.pointIndex));
                

                double d = .1 - (cloth->x.segment<3>(3*coll.pointIndex) 
                    - coll.bary(0)*obstacles_[coll.obstacleIndex]->V.row(tri[0]).transpose()
                    - coll.bary(1)*obstacles_[coll.obstacleIndex]->V.row(tri[1]).transpose()
                    - coll.bary(2)*obstacles_[coll.obstacleIndex]->V.row(tri[2]).transpose()).dot(coll.n_hat);
                if(rel_velocity < .1*d/params_.timeStep) {
                    double I_r_mag = -min(params_.timeStep * 1000. * d, m * (.1*d/params_.timeStep - coll.rel_velocity));
                    double I_r_mag_interp = 2*I_r_mag / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));

                    candidateV.segment<3>(3*coll.pointIndex) -= repulsionReduce * (I_r_mag_interp / m) * coll.n_hat;
                }
            }

            
            
        }


        collisions.clear();


        if(params_.activeForces & SimParameters::F_COLLISION_REPULSION) {
            selfCollisions(cloth,  collisions);
        }


       

        for(auto it = collisions.begin(); it != collisions.end(); ++it) {
            Collision coll = *it;
            double invMPoint = invMass.coeffRef(3*coll.pointIndex, 3*coll.pointIndex);
            double magPoint = 0.;
            Vector3d n_hat = coll.n_hat;
            double m = 1.;
            if(invMPoint > 0) {
                m = 1. / invMPoint;
                magPoint = m * coll.rel_velocity / 2.;

            }

            double magTri = 2* magPoint / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));


            Vector3i tri = cloth->getTemplate().getFaces().row(coll.triIndex);
            //double m1, m2, m3;
            double invm1 = invMass.coeffRef(3*tri[0], 3*tri[0]);
            double invm2 = invMass.coeffRef(3*tri[1], 3*tri[1]);
            double invm3 = invMass.coeffRef(3*tri[2], 3*tri[2]);

            if(coll.rel_velocity < 0) {
                candidateV.segment<3>(3*tri[0]) += repulsionReduce*coll.bary(0) * (magTri * invm1) * n_hat;
                
                candidateV.segment<3>(3*tri[1]) += repulsionReduce*coll.bary(1) * (magTri *invm2) * n_hat;
                candidateV.segment<3>(3*tri[2]) += repulsionReduce*coll.bary(2) * (magTri * invm3) * n_hat;

                candidateV.segment<3>(3*coll.pointIndex) -= repulsionReduce* (magTri / m) * n_hat;
                
            }

            //check rel velocity again

            Vector3d triPointVel = coll.bary(0) * candidateV.segment<3>(3*tri[0]) 
                + coll.bary(1) *candidateV.segment<3>(3*tri[1]) 
                + coll.bary(2) *candidateV.segment<3>(3*tri[2]);

               //cout << "IN REGION. NOW CHECK REL_VELOCITY" << endl;
               //double rel_velocity = n_hat.dot(triPointVel + pointVelocity);
            double rel_velocity = n_hat.dot(candidateV.segment<3>(3*coll.pointIndex)) - n_hat.dot(triPointVel);
           

            double d = .1 - (cloth->x.segment<3>(3*coll.pointIndex) 
                - coll.bary(0)*cloth->x.segment<3>(3*tri[0])
                - coll.bary(1)*cloth->x.segment<3>(3*tri[1])
                - coll.bary(2)*cloth->x.segment<3>(3*tri[2])).dot(n_hat);
            if(rel_velocity < .1*d/params_.timeStep) {
                double I_r_mag = -min(params_.timeStep * 1000. * d, m * (.1*d/params_.timeStep - coll.rel_velocity));
                double I_r_mag_interp = 2*I_r_mag / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));
                candidateV.segment<3>(3*tri[0]) += repulsionReduce * coll.bary(0) * (I_r_mag_interp * invm1) * n_hat;
                candidateV.segment<3>(3*tri[1]) += repulsionReduce * coll.bary(1) * (I_r_mag_interp * invm2) * n_hat;
                candidateV.segment<3>(3*tri[2]) += repulsionReduce * coll.bary(2) * (I_r_mag_interp * invm3) * n_hat;
                candidateV.segment<3>(3*coll.pointIndex) -= repulsionReduce * (I_r_mag_interp / m) * n_hat;
            }
            
        }






        VectorXd testNewX = prevX + h * candidateV;

        int iter = 0;
        do {
            selfCollisionsCT(cloth, testNewX, candidateV, collisions2);

            if(collisions2.size() > 0) {
                cout << "CTCD > 0: " << collisions2.size() << endl;
            }



            for(auto it = collisions2.begin(); it != collisions2.end(); ++it) {
                Collision coll = *it;
                double invMPoint = invMass.coeffRef(3*coll.pointIndex, 3*coll.pointIndex);
                double magPoint = 0.;
                Vector3d n_hat = coll.n_hat;
                double m = 1.;
                if(invMPoint > 0) {
                    m = 1. / invMPoint;
                    magPoint = m * coll.rel_velocity / 2.;

                }


                double magTri = 2* magPoint / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));


                Vector3i tri = cloth->getTemplate().getFaces().row(coll.triIndex);
                //double m1, m2, m3;
                double invm1 = invMass.coeffRef(3*tri[0], 3*tri[0]);
                double invm2 = invMass.coeffRef(3*tri[1], 3*tri[1]);
                double invm3 = invMass.coeffRef(3*tri[2], 3*tri[2]);

 
                candidateV.segment<3>(3*tri[0]) += coll.bary(0) * (magTri * invm1) * n_hat;
                
                candidateV.segment<3>(3*tri[1]) += coll.bary(1) * (magTri *invm2) * n_hat;
                candidateV.segment<3>(3*tri[2]) += coll.bary(2) * (magTri * invm3) * n_hat;

                candidateV.segment<3>(3*coll.pointIndex) -=  (magTri / m) * n_hat;


                //check rel velocity again

                // Vector3d triPointVel = coll.bary(0) * candidateV.segment<3>(3*tri[0]) 
                //     + coll.bary(1) *candidateV.segment<3>(3*tri[1]) 
                //     + coll.bary(2) *candidateV.segment<3>(3*tri[2]);

                //    //cout << "IN REGION. NOW CHECK REL_VELOCITY" << endl;
                //    //double rel_velocity = n_hat.dot(triPointVel + pointVelocity);
                // double rel_velocity = n_hat.dot(candidateV.segment<3>(3*coll.pointIndex)) - n_hat.dot(triPointVel);
               

                // double d = .1 - (cloth->x.segment<3>(3*coll.pointIndex) 
                //     - coll.bary(0)*cloth->x.segment<3>(3*tri[ 0])
                //     - coll.bary(1)*cloth->x.segment<3>(3*tri[1])
                //     - coll.bary(2)*cloth->x.segment<3>(3*tri[2])).dot(n_hat);
                // if(rel_velocity < .1*d/params_.timeStep) {
                //     double I_r_mag = -min(params_.timeStep * 1000. * d, m * (.1*d/params_.timeStep - coll.rel_velocity));
                //     double I_r_mag_interp = 2*I_r_mag / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));
                //     candidateV.segment<3>(3*tri[0]) += repulsionReduce * coll.bary(0) * (I_r_mag_interp * invm1) * n_hat;
                //     candidateV.segment<3>(3*tri[1]) += repulsionReduce * coll.bary(1) * (I_r_mag_interp * invm2) * n_hat;
                //     candidateV.segment<3>(3*tri[2]) += repulsionReduce * coll.bary(2) * (I_r_mag_interp * invm3) * n_hat;
                //     candidateV.segment<3>(3*coll.pointIndex) -= repulsionReduce * (I_r_mag_interp / m) * n_hat;
                // }
                
            }


            iter++;

        } while(collisions2.size() > 0 && iter < 5);









        // if(params_.activeForces & SimParameters::F_COLLISION_REPULSION) {
        //     selfCollisions(cloth, collisions);


        // }
        

        // VectorXd prevX = cloth->x;
        // VectorXd prevV = cloth->v;

        // VectorXd candidateX = cloth->x + h * (cloth->v + delta_v);

        // VectorXd v_avg = (candidateX - prevX) / params_.timeStep;
        // VectorXd candidateV = v_avg;//cloth->v + delta_v;



        // double repulsionReduce = .25;

        // for(auto it = collisions.begin(); it != collisions.end(); ++it) {
        //     Collision coll = *it;
        //     double invMPoint = invMass.coeffRef(3*coll.pointIndex, 3*coll.pointIndex);
        //     double magPoint = 0.;
        //     Vector3d n_hat = coll.n_hat;
        //     double m = 1.;
        //     if(invMPoint > 0) {
        //         m = 1. / invMPoint;
        //         magPoint = m * coll.rel_velocity / 2.;

        //     }

        //     double magTri = 2* magPoint / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));


        //     Vector3i tri = cloth->getTemplate().getFaces().row(coll.triIndex);
        //     //double m1, m2, m3;
        //     double invm1 = invMass.coeffRef(3*tri[0], 3*tri[0]);
        //     double invm2 = invMass.coeffRef(3*tri[1], 3*tri[1]);
        //     double invm3 = invMass.coeffRef(3*tri[2], 3*tri[2]);

        //     if(coll.rel_velocity < 0) {
        //         candidateV.segment<3>(3*tri[0]) += repulsionReduce*coll.bary(0) * (magTri * invm1) * n_hat;
                
        //         candidateV.segment<3>(3*tri[1]) += repulsionReduce*coll.bary(1) * (magTri *invm2) * n_hat;
        //         candidateV.segment<3>(3*tri[2]) += repulsionReduce*coll.bary(2) * (magTri * invm3) * n_hat;

        //         candidateV.segment<3>(3*coll.pointIndex) -= repulsionReduce* (magTri / m) * n_hat;
                
        //     }


        //     //check rel velocity again

        //     Vector3d triPointVel = coll.bary(0) * candidateV.segment<3>(3*tri[0]) 
        //         + coll.bary(1) *candidateV.segment<3>(3*tri[1]) 
        //         + coll.bary(2) *candidateV.segment<3>(3*tri[2]);

        //        //cout << "IN REGION. NOW CHECK REL_VELOCITY" << endl;
        //        //double rel_velocity = n_hat.dot(triPointVel + pointVelocity);
        //     double rel_velocity = n_hat.dot(candidateV.segment<3>(3*coll.pointIndex)) - n_hat.dot(triPointVel);
           

        //     double d = .1 - (cloth->x.segment<3>(3*coll.pointIndex) 
        //         - coll.bary(0)*cloth->x.segment<3>(3*tri[0])
        //         - coll.bary(1)*cloth->x.segment<3>(3*tri[1])
        //         - coll.bary(2)*cloth->x.segment<3>(3*tri[2])).dot(n_hat);
        //     if(rel_velocity < .1*d/params_.timeStep) {
        //         double I_r_mag = -min(params_.timeStep * 1000. * d, m * (.1*d/params_.timeStep - coll.rel_velocity));
        //         double I_r_mag_interp = 2*I_r_mag / (1 + coll.bary(0)*coll.bary(0) + coll.bary(1)*coll.bary(1) + coll.bary(2)*coll.bary(2));
        //         candidateV.segment<3>(3*tri[0]) += repulsionReduce * coll.bary(0) * (I_r_mag_interp * invm1) * n_hat;
        //         candidateV.segment<3>(3*tri[1]) += repulsionReduce * coll.bary(1) * (I_r_mag_interp * invm2) * n_hat;
        //         candidateV.segment<3>(3*tri[2]) += repulsionReduce * coll.bary(2) * (I_r_mag_interp * invm3) * n_hat;
        //         candidateV.segment<3>(3*coll.pointIndex) -= repulsionReduce * (I_r_mag_interp / m) * n_hat;
        //     }
            
        // }

         //candidateV is v^(i+1/2)


        

       
        cloth->x = prevX + params_.timeStep * candidateV;

        
        if(collisions.size() == 0 && collisions2.size() == 0 && collisions3.size() == 0) {
            cloth->v = cloth->v + delta_v;
        } else {

            cloth->v = candidateV;

            F_el.setZero();
            F_d.setZero();
            // dFdx.setZero();
            // dFdv.setZero();
            dFdxCoeffs.clear();
            dFdvCoeffs.clear();

            cloth->computeForces(F_el, F_d, dFdxCoeffs, dFdvCoeffs);

            dFdv.setFromTriplets(dFdvCoeffs.begin(), dFdvCoeffs.end());

            

            SparseMatrix<double> L = I - h/2. * invMass * dFdv;

            VectorXd RHS = candidateV + h/2. * invMass * F_el;

            solver.compute(L);

            VectorXd vn_plus_1 = solver.solve(RHS);

            cloth->v = vn_plus_1;
        }
        

        

        // cout << "CLOTH V AFTER APPLYING UPDATE: " << endl;
        // cout << cloth->v << endl;

        // cloth->x += h * (cloth->v + delta_v);
        // cloth->v += delta_v;

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

/*void Simulation::computeForces(VectorXd &F, VectorXd& dFdx, VectorXd& dFdv) 
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
}*/