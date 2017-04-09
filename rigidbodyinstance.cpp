#include "rigidbodyinstance.h"
#include "vectormath.h"
#include <QGLWidget>
#include "rigidbodytemplate.h"
#include <Eigen/Geometry>
#include <iostream>
#include "collisiondetection.h"

using namespace Eigen;
using namespace std;

RigidBodyInstance::RigidBodyInstance(const RigidBodyTemplate &rbtemplate,
                                     const Eigen::Vector3d &c, const Eigen::Vector3d &theta,
                                     double density)
    : c(c), theta(theta), density(density), rbtemplate_(rbtemplate)
{
    cvel.setZero();
    w.setZero();
    color = Vector3d(1.0, 1.0, 1.0);
    AABB = buildAABB(this);
    time = 0;
}

RigidBodyInstance::~RigidBodyInstance()
{
    delete AABB;
}

void RigidBodyInstance::render()
{
    Matrix3d rot = VectorMath::rotationMatrix(theta);

    glShadeModel(GL_FLAT);
    glEnable(GL_LIGHTING);
    glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    glEnable ( GL_COLOR_MATERIAL );
    glEnable( GL_NORMALIZE);
    glColor4d(color[0], color[1], color[2], 1.0);

    glPushMatrix();
    {
        GLdouble xform[16];
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
                xform[4*j+i] = rot.coeff(i,j);
            xform[4*i+3] = 0;
            xform[12+i] = c[i];
        }
        xform[15] = 1.0;
        glMultMatrixd(xform);
        glScaled(1,1,1);

        glBegin(GL_TRIANGLES);
        const Eigen::MatrixX3i &faces = rbtemplate_.getFaces();
        const Eigen::MatrixX3d &V = rbtemplate_.getVerts();
        int nfaces = faces.rows();
        for(int i=0; i<nfaces; i++)
        {
            Vector3i face = faces.row(i);
            Vector3d normal = (V.row(face[1]) - V.row(face[0])).cross(V.row(face[2])-V.row(face[0]));
            normal /= normal.norm();

            for(int j=0; j<3; j++)
            {                
                //glColor4d(normal[0]*normal[0], normal[1]*normal[1], normal[2]*normal[2],1.0);
                glNormal3d(normal[0], normal[1], normal[2]);
                glVertex3d(V(face[j], 0), V(face[j],1), V(face[j], 2));
            }
        }
        glEnd();
    }
    glPopMatrix();
}