#include "clothinstance.h"
#include "vectormath.h"
#include <QGLWidget>
#include "clothtemplate.h"
#include <Eigen/Geometry>
#include <iostream>
#include "collisiondetection.h"

using namespace Eigen;
using namespace std;

ClothInstance::ClothInstance(const ClothTemplate &ctemplate, VectorXd x_init)
    : x(x_init), ctemplate_(ctemplate)
{
    
    color = Vector3d(1.0, 1.0, 1.0);
    //AABB = buildAABB(this);
    time = 0;
}

ClothInstance::~ClothInstance()
{
    //delete AABB;
}

void ClothInstance::render()
{
    
    glShadeModel(GL_FLAT);
    glEnable(GL_LIGHTING);
    glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    glEnable ( GL_COLOR_MATERIAL );
    glEnable( GL_NORMALIZE);
    glColor4d(color[0], color[1], color[2], 1.0);

    glPushMatrix();
    {
        /*GLdouble xform[16];
        for(int i=0; i<3; i++)
        {
            for(int j=0; j<3; j++)
                xform[4*j+i] = rot.coeff(i,j);
            xform[4*i+3] = 0;
            xform[12+i] = c[i];
        }
        xform[15] = 1.0;
        glMultMatrixd(xform);
        glScaled(1,1,1);*/

        glBegin(GL_TRIANGLES);
        const Eigen::MatrixX3i &faces = ctemplate_.getFaces();
        
        int nfaces = faces.rows();
        for(int i=0; i<nfaces; i++)
        {
            Vector3i face = faces.row(i);
            Vector3d p0 = x.segment<3>(3*face[0]);
            Vector3d p1 = x.segment<3>(3*face[1]);
            Vector3d p2 = x.segment<3>(3*face[2]);

            Vector3d normal = (p1 - p0).cross(p2-p1);
            normal /= normal.norm();

            //for(int j=0; j<3; j++)
            //{                
                //glColor4d(normal[0]*normal[0], normal[1]*normal[1], normal[2]*normal[2],1.0);
            glNormal3d(normal[0], normal[1], normal[2]);
            glVertex3d(p0[0], p0[1], p0[2]);
            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);
            //}
        }
        glEnd();
    }
    glPopMatrix();
}