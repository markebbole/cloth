#include "obstacle.h"

#include <QGLWidget>
#include "distance.h"
#include "vectormath.h"
void Obstacle::render()
{

    glShadeModel(GL_FLAT);
    glEnable(GL_LIGHTING);
    glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
    glEnable ( GL_COLOR_MATERIAL );
    glEnable( GL_NORMALIZE);
    glColor4d(1., 1., 1., 1.0);

    glPushMatrix();
    {

        glBegin(GL_TRIANGLES);
        const Eigen::MatrixX3i &faces =  F;
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

