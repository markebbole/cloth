#include "clothinstance.h"


ClothInstance::ClothInstance(const ClothTemplate &ctemplate, VectorXd x_init)
    : x(x_init), ctemplate_(ctemplate)
{
    
    color = Vector3d(1.0, 1.0, 1.0);
    v.resize(x_init.size());
    v.setZero();
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


void ClothInstance::computeForces(VectorXd& F, SparseMatrix<double> dFdx, vector< Tr >& dFdv) {
    //just gravity for right now.
    SparseMatrix<double> invMass = getTemplate().getInvMass();
    for(int i = 0; i < (int)x.size()/3; ++i) {
        double m = 1. / invMass.coeffRef(3*i, 3*i);
        F(3*i + 1) += -9.8*m;
    }

    //stretching force
    double kStretch = 1000.;
    MatrixX3i triangles = getTemplate().getFaces();
    VectorXd V = getTemplate().getVerts();
    for(int i = 0; i < (int)triangles.rows(); ++i) {

        Vector3i face = triangles.row(i);
        Vector3d p0 = V.segment<3>(3*face[0]);
        Vector3d p1 = V.segment<3>(3*face[1]);
        Vector3d p2 = V.segment<3>(3*face[2]);

        Vector3d cr = (p1-p0).cross(p2-p0);
        double density = getTemplate().getDensity();
        double triArea = density*cr.norm()/2.;
        Vector3d x0 = x.segment<3>(3*face[0]);
        Vector3d x1 = x.segment<3>(3*face[1]);
        Vector3d x2 = x.segment<3>(3*face[2]);

        Vector3d delta_x1 = x.segment<3>(3*face[1]) - x.segment<3>(3*face[0]);
        Vector3d delta_x2 = x.segment<3>(3*face[2]) - x.segment<3>(3*face[0]);

        Vector2d delta_uv1 = V.segment<2>(3*face[1]) - V.segment<2>(3*face[0]);
        Vector2d delta_uv2 = V.segment<2>(3*face[2]) - V.segment<2>(3*face[0]);

        double delta_v1 = delta_uv1(1);
        double delta_v2 = delta_uv2(1);
        double delta_u1 = delta_uv1(0);
        double delta_u2 = delta_uv2(0);


        // cout << "YOOOOOOOO 2 " << endl;

        double uv_denom = delta_u1 * delta_v2 - delta_u2 * delta_v1;
       // Vector3d w_v = delta_x1 * M(1,0) + delta_x2 * M(1,1);
        
        Matrix2d M;
        M << delta_u1, delta_u2, delta_v1, delta_v2;
        M = M.inverse().eval();
        Vector3d w_u = delta_x1 * M(0,0) + delta_x2 * M(1,0);
        Vector3d w_v = delta_x1 * M(0,1) + delta_x2 * M(1,1);


       // Vector3d w_u = ((x1 - x0) * delta_v2 - (x2 - x0) * delta_v1) / uv_denom;
        //Vector3d w_v = -((x1 - x0) * delta_u2 + (x2 - x0) * delta_u1) / uv_denom;

        Vector3d w_u_hat = w_u / w_u.norm();
        Vector3d w_v_hat = w_v / w_v.norm();


        //test w

        
        //cout << "origu: " << w_u << endl << w_u2 << endl;
        //cout << "origv: " << w_v << endl << w_v2 << endl;
        Vector2d C_stretch(triArea * (w_u.norm() - 1), triArea * (w_v.norm() - 1));

        double dw_u_dx0 = (delta_v1 - delta_v2) / uv_denom;
        double dw_u_dx1 = delta_v2 / uv_denom;
        double dw_u_dx2 = -delta_v1 / uv_denom;

        double dw_v_dx0 = (delta_u2 - delta_u1) / uv_denom;
        double dw_v_dx1 = -delta_u2 / uv_denom;
        double dw_v_dx2 = delta_u1 / uv_denom;

        Vector3d dCu_dx0 = triArea * dw_u_dx0 * w_u_hat;
        Vector3d dCu_dx1 = triArea * dw_u_dx1 * w_u_hat;
        Vector3d dCu_dx2 = triArea * dw_u_dx2 * w_u_hat;
        
        // cout << "YOOOOOO 3" << endl;
        Vector3d dCv_dx0 = triArea * dw_v_dx0 * w_v_hat;
        Vector3d dCv_dx1 = triArea * dw_v_dx1 * w_v_hat;
        Vector3d dCv_dx2 = triArea * dw_v_dx2 * w_v_hat;

        MatrixXd dC_dx0(3,2);
        dC_dx0.col(0) = dCu_dx0;
        dC_dx0.col(1) = dCv_dx0;

        MatrixXd dC_dx1(3,2);
        dC_dx1.col(0) = dCu_dx1;
        dC_dx1.col(1) = dCv_dx1;

        MatrixXd dC_dx2(3,2);
        dC_dx2.col(0) = dCu_dx2;
        dC_dx2.col(1) = dCv_dx2;

        //cout << C_stretch << endl;

        Vector3d F0 = -kStretch * dC_dx0 * C_stretch;
        Vector3d F1 = -kStretch * dC_dx1 * C_stretch;
        Vector3d F2 = -kStretch * dC_dx2 * C_stretch;
        F.segment<3>(3*face[0]) += F0;
        F.segment<3>(3*face[1]) += F1;
        F.segment<3>(3*face[2]) += F2;

        /*Matrix3d I;
        I.setIdentity();

        // cout << "YOOOOO1" << endl;

        Matrix3d d2C_u_00 = triArea / w_u.norm() * (dw_u_dx0 * dw_u_dx0) * (I - w_u_hat * w_u_hat.transpose());
        Matrix3d d2C_v_00 = triArea / w_v.norm() * (dw_v_dx0 * dw_v_dx0) * (I - w_v_hat * w_v_hat.transpose());
        
        Matrix3d d2C_u_11 = triArea / w_u.norm() * (dw_u_dx1 * dw_u_dx1) * (I - w_u_hat * w_u_hat.transpose());
        Matrix3d d2C_v_11 = triArea / w_v.norm() * (dw_v_dx1 * dw_v_dx1) * (I - w_v_hat * w_v_hat.transpose());
        
        Matrix3d d2C_u_22 = triArea / w_u.norm() * (dw_u_dx2 * dw_u_dx2) * (I - w_u_hat * w_u_hat.transpose());
        Matrix3d d2C_v_22 = triArea / w_v.norm() * (dw_v_dx2 * dw_v_dx2) * (I - w_v_hat * w_v_hat.transpose());
        

        Matrix3d d2C_u_10 = triArea / w_u.norm() * (dw_u_dx1 * dw_u_dx0) * (I - w_u_hat * w_u_hat.transpose());
        Matrix3d d2C_v_10 = triArea / w_v.norm() * (dw_v_dx1 * dw_v_dx0) * (I - w_v_hat * w_v_hat.transpose());
        
        Matrix3d d2C_u_20 = triArea / w_u.norm() * (dw_u_dx2 * dw_u_dx0) * (I - w_u_hat * w_u_hat.transpose());
        Matrix3d d2C_v_20 = triArea / w_v.norm() * (dw_v_dx2 * dw_v_dx0) * (I - w_v_hat * w_v_hat.transpose());
        
        Matrix3d d2C_u_21 = triArea / w_u.norm() * (dw_u_dx2 * dw_u_dx1) * (I - w_u_hat * w_u_hat.transpose());
        Matrix3d d2C_v_21 = triArea / w_v.norm() * (dw_v_dx2 * dw_v_dx1) * (I - w_v_hat * w_v_hat.transpose());

        // cout << "YOOO 4" << endl;
        
        Matrix3d K_00 = -kStretch * (dC_dx0 * dC_dx0.transpose() + (d2C_u_00 * C_stretch(0) + d2C_v_00 * C_stretch(1)));
        Matrix3d K_11 = -kStretch * (dC_dx1 * dC_dx1.transpose() + (d2C_u_11 * C_stretch(0) + d2C_v_11 * C_stretch(1)));
        Matrix3d K_22 = -kStretch * (dC_dx2 * dC_dx2.transpose() + (d2C_u_22 * C_stretch(0) + d2C_v_22 * C_stretch(1)));

        Matrix3d K_10 = -kStretch * (dC_dx1 * dC_dx0.transpose() + (d2C_u_10 * C_stretch(0) + d2C_v_10 * C_stretch(1)));
        Matrix3d K_20 = -kStretch * (dC_dx2 * dC_dx0.transpose() + (d2C_u_20 * C_stretch(0) + d2C_v_20 * C_stretch(1)));
        Matrix3d K_21 = -kStretch * (dC_dx2 * dC_dx1.transpose() + (d2C_u_21 * C_stretch(0) + d2C_v_21 * C_stretch(1)));
        
        // cout << "YOOO 5" << endl;
        //00
        MatrixXd springDeriv(x.size(), x.size());
        springDeriv.setZero();
        springDeriv.block<3,3>(3*face[0], 3*face[0]) = K_00;
        springDeriv.block<3,3>(3*face[1], 3*face[1]) = K_11;
        springDeriv.block<3,3>(3*face[2], 3*face[2]) = K_22;

        // cout << "YOO 6" << endl;
        springDeriv.block<3,3>(3*face[1], 3*face[0]) = K_10;
        springDeriv.block<3,3>(3*face[2], 3*face[0]) = K_20;
        springDeriv.block<3,3>(3*face[2], 3*face[1]) = K_21;
        springDeriv.block<3,3>(3*face[0], 3*face[0]) = K_10.transpose();
        springDeriv.block<3,3>(3*face[0], 3*face[2]) = K_20.transpose();
        springDeriv.block<3,3>(3*face[1], 3*face[2]) = K_21.transpose();
        // cout << "Yooo 7" << endl;
        dFdx += springDeriv.sparseView();

        // cout << "YOOOO  END" << endl;


        //Vector3d w_v = ()
        //cout << w_u << " " << w_v << endl;
        /*double c0 = w_u.norm() - 1;
        double c1 = w_v.norm() - 1;

        Vector2d C_stretch(triArea*c0, triaArea*c1);

        MatrixXd P(9, 3);

        P.setZero();
        double s1 = -M(0,0) - M(0, 1);
        double s2 = M(0,0);
        double s3 = M(0,1);
        Matrix3d I;
        I.setIdentity();

        P.block<3,3>(0, 0) = s1*I;
        P.block<3,3>(3, 0) = s2*I;
        P.block<3,3>(6, 0) = s3*I;

        VectorXd dC1 = triArea * P * w_u / w_u.norm();

        s1 = -M(1,0) - M(1,1);
        s2 = M(1,0);
        s3 = M(1,1);

        P.block<3,3>(0,0) = s1*I;
        P.block<3,3>(3,0) = s2*I;
        P.block<3,3>(6,0) = s3*I;

        VectorXd dC2 = triArea * P * w_v / w_v.norm();

        VectorXd F_stretch = -kStretch * (dC1 * C_stretch(0) + dC2 * C_stretch(1));
        F.segment<3>(3*face[0]) += F_stretch.segment<3>(0);
        F.segment<3>(3*face[1]) += F_stretch.segment<3>(3);
        F.segment<3>(3*face[2]) += F_stretch.segment<3>(6);
        */
    }

}