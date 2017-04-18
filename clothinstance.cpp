#include "clothinstance.h"


ClothInstance::ClothInstance(const ClothTemplate &ctemplate, VectorXd x_init)
    : x(x_init), ctemplate_(ctemplate)
{
    
    color = Vector3d(1.0, 1.0, 1.0);
    v.resize(x_init.size());
    v.setZero();
    AABB = buildAABB(this);
    time = 0;
}

ClothInstance::~ClothInstance()
{
    delete AABB;
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

        AABBNode* aabb = AABB;

        //do {
            BBox b = aabb->box;

            glLineWidth(1.);
            glColor3f(1., 1., 0.);
            glBegin(GL_LINES);
            glVertex3d(b.mins[0], b.mins[1], b.mins[2]);
            glVertex3d(b.maxs[0], b.maxs[1], b.maxs[2]);

            glEnd();


        //} while(aabb->childTriangle != -1);

        
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
            glColor3f(1., 1., 1.);
            glBegin(GL_TRIANGLES);
            glNormal3d(normal[0], normal[1], normal[2]);
            glVertex3d(p0[0], p0[1], p0[2]);
            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();

            glLineWidth(1.5);
            glColor3f(1., 0., 0.);
            glBegin(GL_LINES);
            glVertex3d(p0[0], p0[1], p0[2]);
            glVertex3d(p1[0], p1[1], p1[2]);

            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);

            glEnd();

            //}
        }
        
    }
    glPopMatrix();
}


void ClothInstance::computeShearForce(VectorXd& F, SparseMatrix<double> dFdx, vector < Tr > & dFdv) {

}



void ClothInstance::computeForces(VectorXd& F_el, VectorXd& F_d, SparseMatrix<double>& dFdx, SparseMatrix<double>& dFdv) {
    //just gravity for right now.
    SparseMatrix<double> invMass = getTemplate().getInvMass();
    for(int i = 0; i < (int)x.size()/3; ++i) {
        double invM = invMass.coeffRef(3*i, 3*i);
        if(invM > 0) {
            double m = 1. / invMass.coeffRef(3*i, 3*i);
            F_el(3*i + 1) += -9.8*m;
        }
       
    }

    //stretching force
    double kStretch = 1000.;
    double kShear = 100.;

    double kBend = 0.000003;
    double kDampStretch = 1;
    MatrixX3i triangles = getTemplate().getFaces();
    VectorXd V = getTemplate().getVerts();
    Matrix3d I;
    I.setIdentity();
    for(int i = 0; i < (int)triangles.rows(); ++i) {

        Vector3i face = triangles.row(i);
        Vector3d p0 = V.segment<3>(3*face[0]);
        Vector3d p1 = V.segment<3>(3*face[1]);
        Vector3d p2 = V.segment<3>(3*face[2]);

        Vector3d cr = (p1-p0).cross(p2-p0);
        double density = getTemplate().getDensity();
        double triArea = abs(cr.norm()/2.);
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

        Vector3d w_u_hat = w_u / w_u.norm();
        Vector3d w_v_hat = w_v / w_v.norm();
        
        Vector2d C_stretch(triArea * (w_u.norm() - 1), triArea * (w_v.norm() - 1));

        double dw_u_dx[3] = {
            (delta_v1 - delta_v2) / uv_denom,
            delta_v2 / uv_denom,
            -delta_v1 / uv_denom
        };

        double dw_v_dx[3] = {
            (delta_u2 - delta_u1) / uv_denom,
            -delta_u2 / uv_denom,
            delta_u1 / uv_denom,
        };

        Vector3d dCu_dx[3];
        Vector3d dCv_dx[3];
        for(int z = 0; z < 3; ++z) {
            dCu_dx[z] = triArea * dw_u_dx[z] * w_u_hat;
            dCv_dx[z] = triArea * dw_v_dx[z] * w_v_hat;
        }


        MatrixXd dC_dx[3];
        for(int z = 0; z < 3; ++z) {
            dC_dx[z].resize(3,2);
            dC_dx[z].col(0) = dCu_dx[z];
            dC_dx[z].col(1) = dCv_dx[z];
        }


        Vector3d F0 = -kStretch * dC_dx[0] * C_stretch;
        Vector3d F1 = -kStretch * dC_dx[1] * C_stretch;
        Vector3d F2 = -kStretch * dC_dx[2] * C_stretch;
        F_el.segment<3>(3*face[0]) += F0;
        F_el.segment<3>(3*face[1]) += F1;
        F_el.segment<3>(3*face[2]) += F2;


        Vector2d C_stretch_dot = dC_dx[0].transpose() * v.segment<3>(3*face[0]) 
            + dC_dx[1].transpose() * v.segment<3>(3*face[1]) + dC_dx[2].transpose() * v.segment<3>(3*face[2]);

        Vector3d F_damp0 = -kDampStretch * dC_dx[0] * C_stretch_dot;
        Vector3d F_damp1 = -kDampStretch * dC_dx[1] * C_stretch_dot;
        Vector3d F_damp2 = -kDampStretch * dC_dx[2] * C_stretch_dot;
        
        F_d.segment<3>(3*face[0]) += F_damp0;
        F_d.segment<3>(3*face[1]) += F_damp1;
        F_d.segment<3>(3*face[2]) += F_damp2;

        cout << "F_D AFTER STRETCH: " << endl;
        cout << F_d << endl;


        //dFdx damping



        double C_shear = triArea * w_u.transpose() * w_v;

        Vector3d dC_shear_dx[3];
        for(int z = 0; z < 3; ++z) {
            dC_shear_dx[z] = triArea * (dw_u_dx[z] * w_v + w_u * dw_v_dx[z]);
        }

        double C_shear_dot = 0.;
        for(int z = 0; z < 3; ++z) {
            C_shear_dot += dC_shear_dx[z].dot(v.segment<3>(3*face[z]));
        }

        F_el.segment<3>(3*face[0]) += -kShear * dC_shear_dx[0] * C_shear;
        F_el.segment<3>(3*face[1]) += -kShear * dC_shear_dx[1] * C_shear;
        F_el.segment<3>(3*face[2]) += -kShear * dC_shear_dx[2] * C_shear;

        F_d.segment<3>(3*face[0]) += -kDampStretch * dC_shear_dx[0] * C_shear_dot;
        F_d.segment<3>(3*face[1]) += -kDampStretch * dC_shear_dx[1] * C_shear_dot;
        F_d.segment<3>(3*face[2]) += -kDampStretch * dC_shear_dx[2] * C_shear_dot;

        cout << "F_D AFTER SHEAR: " << endl;
        cout << F_d << endl;

        Matrix3d I;
        I.setIdentity();

        MatrixXd shearDeriv(x.size(), x.size());
        shearDeriv.setZero();

        Matrix3d K_shear[3][3];

        MatrixXd shearDampDerivDX(x.size(), x.size());
        shearDampDerivDX.setZero();
        MatrixXd shearDampDerivDV(x.size(), x.size());
        shearDampDerivDV.setZero();
        for(int z=0; z<3;++z) {
            for(int zz=0; zz<3; ++zz) {
                double c = triArea * (dw_u_dx[z] * dw_v_dx[zz] + dw_u_dx[zz] * dw_v_dx[z]);
                Matrix3d C = c*I;

                K_shear[z][zz] = -kShear * (dC_shear_dx[z] * dC_shear_dx[zz].transpose() + C * C_shear);
                shearDeriv.block<3,3>(3*face[z], 3*face[zz]) = K_shear[z][zz];

                shearDampDerivDX.block<3,3>(3*face[z], 3*face[zz]) = -kDampStretch*C*C_shear_dot;
                shearDampDerivDV.block<3,3>(3*face[z], 3*face[zz]) = -kDampStretch*dC_shear_dx[z]*dC_shear_dx[zz].transpose();

            }
        }

        dFdx += shearDeriv.sparseView();
        dFdx += shearDampDerivDX.sparseView();
        dFdv += shearDampDerivDV.sparseView();

        //computeShearForce(F, dFdx, dFdv);

        // cout << "YOOOOO1" << endl;
        Matrix3d d2C_u[3][3];
        Matrix3d d2C_v[3][3];

        for(int z = 0; z < 3; ++z) {
            for(int zz = 0; zz < 3; ++zz) {
                d2C_u[z][zz] = triArea / w_u.norm() * (dw_u_dx[z] * dw_u_dx[zz]) * (I - w_u_hat * w_u_hat.transpose());
                d2C_v[z][zz] = triArea / w_v.norm() * (dw_v_dx[z] * dw_v_dx[zz]) * (I - w_v_hat * w_v_hat.transpose());
        
            }
        }

        // cout << "YOOO 4" << endl;
        Matrix3d K_stretch[3][3];

        for(int z = 0; z < 3; ++z) {
            for(int zz = 0; zz < 3; ++zz) {
                K_stretch[z][zz] = -kStretch * (dC_dx[z] * dC_dx[zz].transpose() + (d2C_u[z][zz] * C_stretch(0) + d2C_v[z][zz] * C_stretch(1)));

            }
        }
        
        //00
        MatrixXd springDeriv(x.size(), x.size());
        springDeriv.setZero();

        MatrixXd springDampDeriv(x.size(), x.size());
        MatrixXd springDampDerivDV(x.size(), x.size());
        for(int z=0; z < 3; ++z) {
            for(int zz=0; zz<3; ++zz) {
                springDeriv.block<3,3>(3*face[z], 3*face[zz]) = K_stretch[z][zz];
                springDampDeriv.block<3,3>(3*face[z], 3*face[zz]) = -kDampStretch * (d2C_u[z][zz] * C_stretch_dot(0) + d2C_v[z][zz]*C_stretch_dot(1));
                springDampDerivDV.block<3,3>(3*face[z], 3*face[zz]) = -kDampStretch * (dC_dx[z]*dC_dx[zz].transpose());
            }
        }

        dFdx += springDeriv.sparseView();
        dFdx += springDampDeriv.sparseView();
        dFdv += springDampDerivDV.sparseView();


    }
    vector< Vector4i > adjacentFaces = getTemplate().getAdjacentFaces();

    for(int i = 0; i < (int)adjacentFaces.size(); ++i) {
        Vector4i faceinfo = adjacentFaces[i];
        Vector3d x0;
        Vector3d x3;
        Vector3d x1 = V.segment<3>(3*faceinfo[0]);
        Vector3d x2 = V.segment<3>(3*faceinfo[1]);

        int p0, p1, p2, p3;

        p1 = faceinfo[0];
        p2 = faceinfo[1];

        Vector3i face1 = triangles.row(faceinfo[2]);
        Vector3i face2 = triangles.row(faceinfo[3]);
        if(face1[0] != faceinfo[0] && face1[0] != faceinfo[1]) {
            x0 = V.segment<3>(3*face1[0]);
            p0 = face1[0];
        } else if(face1[1] != faceinfo[0] && face1[1] != faceinfo[1]) {
            x0 = V.segment<3>(3*face1[1]);
            p0 = face1[1];
        } else if(face1[2] != faceinfo[0] && face1[2] != faceinfo[1]) {
            x0 = V.segment<3>(3*face1[2]);
            p0 = face1[2];
        }

        if(face2[0] != faceinfo[0] && face2[0] != faceinfo[1]) {
            x3 = V.segment<3>(3*face2[0]);
            p3 = face2[0];
        } else if(face2[1] != faceinfo[0] && face2[1] != faceinfo[1]) {
            x3 = V.segment<3>(3*face2[1]);
            p3 = face2[1];
        } else if(face2[2] != faceinfo[0] && face2[2] != faceinfo[1]) {
            x3 = V.segment<3>(3*face2[2]);
            p3 = face2[2];
        }

        Vector3d n_a = (x2-x0).cross(x1-x0);
        Vector3d n_b = (x1-x3).cross(x2-x3);

        Vector3d e = x1-x2;

        Vector3d n_a_hat = n_a / n_a.norm();
        Vector3d n_b_hat = n_b / n_b.norm();
        Vector3d e_hat = e / e.norm();

        double cos_theta = n_a_hat.dot(n_b_hat);
        double sin_theta = n_a_hat.cross(n_b_hat).dot(e_hat);

        double C_bend = atan2(sin_theta, cos_theta);

        Vector3d q_a[4] = {x2-x1, x0-x2, x1-x0, Vector3d(0.,0.,0.)};

        Vector3d q_b[4] = {Vector3d(0.,0.,0.), x2-x3, x3-x1, x1-x2};


        Vector4d q_e(0,1,-1,0);

        //eq 31
        Vector3d dna_dx[4][3];
        for(int z = 0; z < 4; ++z) {
            Matrix3d cp = VectorMath::crossProductMatrix(q_a[z]);
            for(int zz = 0; zz < 3; ++zz) {
                dna_dx[z][zz] = cp.row(zz);
            }
        }
        //eq 32
        Vector3d dnb_dx[4][3];
        for(int z = 0; z < 4; ++z) {
            Matrix3d cp = VectorMath::crossProductMatrix(q_b[z]);
            for(int zz = 0; zz < 3; ++zz) {
                dnb_dx[z][zz] = cp.row(zz);
            }
        }

        //eq 33
        Vector3d de_dx[4][3];
        for(int z = 0; z < 4; ++z) {
            for(int zz = 0; zz < 3; ++zz) {
                de_dx[z][zz] = q_e[z] * I.col(zz);
            }
        }

        double n_a_norm = n_a.norm();
        double n_b_norm = n_b.norm();
        double e_norm = e.norm();

        double dcos_theta_dx[4][3];


        for(int z = 0; z < 4; ++z) {
            for(int zz = 0; zz < 3; ++zz) {
                dcos_theta_dx[z][zz] = (1./n_a_norm) * dna_dx[z][zz].dot(n_b_hat) + (1./n_b_norm) * n_a_hat.dot(dnb_dx[z][zz]);

            }
        }

        double dsin_theta_dx[4][3];
        for(int z = 0; z < 4; ++z) {
           
            for(int zz = 0; zz < 3; ++zz) {
                dsin_theta_dx[z][zz] = (((1./n_a_norm) * dna_dx[z][zz]).cross(n_b_hat) + n_a_hat.cross((1./n_b_norm) * dnb_dx[z][zz])).dot(e_hat) +
                    (n_a_hat.cross(n_b_hat)).dot((1./e_norm) * de_dx[z][zz]);
            }
        }
        
        double dC_bend_dx[4][3];

        for(int z = 0; z < 4; ++z) {
            for(int zz = 0; zz < 3; ++zz) {
                dC_bend_dx[z][zz] = cos_theta * dsin_theta_dx[z][zz] - sin_theta * dcos_theta_dx[z][zz];
            }
        }
        
        Vector3d dC_bend_0(dC_bend_dx[0][0], dC_bend_dx[0][1], dC_bend_dx[0][2]);
        Vector3d dC_bend_1(dC_bend_dx[1][0], dC_bend_dx[1][1], dC_bend_dx[1][2]);
        Vector3d dC_bend_2(dC_bend_dx[2][0], dC_bend_dx[2][1], dC_bend_dx[2][2]);
        Vector3d dC_bend_3(dC_bend_dx[3][0], dC_bend_dx[3][1], dC_bend_dx[3][2]);

        Vector3d dC_bend[4] = {dC_bend_0, dC_bend_1, dC_bend_2, dC_bend_3};

        Vector3d F_0 = -kBend * dC_bend_0 * C_bend;
        Vector3d F_1 = -kBend * dC_bend_1 * C_bend;
        Vector3d F_2 = -kBend * dC_bend_2 * C_bend;
        Vector3d F_3 = -kBend * dC_bend_3 * C_bend;

        F_el.segment<3>(3*p0) += F_0;
        F_el.segment<3>(3*p1) += F_1;
        F_el.segment<3>(3*p2) += F_2;
        F_el.segment<3>(3*p3) += F_3;

        Vector3d d2na_dx[4][3][4][3];
        Vector3d d2nb_dx[4][3][4][3];
        Matrix3d Z;
        Z.setZero();
        Matrix3d dq0a_dx[4] = {Z, -I, I, Z};
        Matrix3d dq1a_dx[4] = {I, Z, -I, Z};
        Matrix3d dq2a_dx[4] = {-I, I, Z, Z};
        Matrix3d dq3a_dx[4] = {Z, Z, Z, Z};

        Matrix3d dq0b_dx[4] = {Z, Z, Z, Z};
        Matrix3d dq1b_dx[4] = {Z, Z, I, -I};
        Matrix3d dq2b_dx[4] = {Z, -I, Z, I};
        Matrix3d dq3b_dx[4] = {Z, I, -I, Z};

        //m0
        for(int s = 0; s < 3; ++s) {
            for(int n = 0; n < 4; ++n) {
                for(int t = 0; t < 3; ++t) {
                    d2na_dx[0][s][n][t] = VectorMath::crossProductMatrix(dq0a_dx[n].col(t)).row(s);
                    d2nb_dx[0][s][n][t] = VectorMath::crossProductMatrix(dq0b_dx[n].col(t)).row(s);
                }
            }
        }

        //m1
        for(int s = 0; s < 3; ++s) {
            for(int n = 0; n < 4; ++n) {
                for(int t = 0; t < 3; ++t) {
                    d2na_dx[1][s][n][t] = VectorMath::crossProductMatrix(dq1a_dx[n].col(t)).row(s);
                    d2nb_dx[1][s][n][t] = VectorMath::crossProductMatrix(dq1b_dx[n].col(t)).row(s);
                    
                }
            }
        }

        //m2
        for(int s = 0; s < 3; ++s) {
            for(int n = 0; n < 4; ++n) {
                for(int t = 0; t < 3; ++t) {
                    d2na_dx[2][s][n][t] = VectorMath::crossProductMatrix(dq2a_dx[n].col(t)).row(s);
                    d2nb_dx[2][s][n][t] = VectorMath::crossProductMatrix(dq2b_dx[n].col(t)).row(s);
                    
                }
            }
        }

        //m3
        for(int s = 0; s < 3; ++s) {
            for(int n = 0; n < 4; ++n) {
                for(int t = 0; t < 3; ++t) {
                    d2na_dx[3][s][n][t] = VectorMath::crossProductMatrix(dq3a_dx[n].col(t)).row(s);
                    d2nb_dx[3][s][n][t] = VectorMath::crossProductMatrix(dq3b_dx[n].col(t)).row(s);
                    
                }
            }
        }
        
        double d2cos_theta_dxdx[4][3][4][3];
        for(int m = 0; m < 4; ++m) {
            for(int s = 0; s < 3; ++s) {
                for(int n = 0; n < 4; ++n) {
                    for(int t = 0; t < 3; ++t) {
                        d2cos_theta_dxdx[m][s][n][t] = (1./n_a_norm)*d2na_dx[m][s][n][t].dot(n_b_hat) 
                            + (1./n_b_norm/n_a_norm) * dnb_dx[n][t].dot(dna_dx[m][s])
                            + (1./n_b_norm/n_a_norm) * dna_dx[n][t].dot(dnb_dx[m][s])
                            + (n_a_hat.dot(d2nb_dx[m][s][n][t]));
                    }
                }
            }
        }


        double d2sin_theta_dxdx[4][3][4][3];
        for(int m = 0; m < 4; ++m) {
            for(int s = 0; s < 3; ++s) {
                for(int n = 0; n < 4; ++n) {
                    for(int t = 0; t < 3; ++t) {
                        Vector3d dna_hat_nt = (1./n_a_norm) * dna_dx[n][t];
                        Vector3d dna_hat_ms = (1./n_a_norm) * dna_dx[m][s];
                        Vector3d dnb_hat_nt = (1./n_b_norm) * dnb_dx[n][t];
                        Vector3d dnb_hat_ms = (1./n_b_norm) * dnb_dx[m][s];
                        
                        d2sin_theta_dxdx[m][s][n][t] = 
                            ((1./n_a_norm) * d2na_dx[m][s][n][t].cross(n_b_hat)
                          + (dna_hat_ms).cross(dnb_hat_nt)
                          + (dna_hat_nt).cross(dnb_hat_ms)
                          + (n_a_hat.cross((1./n_b_norm) * d2nb_dx[m][s][n][t]))).dot(e_hat)

                          + (dna_hat_ms.cross(n_b_hat) + n_a_hat.cross(dnb_hat_ms)).dot((1./e_norm) * de_dx[n][t])
                          + (dna_hat_nt.cross(n_b_hat) + n_a_hat.cross(dnb_hat_nt)).dot((1./e_norm) * de_dx[m][s]);

                    }
                }
            }
        }

        double d2C_bend_dxdx[4][3][4][3];

        for(int m = 0; m < 4; ++m) {
            for(int s = 0; s < 3; ++s) {
                for(int n = 0; n < 4; ++n) {
                    for(int t = 0; t < 3; ++t) {
                        d2C_bend_dxdx[m][s][n][t] = cos_theta * d2sin_theta_dxdx[m][s][n][t] - sin_theta * d2cos_theta_dxdx[m][s][n][t]
                        + (sin_theta*sin_theta + cos_theta*cos_theta) * (dsin_theta_dx[m][s] * dcos_theta_dx[n][t] + dcos_theta_dx[m][s]*dsin_theta_dx[n][t])
                        + 2*sin_theta*cos_theta*(dcos_theta_dx[m][s]*dcos_theta_dx[n][t] - dsin_theta_dx[m][s]*dsin_theta_dx[n][t]);
                    }
                }
            }
        }



        MatrixXd bendDeriv(x.size(), x.size());
        bendDeriv.setZero();

        MatrixXd bendDampDerivDX(x.size(), x.size());
        bendDampDerivDX.setZero();

        MatrixXd bendDampDerivDV(x.size(), x.size());
        bendDampDerivDV.setZero();


        int p_[4] = {p0, p1, p2, p3};


        double dC_bend_dt = 0.;
        
        for(int z = 0; z < 4; ++z) {
            dC_bend_dt += dC_bend[z].dot(v.segment<3>(3*p_[z]));
        }


        Vector3d F_bend_damp_0 = -kDampStretch * dC_bend_0 * dC_bend_dt;
        Vector3d F_bend_damp_1 = -kDampStretch * dC_bend_1 * dC_bend_dt;
        Vector3d F_bend_damp_2 = -kDampStretch * dC_bend_2 * dC_bend_dt;
        Vector3d F_bend_damp_3 = -kDampStretch * dC_bend_3 * dC_bend_dt;

        F_d.segment<3>(3*p0) += F_bend_damp_0;
        F_d.segment<3>(3*p1) += F_bend_damp_1;
        F_d.segment<3>(3*p2) += F_bend_damp_2;
        F_d.segment<3>(3*p3) += F_bend_damp_3;

        cout << "F_D AFTER BEND: " << endl;
        cout << F_d << endl;


        for(int i = 0; i < 4; ++i) {
            for(int j = 0; j < 4; ++j) {
                Matrix3d C_built;
                for(int k=0; k < 3; ++k) {
                    for(int l = 0; l < 3; ++l) {
                        C_built(k,l) = d2C_bend_dxdx[i][k][j][l];
                    }
                }
                Matrix3d K = -kBend * dC_bend[i] * dC_bend[j].transpose() + C_built*C_bend;
                bendDeriv.block<3,3>(3*p_[i], 3*p_[j]) = K;
                bendDampDerivDX.block<3,3>(3*p_[i], 3*p_[j]) = -kDampStretch * C_built * dC_bend_dt;
                bendDampDerivDV.block<3,3>(3*p_[i], 3*p_[j]) = -kDampStretch * dC_bend[i] * dC_bend[j].transpose();

            }
        }

        dFdx += bendDeriv.sparseView();
        dFdx += bendDampDerivDX.sparseView();
        dFdv += bendDampDerivDV.sparseView();
    }

}