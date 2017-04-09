#include "rigidbodytemplate.h"
#include "distance.h"
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <map>
#include <Eigen/Sparse>

using namespace std;
using namespace Eigen;

RigidBodyTemplate::RigidBodyTemplate(const std::string &meshFilename, double scale) : volume_(0), radius_(0)
{
    inertiaTensor_.setZero();

    ifstream ifs(meshFilename.c_str());
    vector<Vector3d> pts;
    vector<Vector4i> tets;
    while(true)
    {
        char c;
        ifs >> c;
        if(!ifs)
            break;
        if(c == 'v')
        {
            Vector3d pt;
            ifs >> pt[0] >> pt[1] >> pt[2];
            pts.push_back(pt);
        }
        if(c == 't')
        {
            Vector4i tet;
            ifs >> tet[0] >> tet[1] >> tet[2] >> tet[3];
            tets.push_back(tet);
        }
    }
    V.resize(pts.size(), 3);
    for(int i=0; i<(int)pts.size(); i++)
        V.row(i) = pts[i];
    T.resize(tets.size(), 4);
    for(int i=0; i<(int)tets.size(); i++)
    {
        T.row(i) = tets[i];
    }

    V *= scale;

    initialize();
}

RigidBodyTemplate::RigidBodyTemplate(const MatrixX3d &verts, const MatrixX4i &tets) : V(verts), T(tets), volume_(0)
{
    inertiaTensor_.setZero();
    initialize();
}

RigidBodyTemplate::~RigidBodyTemplate()
{    
}

void RigidBodyTemplate::initialize()
{
    cout << "Initializing rigid body... ";
    cout.flush();
    computeFaces();
    computeVolume();
    Vector3d cm = computeCenterOfMass();
    com_ = cm;
    for(int i=0; i<V.rows(); i++)
        V.row(i) -= cm;

    for(int i=0; i<V.rows(); i++)
    {
        radius_ = max(radius_, V.row(i).norm());

        
    }
    cout << "Radius: " << radius_ << endl;
    computeInertiaTensor();
    // TODO: also precompute the signed distance field data

    //vdf = Distance::vertexDistanceField(V, F);
    cout << "done." << endl;
}

struct triple
{
    triple(int aa, int bb, int cc) : a(aa), b(bb), c(cc)
    {
        if(a < b)
            std::swap(a,b);
        if(a < c)
            std::swap(a,c);
        if(b < c)
            std::swap(b,c);
    }

    int a, b, c;
    bool operator<(const triple &other) const
    {
        if(a < other.a)
            return true;
        else if(a > other.a)
            return false;
        if(b < other.b)
            return true;
        else if(b > other.b)
            return false;
        return c < other.c;
    }
};

void RigidBodyTemplate::computeFaces()
{
    int ntets = (int)T.rows();
    MatrixX3i allfaces(4*ntets, 3);
    Matrix<int, 4, 3> faceidx;
    faceidx << 0, 1, 3,
            3, 1, 2,
            3, 2, 0,
            0, 2, 1;

    for(int i=0; i<ntets; i++)
    {
        Vector4i tet = T.row(i);
        for(int face=0; face<4; face++)
        {
            for(int k=0; k<3; k++)
                allfaces(4*i+face, k) = tet[faceidx(face,k)];
        }
    }

    map<triple, vector<int> > faces;
    for(int i=0; i<4*ntets; i++)
    {
        triple t(allfaces(i,0), allfaces(i,1), allfaces(i,2));
        faces[t].push_back(i/4);
    }

    int nfaces=0;
    for(map<triple, vector<int> >::iterator it = faces.begin(); it != faces.end(); ++it)
        if(it->second.size() == 1)
            nfaces++;

    int ndualedges = (int)faces.size() - nfaces;
    dualEdges.resize(ndualedges, 2);
    F.resize(nfaces,3);
    int idx=0;

    for(int i=0; i<4*ntets; i++)
    {
        triple t(allfaces(i,0), allfaces(i,1), allfaces(i,2));
        if(faces[t].size() == 1)
        {
            F.row(idx) = allfaces.row(i);
            idx++;
        }        
    }

    int deidx=0;
    for(map<triple, vector<int> >::iterator it = faces.begin(); it != faces.end(); ++it)
    {
        if(it->second.size() == 2)
        {
            dualEdges(deidx, 0) = it->second[0];
            dualEdges(deidx, 1) = it->second[1];
            deidx++;
        }
    }
}

void RigidBodyTemplate::computeVolume()
{
    volume_ = 0;
    for(int i=0; i<T.rows(); i++)
    {
        Vector4i face = T.row(i);
        volume_ += 1.0/6.0 * ((V.row(face[1])-V.row(face[0])).cross(V.row(face[2])-V.row(face[0]))).dot(V.row(face[3])-V.row(face[0]));
    }
}

Vector3d RigidBodyTemplate::computeCenterOfMass()
{
    Vector3d cm(0,0,0);
    for(int i=0; i<F.rows(); i++)
    {
        Vector3d pts[3];
        for(int j=0; j<3; j++)
        {
            pts[j] = V.row(F(i,j));
        }
        Vector3d normal = (pts[1]-pts[0]).cross(pts[2]-pts[0]);
        double area = 0.5 * normal.norm();
        normal /= normal.norm();

        Vector3d term(0,0,0);
        for(int j=0; j<3; j++)
        {
            for(int k=0; k<3; k++)
            {
                for(int l=k; l<3; l++)
                    term[j] += pts[k][j]*pts[l][j];
            }
            term[j] *= area*normal[j]/12.0;
        }

        cm += term;
    }

    return cm/volume_;
}

void RigidBodyTemplate::computeInertiaTensor()
{
    Vector3d quads(0,0,0);
    Vector3d mixed(0,0,0);
    for(int i=0; i<F.rows(); i++)
    {
        Vector3d pts[3];
        for(int j=0; j<3; j++)
        {
            pts[j] = V.row(F(i, j));
        }
        Vector3d normal = (pts[1]-pts[0]).cross(pts[2]-pts[0]);
        double area = 0.5 * normal.norm();
        normal /= normal.norm();


        Vector3d term(0,0,0);
        Vector3d mixterm(0,0,0);
        for(int j=0; j<3; j++)
        {
            for(int k=0; k<3; k++)
            {
                for(int l=k; l<3; l++)
                {
                    for(int m=l; m<3; m++)
                        term[j] += pts[k][j]*pts[l][j]*pts[m][j];
                }
            }
            term[j] *= area*normal[j]/30.0;
        }
        double mix = 0;
        for(int j=0; j<3; j++)
        {
            mix += 6.0*pts[j][0]*pts[j][1]*pts[j][2];
            for(int k=0; k<3; k++)
            {
                mix += 2.0*pts[j][k]*pts[j][(k+1)%3]*pts[(j+1)%3][(k+2)%3];
                mix += 2.0*pts[j][k]*pts[j][(k+1)%3]*pts[(j+2)%3][(k+2)%3];
            }
            mix += pts[j][0]*pts[(j+1)%3][1]*pts[(j+2)%3][2];
            mix += pts[j][2]*pts[(j+1)%3][1]*pts[(j+2)%3][0];
        }
        for(int j=0; j<3; j++)
            mixterm[j] = mix*area*normal[j]/60.0;

        quads += term;
        mixed += mixterm;
    }

    inertiaTensor_ << quads[1]+quads[2], -mixed[2], -mixed[1],
            -mixed[2], quads[0]+quads[2], -mixed[0],
            -mixed[1], -mixed[0], quads[0]+quads[1];
}