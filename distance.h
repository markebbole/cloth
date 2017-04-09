#ifndef DISTANCE_H
#define DISTANCE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <set>
#include <limits>
#include "collisiondetection.h"
#include <iostream>

class Distance
{


    static Eigen::Matrix3d constructBaryMatrix(Eigen::Vector3d& v0, Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& v3) {
        Eigen::Matrix3d T;
        T << v0[0] - v3[0], v1[0] - v3[0], v2[0] - v3[0],
             v0[1] - v3[1], v1[1] - v3[1], v2[1] - v3[1],
             v0[2] - v3[2], v1[2] - v3[2], v2[2] - v3[2];

        return T;
    }

public:


    static Eigen::VectorXd vertexDistanceField(Eigen::MatrixX3d& verts, Eigen::MatrixX3i& faces) {
        std::set<int> boundaryVerts;

        Eigen::VectorXd distances(verts.rows());

        //get verts on the boundary of the object. These have distance 0
        for(int i = 0; i < (int)faces.rows(); ++i) {
            Eigen::Vector3i face = faces.row(i);
            boundaryVerts.insert(face[0]);
            boundaryVerts.insert(face[1]);
            boundaryVerts.insert(face[2]);
        }

        Eigen::Vector3d v0;
        Eigen::Vector3d v1;
        Eigen::Vector3d v2;

        


        for(int i = 0; i < (int)verts.rows(); ++i) {

            //check if a boundary vert
            if(boundaryVerts.count(i) > 0) {
                distances[i] = 0.;
            } else {
                double shortestDistance = std::numeric_limits<double>::max();
                Eigen::Vector3d vert = verts.row(i);

                //for this vert, find face which has smallest distance
                for(int j = 0; j < (int)faces.rows(); ++j) {
                    Eigen::Vector3i f = faces.row(j);
                    v0 = verts.row(f[0]);
                    v1 = verts.row(f[1]);
                    v2 = verts.row(f[2]);

                    Eigen::Vector3d dist;
                    Eigen::Vector3d d = vertexFaceDistance(vert, v0, v1, v2, dist[0], dist[1], dist[2]);
                    //bool faceBetter = vertexPlaneDistanceLessThan(vert, v0, v1, v2, shortestDistance);

                    /*if((d.norm() < shortestDistance && !faceBetter) || (d.norm() >= shortestDistance && faceBetter)) {
                        std::cout << "OH SHITTTTTTTTTTTTT" << std::endl;
                        std::cout << vert << std::endl;
                        std::cout << v0 << " " << v1 << " " << v2 << std::endl;

                        std::cout << d.norm() << std::endl;
                        std::cout << shortestDistance << std::endl;
                    }*/
                    if(d.norm() < shortestDistance) {
                        shortestDistance = d.norm();
                    } 

                    

                    /*Eigen::Vector3d dist;
                    if(faceBetter) {
                        Eigen::Vector3d d = vertexFaceDistance(vert, v0, v1, v2, dist[0], dist[1], dist[2]);
                        shortestDistance = d.norm();
                    }*/
                }

                if(shortestDistance <= 0) {
                    std::cout << "PROBLEMMMMM " << v0 << " " << v1 << " " << " " << v2 << std::endl;
                }

                distances[i] = -shortestDistance;
            }


        }

        return distances;
    }

    //This computes D(v)
    static double pointBoundaryDistance(Eigen::Vector3d& point, 
        const Eigen::MatrixX3d& verts, 
        Eigen::Vector4i tet, 
        const Eigen::VectorXd& vertDistances) {

        Eigen::Vector3d v0 = verts.row(tet[0]);
        Eigen::Vector3d v1 = verts.row(tet[1]);
        Eigen::Vector3d v2 = verts.row(tet[2]);
        Eigen::Vector3d v3 = verts.row(tet[3]);

        double d0 = vertDistances[tet[0]];
        double d1 = vertDistances[tet[1]];
        double d2 = vertDistances[tet[2]];
        double d3 = vertDistances[tet[3]];

        /*Eigen::Vector3d v0;
        Eigen::Vector3d v1;
        Eigen::Vector3d v2;
        Eigen::Vector3d v3;

        int whichTet = 0;
        std::cout << "about to find which tet the point is in" << std::endl;

        for(int i = 0; i < (int)tets.rows(); ++i) {
            Eigen::Vector4i tet = tets.row(i);

            v0 = verts.row(tet[0]);
            v1 = verts.row(tet[1]);
            v2 = verts.row(tet[2]);
            v3 = verts.row(tet[3]);

            bool pointInTet = vertInTet(point, v0, v1, v2, v3);
            whichTet = i;

            if(pointInTet) { break; }
        }*/

        //std::cout << "about to construct T" << std::endl;

        /*std::cout << v0 << std::endl;
        std::cout << v1 << std::endl;
        std::cout << v2 << std::endl;
        std::cout << v3 << std::endl;*/

        // vi are the vertices of the tet that contains point

        Eigen::RowVector3d f(d0-d3, d1-d3, d2-d3);

        Eigen::Matrix3d T = constructBaryMatrix(v0, v1, v2, v3);

        double interpolatedDistance = d3 + f * T.inverse() * (point - v3);

        return interpolatedDistance;
    }

    //Given a point and the tet it lies inside, find dD(v).
    static Eigen::RowVector3d pointBoundaryDistanceDifferential(const Eigen::Vector4i& tet, const Eigen::MatrixX3d& verts,
        const Eigen::VectorXd& vertDistances) {

        //std::cout << "hello" << std::endl;



        Eigen::Vector3d v0 = verts.row(tet[0]);
        Eigen::Vector3d v1 = verts.row(tet[1]);
        Eigen::Vector3d v2 = verts.row(tet[2]);
        Eigen::Vector3d v3 = verts.row(tet[3]);

        double d0 = vertDistances[tet[0]];
        double d1 = vertDistances[tet[1]];
        double d2 = vertDistances[tet[2]];
        double d3 = vertDistances[tet[3]];

        Eigen::RowVector3d f(d0-d3, d1-d3, d2-d3);

        Eigen::Matrix3d T = constructBaryMatrix(v0, v1, v2, v3);   

        Eigen::RowVector3d differential = f * T.inverse();

        return differential;


    }


    // Efficiently calculates whether or not a point p is closer to than eta distance to the plane spanned by vertices q0, q1, and q2.
    // This method does not require any floating point divisions and so is significantly faster than computing the distance itself.
    static bool vertexPlaneDistanceLessThan(const Eigen::Vector3d &p,
                                            const Eigen::Vector3d &q0, const Eigen::Vector3d &q1, const Eigen::Vector3d &q2, double eta)
    {
        Eigen::Vector3d c = (q1 - q0).cross(q2 - q0);
        return c.dot(p - q0) * c.dot(p - q0) < eta * eta * c.dot(c);
    }

    // Efficiently calculates whether or not the line spanned by vertices (p0, p1) is closer than eta distance to the line spanned by vertices (q0, q1).
    // This method does not require any floating point divisions and so is significantly faster than computing the distance itself.
    static bool lineLineDistanceLessThan(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                         const Eigen::Vector3d &q0, const Eigen::Vector3d &q1, double eta)
    {
        Eigen::Vector3d c = (p1 - p0).cross(q1 - q0);
        return c.dot(q0 - p0) * c.dot(q0 - p0) < eta * eta * c.dot(c);
    }

    // Computes the vector between a point p and the closest point to p on the triangle (q0, q1, q2). Also returns the barycentric coordinates of this closest point on the triangle;
    // q0bary is the barycentric coordinate of q0, etc. (The distance from p to the triangle is the norm of this vector.)
    static Eigen::Vector3d vertexFaceDistance(const Eigen::Vector3d &p,
            const Eigen::Vector3d &q0, const Eigen::Vector3d &q1, const Eigen::Vector3d &q2,
            double &q0bary, double &q1bary, double &q2bary)
    {
        Eigen::Vector3d ab = q1 - q0;
        Eigen::Vector3d ac = q2 - q0;
        Eigen::Vector3d ap = p - q0;

        double d1 = ab.dot(ap);
        double d2 = ac.dot(ap);

        // corner and edge cases

        if (d1 <= 0 && d2 <= 0)
        {
            q0bary = 1.0;
            q1bary = 0.0;
            q2bary = 0.0;
            return q0 - p;
        }

        Eigen::Vector3d bp = p - q1;
        double d3 = ab.dot(bp);
        double d4 = ac.dot(bp);
        if (d3 >= 0 && d4 <= d3)
        {
            q0bary = 0.0;
            q1bary = 1.0;
            q2bary = 0.0;
            return q1 - p;
        }

        double vc = d1 * d4 - d3 * d2;
        if ((vc <= 0) && (d1 >= 0) && (d3 <= 0))
        {
            double v = d1 / (d1 - d3);
            q0bary = 1.0 - v;
            q1bary = v;
            q2bary = 0;
            return (q0 + v * ab) - p;
        }

        Eigen::Vector3d cp = p - q2;
        double d5 = ab.dot(cp);
        double d6 = ac.dot(cp);
        if (d6 >= 0 && d5 <= d6)
        {
            q0bary = 0;
            q1bary = 0;
            q2bary = 1.0;
            return q2 - p;
        }

        double vb = d5 * d2 - d1 * d6;
        if ((vb <= 0) && (d2 >= 0) && (d6 <= 0))
        {
            double w = d2 / (d2 - d6);
            q0bary = 1 - w;
            q1bary = 0;
            q2bary = w;
            return (q0 + w * ac) - p;
        }

        double va = d3 * d6 - d5 * d4;
        if ((va <= 0) && (d4 - d3 >= 0) && (d5 - d6 >= 0))
        {
            double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            q0bary = 0;
            q1bary = 1.0 - w;
            q2bary = w;

            return (q1 + w * (q2 - q1)) - p;
        }

        // face case
        double denom = 1.0 / (va + vb + vc);
        double v = vb * denom;
        double w = vc * denom;
        double u = 1.0 - v - w;
        q0bary = u;
        q1bary = v;
        q2bary = w;
        return (u * q0 + v * q1 + w * q2) - p;
    }

    // Computes the shotest vector between a segment (p0, p1) and segment (q0, q1). Also returns the barycentric coordinates of the closest points on both segments; p0bary is the barycentric
    // coordinate of p0, etc. (The distance between the segments is the norm of this vector).
    static Eigen::Vector3d edgeEdgeDistance(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                                            const Eigen::Vector3d &q0, const Eigen::Vector3d &q1,
                                            double &p0bary, double &p1bary,
                                            double &q0bary, double &q1bary)
    {
        Eigen::Vector3d d1 = p1 - p0;
        Eigen::Vector3d d2 = q1 - q0;
        Eigen::Vector3d r = p0 - q0;
        double a = d1.squaredNorm();
        double e = d2.squaredNorm();
        double f = d2.dot(r);

        double s, t;

        double c = d1.dot(r);
        double b = d1.dot(d2);
        double denom = a * e - b * b;
        if (denom != 0.0)
        {
            s = clamp( (b * f - c * e) / denom );
        }
        else
        {
            //parallel edges and/or degenerate edges; values of s doesn't matter
            s = 0;
        }
        double tnom = b * s + f;
        if (tnom < 0 || e == 0)
        {
            t = 0;
            if (a == 0)
                s = 0;
            else
                s = clamp(-c / a);
        }
        else if (tnom > e)
        {
            t = 1.0;
            if (a == 0)
                s = 0;
            else
                s = clamp( (b - c) / a );
        }
        else
            t = tnom / e;

        Eigen::Vector3d c1 = p0 + s * d1;
        Eigen::Vector3d c2 = q0 + t * d2;

        p0bary = 1.0 - s;
        p1bary = s;
        q0bary = 1.0 - t;
        q1bary = t;

        return c2 - c1;
    }

    // Computes the shortest distance between a triangle mesh and itself, i.e. the shortest distance between a vertex and a face that does not contain that vertex, or of an edge and another edge that
    // does not share a vertex. The mesh has verts1.size()/3 vertices, stored as consecutive triplets in the vector verts, and faces stored as vertex indices in the columns of faces. This method assumes
    // that each vertex is part of at least one triangle.
    // Distances between primitives *all* of whose vertices are in fixedVerts are ignored.
    static double meshSelfDistance(const Eigen::VectorXd &verts, const Eigen::Matrix3Xi &faces, const std::set<int> &fixedVerts);

private:
    static double clamp(double u)
    {
        return std::min(1.0, std::max(u, 0.0));
    }
};

#endif
