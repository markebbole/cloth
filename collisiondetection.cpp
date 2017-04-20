#include "collisiondetection.h"
#include "clothinstance.h"
#include "clothtemplate.h"
#include <Eigen/Core>
#include "vectormath.h"
#include <Eigen/Geometry>
#include <iostream>

using namespace std;

bool intersects(const BBox &b1, const BBox &b2)
{
    for(int i=0; i<3; i++)
    {
        if( (b1.maxs[i] < b2.mins[i]) || (b2.maxs[i] < b1.mins[i]))
            return false;
    }
    return true;
}



class NodeComparator
{
public:
    NodeComparator(int axis) : axis(axis) {}

    int axis;

    bool operator()(const AABBNode *left, const AABBNode *right) const
    {
        return left->box.mins[axis] < right->box.mins[axis];
    }
};

AABBNode *buildAABB(vector<AABBNode *> nodes)
{
    if(nodes.size() == 0)
        return NULL;
    else if(nodes.size() == 1)
        return nodes[0];

    double axismins[3];
    double axismaxs[3];
    for(int i=0; i<3; i++)
    {
        axismins[i] = numeric_limits<double>::infinity();
        axismaxs[i] = -numeric_limits<double>::infinity();
    }
    int nnodes = (int)nodes.size();
    for(int i=0; i<nnodes; i++)
    {
        for(int j=0; j<3; j++)
        {
            axismins[j] = min(axismins[j], nodes[i]->box.mins[j]);
            axismaxs[j] = max(axismaxs[j], nodes[i]->box.maxs[j]);
        }
    }
    double widths[3];
    for(int i=0; i<3; i++)
        widths[i] = axismaxs[i] - axismins[i];
    int splitaxis = -1;
    if(widths[0] >= widths[1] && widths[0] >= widths[2])
        splitaxis = 0;
    else if(widths[1] >= widths[0] && widths[1] >= widths[2])
        splitaxis = 1;
    else
        splitaxis = 2;
    std::sort(nodes.begin(), nodes.end(), NodeComparator(splitaxis));
    vector<AABBNode *> left(nnodes/2);
    vector<AABBNode *> right(nnodes - nnodes/2);
    for(int i=0; i<nnodes/2; i++)
    {
        left[i] = nodes[i];
    }
    for(int i=nnodes/2; i<nnodes; i++)
    {
        right[i-nnodes/2] = nodes[i];
    }
    AABBNode *node = new AABBNode;
    node->left = buildAABB(left);
    node->right = buildAABB(right);
    for(int i=0; i<3; i++)
    {
        node->box.mins[i] = min(node->left->box.mins[i], node->right->box.mins[i]);
        node->box.maxs[i] = max(node->left->box.maxs[i], node->right->box.maxs[i]);
    }
    return node;
}

void refitAABB(const ClothInstance * instance, AABBNode *node)
{
    if(node->childTriangle != -1)
    {
        Eigen::Vector3i tri = instance->getTemplate().getFaces().row(node->childTriangle);
        for(int k=0; k<3; k++)
        {
            node->box.mins[k] = numeric_limits<double>::infinity();
            node->box.maxs[k] = -numeric_limits<double>::infinity();
        }
        for(int k=0; k<3; k++)
        {
            Eigen::Vector3d point = instance->x.segment<3>(3*tri[k]);

            //instance->c + VectorMath::rotationMatrix(instance->theta)*instance->getTemplate().getVerts().row(tet[k]).transpose();
            for(int l=0; l<3; l++)
            {
                node->box.mins[l] = min(node->box.mins[l], point[l]);
                node->box.maxs[l] = max(node->box.maxs[l], point[l]);
            }
        }
    }
    else if(node->left && node->right)
    {
        refitAABB(instance, node->left);
        refitAABB(instance, node->right);
        for(int i=0; i<3; i++)
        {
            node->box.mins[i] = min(node->left->box.mins[i], node->right->box.mins[i]);
            node->box.maxs[i] = max(node->left->box.maxs[i], node->right->box.maxs[i]);
        }
    }
}

AABBNode *buildAABB(const ClothInstance * instance)
{    
    int nTris = (int)instance->getTemplate().getFaces().rows();
    vector<AABBNode *> leaves(nTris);
    for(int j=0; j<nTris; j++)
    {
        AABBNode *leaf = new AABBNode;
        leaf->childTriangle = j;
        Eigen::Vector3i tri = instance->getTemplate().getFaces().row(j);
        BBox box;
        for(int k=0; k<3; k++)
        {
            box.mins[k] = numeric_limits<double>::infinity();
            box.maxs[k] = -numeric_limits<double>::infinity();
        }
        for(int k=0; k<3; k++)
        {
            Eigen::Vector3d point = instance->x.segment<3>(3*tri[k]);//instance->c + VectorMath::rotationMatrix(instance->theta)*instance->getTemplate().getVerts().row(tet[k]).transpose();
            for(int l=0; l<3; l++)
            {
                box.mins[l] = min(box.mins[l], point[l]);
                box.maxs[l] = max(box.maxs[l], point[l]);
            }
        }
        leaf->box = box;
        leaves[j] = leaf;
    }
    return buildAABB(leaves);
}

bool vertInTet(const Eigen::Vector3d &p, const Eigen::Vector3d &q1, const Eigen::Vector3d &q2, const Eigen::Vector3d &q3, const Eigen::Vector3d &q4)
{
    if( (q2-p).cross(q3-p).dot(q4-p) < 0)
        return false;
    if( (p-q1).cross(q3-q1).dot(q4-q1) < 0)
        return false;
    if( (q2-q1).cross(p-q1).dot(q4-q1) < 0)
        return false;
    if( (q2-q1).cross(q3-q1).dot(p-q1) < 0)
        return false;
    return true;
}

/*void tetTetIntersect(const AABBNode *node1, const AABBNode *node2, int body1, int body2, const std::vector<RigidBodyInstance *> instances, std::set<Collision> &collisions)
{
    if(body1==body2)
        return;

    Eigen::Vector4i tet1 = instances[body1]->getTemplate().getTets().row(node1->childtet);
    Eigen::Vector4i tet2 = instances[body2]->getTemplate().getTets().row(node2->childtet);
    Eigen::Vector3d verts1[4];
    Eigen::Vector3d verts2[4];
    for(int i=0; i<4; i++)
    {
        verts1[i] = instances[body1]->c + VectorMath::rotationMatrix(instances[body1]->theta)*instances[body1]->getTemplate().getVerts().row(tet1[i]).transpose();
        verts2[i] = instances[body2]->c + VectorMath::rotationMatrix(instances[body2]->theta)*instances[body2]->getTemplate().getVerts().row(tet2[i]).transpose();
    }
    for(int i=0; i<4; i++)
    {
        if(vertInTet(verts1[i], verts2[0], verts2[1], verts2[2], verts2[3]))
        {
            Collision c;
            c.body1 = body1;
            c.body2 = body2;
            c.collidingVertex = tet1[i];
            c.collidingTet = node2->childtet;
            collisions.insert(c);
        }
        if(vertInTet(verts2[i], verts1[0], verts1[1], verts1[2], verts1[3]))
        {
            Collision c;
            c.body1 = body2;
            c.body2 = body1;
            c.collidingVertex = tet2[i];
            c.collidingTet = node1->childtet;
            collisions.insert(c);
        }
    }
}*/

/*void intersect(const AABBNode *node1, const AABBNode *node2, int body1, int body2, const std::vector<RigidBodyInstance *> instances, std::set<Collision> &collisions)
{
    if(!node1 || !node2)
        return;

    if(!intersects(node1->box, node2->box))
        return;

    if(node1->childtet != -1)
    {
        if(node2->childtet != -1)
        {
            tetTetIntersect(node1, node2, body1, body2, instances, collisions);
        }
        else
        {
            intersect(node1, node2->left, body1, body2, instances, collisions);
            intersect(node1, node2->right, body1, body2, instances, collisions);
        }
    }
    else
    {
        if(node2->childtet != -1)
        {
            intersect(node1->left, node2, body1, body2, instances, collisions);
            intersect(node1->right, node2, body1, body2, instances, collisions);
        }
        else
        {
            intersect(node1->left, node2->left, body1, body2, instances, collisions);
            intersect(node1->left, node2->right, body1, body2, instances, collisions);
            intersect(node1->right, node2->left, body1, body2, instances, collisions);
            intersect(node1->right, node2->right, body1, body2, instances, collisions);
        }
    }
}*/

void pointTriangleProximity(ClothInstance* cloth, int triangleIndex, int pointIndex, Vector3d point, std::set<Collision> &collisions) {
    Vector3i tri = cloth->getTemplate().getFaces().row(triangleIndex);
    Vector3d x1 = cloth->x.segment<3>(3*tri[0]);
    Vector3d x2 = cloth->x.segment<3>(3*tri[1]);
    Vector3d x3 = cloth->x.segment<3>(3*tri[2]);

    Vector3d v1 = cloth->v.segment<3>(3*tri[0]);
    Vector3d v2 = cloth->v.segment<3>(3*tri[1]);
    Vector3d v3 = cloth->v.segment<3>(3*tri[2]);


    //cout << x1 << endl << x2 << endl << x3 << endl << endl;


    
    Vector3d vec_43 = point - x3;
    Vector3d n_hat = (x3-x1).cross(x2-x1).normalized();
    if(n_hat.dot(point - x1) < 0) {
        n_hat = -n_hat;
    }

   // cout << "normal" << endl << n_hat<< endl;

    Vector3d pointVelocity = cloth->v.segment<3>(3*pointIndex);


    if(abs(vec_43.dot(n_hat)) < .1) {
       // cout << vec_43.dot(n_hat) << endl;
       // cout << "below thresh" << endl;
        double m11 = (x1-x3).dot(x1-x3);
        double m21 = (x1-x3).dot(x2-x3);
        double m12 = (x1-x3).dot(x2-x3);
        double m22 = (x2-x3).dot(x2-x3);
        Matrix2d M;
        M << m11, m12, m21, m22;

        Vector2d A((x1-x3).dot(point-x3), (x2-x3).dot(point-x3));

        Vector2d w = M.inverse() * A;
        double w3 = 1. - w(0) - w(1);
        //.1 should be replaced with characteristic length of triangle. sqrt of area?
        double sqrtarea = .1*sqrt(abs((x2-x1).cross(x3-x1).norm())/2.);
       // cout << "here are the W's" << endl;
       // cout << w << endl;
       // cout << w3 << endl;
        //cout << "sqrtarea: " << sqrtarea << endl;
        if(w(0) >= -sqrtarea && w(0) <= 1 + sqrtarea && w(1) >= -sqrtarea && w(1) <= 1 + sqrtarea && w3 >= -sqrtarea && w3 <= 1+sqrtarea) {
            Vector3d triPointVel = w(0) * v1 + w(1) * v2 + w3 * v3;
            //cout << "IN REGION. NOW CHECK REL_VELOCITY" << endl;
            //double rel_velocity = n_hat.dot(triPointVel + pointVelocity);
            double rel_velocity = n_hat.dot(pointVelocity) - n_hat.dot(triPointVel);
         //   cout << "REL_V: " << rel_velocity << endl;

            if(rel_velocity < 0) {
                Collision c;
                c.pointIndex = pointIndex;
                c.triIndex = triangleIndex;
                c.bary = Vector3d(w(0), w(1), w3);
                c.n_hat = n_hat;
                c.rel_velocity = rel_velocity;
                collisions.insert(c);
            }
            
        }
    }
}

void pointTest(ClothInstance* cloth, AABBNode* clothNode, int pIndex, BBox& pointBox, std::set<Collision> &collisions) {
    int nTris = (int)cloth->getTemplate().getFaces().rows();

    Vector3d point = cloth->x.segment<3>(3*pIndex);

    if(!clothNode) {
        return;
    }

    if(!intersects(clothNode->box, pointBox)) {
        return;
    }

    //leaf node
    if(clothNode->childTriangle != -1) {

        //cout << "got to leaf" << endl;
        Vector3i tri = cloth->getTemplate().getFaces().row(clothNode->childTriangle);
        if(tri[0] == pIndex || tri[1] == pIndex || tri[2] == pIndex) {
            return;
        }

        //cout << pIndex << endl;

        //cout << "different triangle" << endl;

        pointTriangleProximity(cloth, clothNode->childTriangle, pIndex, point, collisions);
    } else {
        pointTest(cloth, clothNode->left, pIndex, pointBox, collisions);
        pointTest(cloth, clothNode->right, pIndex, pointBox, collisions);
    }

}

void selfCollisions(ClothInstance* cloth, std::set<Collision> &collisions) {
    collisions.clear();
    int nPoints = (int)cloth->getTemplate().getVerts().size()/3;

    refitAABB(cloth, cloth->AABB);

    for(int i = 0; i < nPoints; ++i) {
        Vector3d p = cloth->x.segment<3>(3*i);
        BBox pointBox;
        Vector3d diff = p - Vector3d(.05, .05, .05);
        pointBox.mins[0] = diff[0];
        pointBox.mins[1] = diff[1];
        pointBox.mins[2] = diff[2];

        diff = p + Vector3d(.05, .05, .05);
        pointBox.maxs[0] = diff[0];
        pointBox.maxs[1] = diff[1];
        pointBox.maxs[2] = diff[2];


        pointTest(cloth, cloth->AABB, i, pointBox, collisions);
    }
}

/*void collisionDetection(const std::vector<RigidBodyInstance *> instances, std::set<Collision> &collisions)
{
    collisions.clear();
    int nbodies = (int)instances.size();

    for(int i=0; i<nbodies; i++)
    {
        refitAABB(instances[i], instances[i]->AABB);
    }

    for(int i=0; i<nbodies; i++)
    {
        for(int j=i+1; j<nbodies; j++)
        {
            intersect(instances[i]->AABB, instances[j]->AABB, i, j, instances, collisions);
        }
    }
}*/
