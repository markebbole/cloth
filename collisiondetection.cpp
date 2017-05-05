#include "collisiondetection.h"


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

void refitAABBCT(const ClothInstance * instance, AABBNode *node, VectorXd& newX)
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
            Eigen::Vector3d point2 = newX.segment<3>(3*tri[k]);


            for(int l=0; l<3; l++)
            {
                node->box.mins[l] = min(node->box.mins[l], point[l]);
                node->box.mins[l] = min(node->box.mins[l], point2[l]);

                node->box.maxs[l] = max(node->box.maxs[l], point[l]);
                node->box.maxs[l] = max(node->box.maxs[l], point2[l]);

            }
        }
    }
    else if(node->left && node->right)
    {
        refitAABBCT(instance, node->left, newX);
        refitAABBCT(instance, node->right, newX);
        for(int i=0; i<3; i++)
        {
            node->box.mins[i] = min(node->left->box.mins[i], node->right->box.mins[i]);
            node->box.maxs[i] = max(node->left->box.maxs[i], node->right->box.maxs[i]);
        }
    }
}

AABBNode *buildObstacleAABB(const Obstacle* obst) {
    int nTris = (int)obst->F.rows();
    vector<AABBNode *> leaves(nTris);
    for(int j=0; j<nTris; j++)
    {
        AABBNode *leaf = new AABBNode;
        leaf->childTriangle = j;
        Eigen::Vector3i tri = obst->F.row(j);
        BBox box;
        for(int k=0; k<3; k++)
        {
            box.mins[k] = numeric_limits<double>::infinity();
            box.maxs[k] = -numeric_limits<double>::infinity();
        }
        for(int k=0; k<3; k++)
        {
            Eigen::Vector3d point = obst->V.row(tri[k]);//>x.segment<3>(3*tri[k]);
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
            Eigen::Vector3d point = instance->x.segment<3>(3*tri[k]);
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

    Vector3d pointVelocity = cloth->v.segment<3>(3*pointIndex);


    if(abs(vec_43.dot(n_hat)) < .1) {
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

        if(w(0) >= -sqrtarea && w(0) <= 1 + sqrtarea && w(1) >= -sqrtarea && w(1) <= 1 + sqrtarea && w3 >= -sqrtarea && w3 <= 1+sqrtarea) {
            Vector3d triPointVel = w(0) * v1 + w(1) * v2 + w3 * v3;

            double rel_velocity = n_hat.dot(pointVelocity) - n_hat.dot(triPointVel);

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

void pointTest(ClothInstance* cloth, AABBNode* clothNode, int pIndex, BBox& pointBox, std::set<Collision> &collisions) {

    Vector3d point = cloth->x.segment<3>(3*pIndex);

    if(!clothNode) {
        return;
    }

    if(!intersects(clothNode->box, pointBox)) {
        return;
    }

    //leaf node
    if(clothNode->childTriangle != -1) {

        Vector3i tri = cloth->getTemplate().getFaces().row(clothNode->childTriangle);
        if(tri[0] == pIndex || tri[1] == pIndex || tri[2] == pIndex) {
            return;
        }

        pointTriangleProximity(cloth, clothNode->childTriangle, pIndex, point, collisions);
    } else {
        pointTest(cloth, clothNode->left, pIndex, pointBox, collisions);
        pointTest(cloth, clothNode->right, pIndex, pointBox, collisions);
    }

}


void pointTestCT(ClothInstance* cloth, AABBNode* clothNode, int pIndex, BBox& pointBox, std::set<Collision> &collisions, VectorXd& newX, VectorXd& newV) {

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

        Vector3d p0 = cloth->x.segment<3>(3*pIndex);
        Vector3d p1 = cloth->x.segment<3>(3*tri[0]);
        Vector3d p2 = cloth->x.segment<3>(3*tri[1]);
        Vector3d p3 = cloth->x.segment<3>(3*tri[2]);
        Vec3d p0_(p0[0], p0[1], p0[2]);
        Vec3d p1_(p1[0], p1[1], p1[2]);
        Vec3d p2_(p2[0], p2[1], p2[2]);
        Vec3d p3_(p3[0], p3[1], p3[2]);

        Vector3d p0_new = newX.segment<3>(3*pIndex);
        Vector3d p1_new = newX.segment<3>(3*tri[0]);
        Vector3d p2_new = newX.segment<3>(3*tri[1]);
        Vector3d p3_new = newX.segment<3>(3*tri[2]);
        Vec3d p0_new_(p0_new[0], p0_new[1], p0_new[2]);
        Vec3d p1_new_(p1_new[0], p1_new[1], p1_new[2]);
        Vec3d p2_new_(p2_new[0], p2_new[1], p2_new[2]);
        Vec3d p3_new_(p3_new[0], p3_new[1], p3_new[2]);


        
        
        rootparity::RootParityCollisionTest t(p0_, p1_, p2_, p3_, p0_new_, p1_new_, p2_new_, p3_new_, false);
        bool isCollision = t.run_test();
        if(isCollision) {
            Vector3i tri = cloth->getTemplate().getFaces().row(clothNode->childTriangle);

            Vector3d v1 = newV.segment<3>(3*tri[0]);
            Vector3d v2 = newV.segment<3>(3*tri[1]);
            Vector3d v3 = newV.segment<3>(3*tri[2]);

            Vector3d n_hat = (p3-p1).cross(p2-p1).normalized();
            if(n_hat.dot(p0 - p1) < 0) {
                n_hat = -n_hat;
            }

            Vector3d pointVelocity = newV.segment<3>(3*pIndex);

            double m11 = (p1-p3).dot(p1-p3);
            double m21 = (p1-p3).dot(p2-p3);
            double m12 = (p1-p3).dot(p2-p3);
            double m22 = (p2-p3).dot(p2-p3);
            Matrix2d M;
            M << m11, m12, m21, m22;

            Vector2d A((p1-p3).dot(p0-p3), (p2-p3).dot(p0-p3));

            Vector2d w = M.inverse() * A;
            double w3 = 1. - w(0) - w(1);

            Vector3d triPointVel = w(0) * v1 + w(1) * v2 + w3 * v3;

            double rel_velocity = n_hat.dot(pointVelocity) - n_hat.dot(triPointVel);

            Collision c;
            c.pointIndex = pIndex;
            c.triIndex = clothNode->childTriangle;
            c.bary = Vector3d(w(0), w(1), w3);            
            c.n_hat = n_hat;
            c.rel_velocity = rel_velocity;
            collisions.insert(c);
                
        }

        
    } else {
        pointTestCT(cloth, clothNode->left, pIndex, pointBox, collisions, newX, newV);
        pointTestCT(cloth, clothNode->right, pIndex, pointBox, collisions, newX, newV);
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


// void pointObstacleTest(ClothInstance* cloth, int pIndex, BBox& pointBox, Obstacle* obst, int obstIndex, AABBNode* obstNode, std::set<Collision>& collisions) {
//     if(!obstNode)
//         return;

//     if(!intersects(pointBox, obstNode->box))
//         return;

//     if(obstNode->childTriangle != -1) {
//         pointObstacleTriangleProximity(cloth, pIndex, pointBox, obst, obstIndex, obstNode, collisions);
//     } else {
//         pointObstacleTest(cloth, pIndex, pointBox, obst, obstIndex, obstNode->left, collisions);
//         pointObstacleTest(cloth, pIndex, pointBox, obst, obstIndex, obstNode->right, collisions);
//     }
// }

void obstacleCollisions(ClothInstance* cloth, std::vector< Obstacle *>& obstacles, std::set<Collision> &collisions) {
    collisions.clear();

    refitAABB(cloth, cloth->AABB);


    for(int i = 0; i < (int)obstacles.size(); ++i) {
        triangleTriangleCollisions(cloth, cloth->AABB, obstacles[i]->AABB, i, obstacles, collisions);
    }
}

void obstacleCollisionsCT(ClothInstance* cloth, VectorXd& newX, VectorXd& newV, std::vector< Obstacle*>& obstacles, std::set<Collision> &collisions) {
    collisions.clear();

    refitAABBCT(cloth, cloth->AABB, newX);

    for(int i = 0; i < (int)obstacles.size(); ++i) {
        triangleTriangleCollisionsCT(cloth, newX, newV, cloth->AABB, obstacles[i]->AABB, i, obstacles, collisions);
    }
}


Collision pointTriProx(Vector3d& p0, Vector3d& p1, Vector3d& p2, Vector3d& p3) {

    Vector3d vec_43 = p0 - p3;
    Vector3d n_hat = (p3-p1).cross(p2-p1).normalized();
    if(n_hat.dot(p0-p1) < 0) {
        n_hat = -n_hat;
    }

    Collision c;
    c.isValid = false;

    if(abs(vec_43.dot(n_hat)) < .1) {
        double m11 = (p1-p3).dot(p1-p3);
        double m21 = (p1-p3).dot(p2-p3);
        double m12 = (p1-p3).dot(p2-p3);
        double m22 = (p2-p3).dot(p2-p3);
        Matrix2d M;
        M << m11, m12, m21, m22;

        Vector2d A((p1-p3).dot(p0-p3), (p2-p3).dot(p0-p3));

        Vector2d w = M.inverse() * A;
        double w3 = 1. - w(0) - w(1);
        //.1 should be replaced with characteristic length of triangle. sqrt of area?
        double sqrtarea = .1*sqrt(abs((p2-p1).cross(p3-p1).norm())/2.);

        if(w(0) >= -sqrtarea && w(0) <= 1 + sqrtarea && w(1) >= -sqrtarea && w(1) <= 1 + sqrtarea && w3 >= -sqrtarea && w3 <= 1+sqrtarea) {
            c.bary = Vector3d(w(0), w(1), w3);
            c.n_hat = n_hat;

            c.isValid = true;
        }

    }

    return c;


}

void triTriIntersect(ClothInstance* cloth, AABBNode* clothNode, AABBNode* otherNode, int otherObjectIndex, std::vector<Obstacle*>& obstacles, std::set<Collision>& collisions) {
    Vector3i clothTri = cloth->getTemplate().getFaces().row(clothNode->childTriangle);
    Vector3i objTri = obstacles[otherObjectIndex]->F.row(otherNode->childTriangle);

    Vector3d clothVerts[3];
    Vector3d objVerts[3];

    for(int i = 0; i < 3; ++i) {
        clothVerts[i] = cloth->x.segment<3>(3*clothTri[i]);
        objVerts[i] = obstacles[otherObjectIndex]->V.row(objTri[i]);

    }

    for(int i = 0; i < 3; ++i) {

        Collision potentialColl = pointTriProx(clothVerts[i], objVerts[0], objVerts[1], objVerts[2]);
        Collision potentialColl2 = pointTriProx(objVerts[i], clothVerts[0], clothVerts[1], clothVerts[2]);

        if(potentialColl.isValid) {

            potentialColl.pointIndex = clothTri[i];
            double rel_velocity = potentialColl.n_hat.dot(cloth->v.segment<3>(3*clothTri[i]));

            potentialColl.rel_velocity = rel_velocity;
            potentialColl.obstacleIndex = otherObjectIndex;
            potentialColl.triIndex = otherNode->childTriangle;
            collisions.insert(potentialColl);
        }

        if(potentialColl2.isValid) {
            potentialColl2.pointIndex = -1;

            potentialColl2.triIndex = clothNode->childTriangle;
            Vector3d v1 = cloth->v.segment<3>(3*clothTri[0]);
            Vector3d v2 = cloth->v.segment<3>(3*clothTri[1]);
            Vector3d v3 = cloth->v.segment<3>(3*clothTri[2]);


            Vector3d triPointVel = potentialColl2.bary(0) * v1 
                + potentialColl2.bary(1) * v2 + potentialColl2.bary(2) * v3;

            potentialColl2.rel_velocity = -potentialColl2.n_hat.dot(triPointVel);
            potentialColl2.obstaclePointIndex = objTri[i];
            potentialColl2.obstacleIndex = otherObjectIndex;
            collisions.insert(potentialColl2);

        }
    }
}


void triTriIntersectCT(ClothInstance* cloth, VectorXd& newX, VectorXd& newV, AABBNode* clothNode, AABBNode* otherNode, int otherObjectIndex, std::vector<Obstacle*>& obstacles, std::set<Collision>& collisions) {
    Vector3i clothTri = cloth->getTemplate().getFaces().row(clothNode->childTriangle);
    Vector3i objTri = obstacles[otherObjectIndex]->F.row(otherNode->childTriangle);

    Vector3d clothVerts[3];
    Vector3d clothVertsNew[3];
    Vector3d objVerts[3];

    Vec3d clothVerts_[3];
    Vec3d clothVertsNew_[3];
    Vec3d objVerts_[3];


    for(int i = 0; i < 3; ++i) {
        clothVerts[i] = cloth->x.segment<3>(3*clothTri[i]);
        clothVertsNew[i] = newX.segment<3>(3*clothTri[i]);
        objVerts[i] = obstacles[otherObjectIndex]->V.row(objTri[i]);


        clothVerts_[i] = Vec3d(clothVerts[i][0], clothVerts[i][1], clothVerts[i][2]);
        clothVertsNew_[i] = Vec3d(clothVertsNew[i][0], clothVertsNew[i][1], clothVertsNew[i][2]);
        objVerts_[i] = Vec3d(objVerts[i][0], objVerts[i][1], objVerts[i][2]);



    }



    for(int i = 0; i < 3; ++i) {


        rootparity::RootParityCollisionTest t1(clothVerts_[i], objVerts_[0], objVerts_[1], objVerts_[2], 
            clothVertsNew_[i], objVerts_[0], objVerts_[1], objVerts_[2], false);
        bool isCollision1 = t1.run_test();

        rootparity::RootParityCollisionTest t2(objVerts_[i], clothVerts_[0], clothVerts_[1], clothVerts_[2],
            objVerts_[i], clothVertsNew_[0], clothVertsNew_[1], clothVertsNew_[2], false);

        bool isCollision2 = t2.run_test();

        if(isCollision1) {
            Vector3i tri = cloth->getTemplate().getFaces().row(clothNode->childTriangle);

            Vector3d n_hat = (objVerts[2]-objVerts[0]).cross(objVerts[1]-objVerts[0]).normalized();
            if(n_hat.dot(clothVerts[i] - objVerts[0]) < 0) {
                n_hat = -n_hat;
            }

            Vector3d pointVelocity = newV.segment<3>(3*clothTri[i]);

            double m11 = (objVerts[0]-objVerts[2]).dot(objVerts[0]-objVerts[2]);
            double m21 = (objVerts[0]-objVerts[2]).dot(objVerts[1]-objVerts[2]);
            double m12 = (objVerts[0]-objVerts[2]).dot(objVerts[1]-objVerts[2]);
            double m22 = (objVerts[1]-objVerts[2]).dot(objVerts[1]-objVerts[2]);
            Matrix2d M;
            M << m11, m12, m21, m22;

            Vector2d A((objVerts[0]-objVerts[2]).dot(clothVerts[i]-objVerts[2]), (objVerts[1]-objVerts[2]).dot(clothVerts[i]-objVerts[2]));

            Vector2d w = M.inverse() * A;
            double w3 = 1. - w(0) - w(1);

            double rel_velocity = n_hat.dot(pointVelocity);

            Collision c;
            c.pointIndex = clothTri[i];
            c.bary = Vector3d(w(0), w(1), w3);            
            c.n_hat = n_hat;
            c.rel_velocity = rel_velocity;
            c.isValid = true;
            c.obstacleIndex = otherObjectIndex;
            c.triIndex = otherNode->childTriangle;
            collisions.insert(c);

                
        }

        if(isCollision2) {

            Vector3i tri = obstacles[otherObjectIndex]->F.row(otherNode->childTriangle);

            Vector3d n_hat = (clothVerts[2]-clothVerts[0]).cross(clothVerts[1]-clothVerts[0]).normalized();
            if(n_hat.dot(objVerts[i] - clothVerts[0]) < 0) {
                n_hat = -n_hat;
            }

            double m11 = (clothVerts[0]-clothVerts[2]).dot(clothVerts[0]-clothVerts[2]);
            double m21 = (clothVerts[0]-clothVerts[2]).dot(clothVerts[1]-clothVerts[2]);
            double m12 = (clothVerts[0]-clothVerts[2]).dot(clothVerts[1]-clothVerts[2]);
            double m22 = (clothVerts[1]-clothVerts[2]).dot(clothVerts[1]-clothVerts[2]);
            Matrix2d M;
            M << m11, m12, m21, m22;

            Vector2d A((clothVerts[0]-clothVerts[2]).dot(objVerts[i]-clothVerts[2]), (clothVerts[1]-clothVerts[2]).dot(objVerts[i]-clothVerts[2]));

            Vector2d w = M.inverse() * A;
            double w3 = 1. - w(0) - w(1);


            Vector3d v1 = newV.segment<3>(3*clothTri[0]);
            Vector3d v2 = newV.segment<3>(3*clothTri[1]);
            Vector3d v3 = newV.segment<3>(3*clothTri[2]);


            

            Collision c;
            c.bary = Vector3d(w(0), w(1), w3);

            Vector3d triPointVel = c.bary(0) * v1 
                + c.bary(1) * v2 + c.bary(2) * v3;

            c.rel_velocity = -n_hat.dot(triPointVel);

            c.isValid = true;
            c.pointIndex = -1;

            c.triIndex = clothNode->childTriangle;
            c.obstaclePointIndex = objTri[i];
            c.obstacleIndex = otherObjectIndex;
            collisions.insert(c);


        }

        // if(potentialColl2.isValid) {
        //     potentialColl2.pointIndex = -1;

        //     potentialColl2.triIndex = clothNode->childTriangle;
        //     Vector3d v1 = cloth->v.segment<3>(3*clothTri[0]);
        //     Vector3d v2 = cloth->v.segment<3>(3*clothTri[1]);
        //     Vector3d v3 = cloth->v.segment<3>(3*clothTri[2]);


        //     Vector3d triPointVel = potentialColl2.bary(0) * v1 
        //         + potentialColl2.bary(1) * v2 + potentialColl2.bary(2) * v3;

        //     potentialColl2.rel_velocity = -potentialColl2.n_hat.dot(triPointVel);
        //     potentialColl2.obstaclePointIndex = objTri[i];
        //     potentialColl2.obstacleIndex = otherObjectIndex;
        //     collisions.insert(potentialColl2);

        // }
    }
}


void triangleTriangleCollisions(ClothInstance* cloth, AABBNode* clothNode, AABBNode* otherNode, int otherObjectIndex, std::vector<Obstacle*>& obstacles, std::set<Collision>& collisions) {
    if(!otherNode || !clothNode) {
        return;
    }

    if(!intersects(clothNode->box, otherNode->box)) {
        return;
    }

    if(clothNode->childTriangle != -1) {
        if(otherNode->childTriangle != -1) {
            triTriIntersect(cloth, clothNode, otherNode, otherObjectIndex, obstacles, collisions);

        } else {
            triangleTriangleCollisions(cloth, clothNode, otherNode->left, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisions(cloth, clothNode, otherNode->right, otherObjectIndex, obstacles, collisions);

        }
    } else {
        if(otherNode->childTriangle != -1) {
            triangleTriangleCollisions(cloth, clothNode->left, otherNode, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisions(cloth, clothNode->right, otherNode, otherObjectIndex, obstacles, collisions);

        } else {
            triangleTriangleCollisions(cloth, clothNode->left, otherNode->left, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisions(cloth, clothNode->left, otherNode->right, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisions(cloth, clothNode->right, otherNode->left, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisions(cloth, clothNode->right, otherNode->right, otherObjectIndex, obstacles, collisions);

        }
    }
}


void triangleTriangleCollisionsCT(ClothInstance* cloth, VectorXd& newX, VectorXd& newV, AABBNode* clothNode, AABBNode* otherNode, int otherObjectIndex, std::vector<Obstacle*>& obstacles, std::set<Collision>& collisions) {
    if(!otherNode || !clothNode) {
        return;
    }

    if(!intersects(clothNode->box, otherNode->box)) {
        return;
    }

    if(clothNode->childTriangle != -1) {
        if(otherNode->childTriangle != -1) {

            triTriIntersectCT(cloth, newX, newV, clothNode, otherNode, otherObjectIndex, obstacles, collisions);
            
        } else {
            triangleTriangleCollisionsCT(cloth, newX, newV, clothNode, otherNode->left, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisionsCT(cloth, newX, newV, clothNode, otherNode->right, otherObjectIndex, obstacles, collisions);

        }
    } else {
        if(otherNode->childTriangle != -1) {
            triangleTriangleCollisionsCT(cloth, newX, newV, clothNode->left, otherNode, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisionsCT(cloth, newX, newV, clothNode->right, otherNode, otherObjectIndex, obstacles, collisions);

        } else {
            triangleTriangleCollisionsCT(cloth, newX, newV, clothNode->left, otherNode->left, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisionsCT(cloth, newX, newV, clothNode->left, otherNode->right, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisionsCT(cloth, newX, newV, clothNode->right, otherNode->left, otherObjectIndex, obstacles, collisions);
            triangleTriangleCollisionsCT(cloth, newX, newV, clothNode->right, otherNode->right, otherObjectIndex, obstacles, collisions);

        }
    }
}

void selfCollisionsCT(ClothInstance* cloth, VectorXd& newX, VectorXd& newV, std::set<Collision> &collisions) {
    collisions.clear();
    int nPoints = (int)cloth->getTemplate().getVerts().size()/3;

    refitAABBCT(cloth, cloth->AABB, newX);

    for(int i = 0; i < nPoints; ++i) {
        Vector3d p = cloth->x.segment<3>(3*i);
        Vector3d p2 = newX.segment<3>(3*i);

        BBox pointBox;
        for(int j = 0; j < 3; ++j) {
            pointBox.mins[j] = min(p[j], p2[j]);
            pointBox.maxs[j] = max(p[j], p2[j]);
        }

        for(int j = 0; j < 3; ++j) {
            pointBox.mins[j] -= 0.05;
            pointBox.maxs[j] += 0.05;
        }



        pointTestCT(cloth, cloth->AABB, i, pointBox, collisions, newX, newV);
    }

}
