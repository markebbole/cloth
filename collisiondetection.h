#ifndef COLLISIONDETECTION
#define COLLISIONDETECTION

#include <vector>
#include <set>
#include <Eigen/Core>
#include "clothinstance.h"
#include "clothtemplate.h"
#include <Eigen/Core>
#include "vectormath.h"
#include <Eigen/Geometry>
#include <iostream>
#include "exact-ccd/rootparitycollisiontest.h"
#include "exact-ccd/vec.h"
#include "obstacle.h"

class Obstacle;
class ClothInstance;

struct BBox
{
    double mins[3];
    double maxs[3];
};

struct AABBNode
{
    AABBNode() : left(NULL), right(NULL), childTriangle(-1) {}
    ~AABBNode() {delete left; delete right;}

    AABBNode *left;
    AABBNode *right;
    BBox box;
    int childTriangle;
};



struct Collision
{
    int pointIndex;
    int triIndex;
    int obstacleIndex;
    Eigen::Vector3d n_hat;
    Eigen::Vector3d bary;
    double rel_velocity;
    int obstaclePointIndex;
    bool isValid;

    bool operator<(const Collision& other) const
    {
        if(pointIndex < other.pointIndex) {
            return true;
        }

        if(pointIndex > other.pointIndex) {
            return false;
        }

        return triIndex < other.triIndex;

    }

};

bool vertInTet(const Eigen::Vector3d &p, const Eigen::Vector3d &q1, const Eigen::Vector3d &q2, const Eigen::Vector3d &q3, const Eigen::Vector3d &q4);
void selfCollisions(ClothInstance* cloth,  std::set<Collision> &collisions);
void selfCollisionsCT(ClothInstance* cloth, Eigen::VectorXd& newX, Eigen::VectorXd& newV, std::set<Collision> &collisions);
void obstacleCollisions(ClothInstance* cloth, std::vector< Obstacle *>& obstacles, std::set<Collision> &collisions);
void triangleTriangleCollisions(ClothInstance* cloth, AABBNode* clothNode, AABBNode* otherNode, int otherObjectIndex, std::vector<Obstacle*>& obstacles, std::set<Collision>& collisions);

//void collisionDetection(const std::vector<RigidBodyInstance *> instances, std::set<Collision> &collisions);
AABBNode *buildAABB(const ClothInstance * instance);
AABBNode *buildObstacleAABB(const Obstacle * instance);


#endif // COLLISIONDETECTION