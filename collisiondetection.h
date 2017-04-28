#ifndef COLLISIONDETECTION
#define COLLISIONDETECTION

#include <vector>
#include <set>
#include <Eigen/Core>

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

// struct Collision
// {
//     int body1; // indices into the list of rigid body instances
//     int body2;

//     int collidingVertex; // index into body1's vertex list
//     int collidingTet; // index into body2's tetrahedra list

//     // constructed so that only one collision between a vertex and a rigid body will be kept (in case the vertex straddles multiple tets)
//     bool operator<(const Collision &other) const
//     {
//         if(body1 < other.body1)
//             return true;
//         if(body1 > other.body1)
//             return false;
//         if(body2 < other.body2)
//             return true;
//         if(body2 > other.body2)
//             return false;
//         return (collidingVertex < other.collidingVertex);
//     }
// };

struct Collision
{
    int pointIndex;
    int triIndex;
    Eigen::Vector3d n_hat;
    Eigen::Vector3d bary;
    double rel_velocity;

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

//void collisionDetection(const std::vector<RigidBodyInstance *> instances, std::set<Collision> &collisions);
AABBNode *buildAABB(const ClothInstance * instance);
AABBNode *buildObstacleAABB(const Obstacle * instance);


#endif // COLLISIONDETECTION