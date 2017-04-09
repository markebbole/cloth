#ifndef CLOTHINSTANCE_H
#define CLOTHINSTANCE_H

#include <Eigen/Core>
#include <list>
#include <vector>

class ClothTemplate;
struct AABBNode;

class ClothInstance
{
public:
    ClothInstance(const ClothTemplate &ctemplate, Eigen::VectorXd q_init);
    ~ClothInstance();

    void render();  

    int time;  

    Eigen::VectorXd q;
    Eigen::VectorXd v;

    Eigen::Vector3d color;

    //AABBNode *AABB;

    const ClothTemplate &getTemplate() const {return ctemplate_;}

private:
    const ClothTemplate &ctemplate_;
};

#endif // CLOTHINSTANCE_H
