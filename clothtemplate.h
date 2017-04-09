#ifndef CLOTHTEMPLATE_H
#define CLOTHTEMPLATE_H

#include <string>
#include <Eigen/Core>
#include <set>

class ClothTemplate
{
public:
    ClothTemplate(const Eigen::VectorXd &verts, const Eigen::MatrixX3i &faces);

    ~ClothTemplate();

    const Eigen::VectorXd &getVerts() const {return V;}
    const Eigen::MatrixX3i &getFaces() const {return F;}

private:
    ClothTemplate(const ClothTemplate &other);
    ClothTemplate &operator=(const ClothTemplate &other);

    Eigen::VectorXd V;
    Eigen::MatrixX3i F;
};

#endif // RIGIDBODYTEMPLATE_H
