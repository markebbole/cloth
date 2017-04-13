#ifndef CLOTHTEMPLATE_H
#define CLOTHTEMPLATE_H

#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

#include <set>

using namespace std;
using namespace Eigen;

typedef Eigen::Triplet<double> Tr;
class ClothTemplate
{
public:
    ClothTemplate(const Eigen::VectorXd &verts, const Eigen::MatrixX3i &faces, double d);

    ~ClothTemplate();

    const Eigen::VectorXd &getVerts() const {return V;}
    const Eigen::MatrixX3i &getFaces() const {return F;}
    double getDensity() const { return density; };
    const Eigen::SparseMatrix<double> &getInvMass() const { return invMass; }

private:
    ClothTemplate(const ClothTemplate &other);
    ClothTemplate &operator=(const ClothTemplate &other);

    Eigen::VectorXd V;
    Eigen::MatrixX3i F;
    Eigen::SparseMatrix<double> invMass;
    double density;

    void init();
};

#endif // RIGIDBODYTEMPLATE_H
