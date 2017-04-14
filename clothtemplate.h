#ifndef CLOTHTEMPLATE_H
#define CLOTHTEMPLATE_H

#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <map>
#include <set>
#include <iostream>
#include <utility>
using namespace std;
using namespace Eigen;

typedef Eigen::Triplet<double> Tr;

struct epair
{
    epair(int aa, int bb) : a(aa), b(bb)
    {
        if(a < b)
            std::swap(a,b);

    }

    int a, b;
    bool operator<(const epair &other) const
    {
        if(a < other.a)
            return true;
        else if(a > other.a)
            return false;
        return b < other.b;
    }
};

class ClothTemplate
{
public:
    ClothTemplate(const Eigen::VectorXd &verts, const Eigen::MatrixX3i &faces, double d);

    ~ClothTemplate();

    const Eigen::VectorXd &getVerts() const {return V;}
    const Eigen::MatrixX3i &getFaces() const {return F;}
    double getDensity() const { return density; };
    const Eigen::SparseMatrix<double> &getInvMass() const { return invMass; }
    const vector < Vector4i > &getAdjacentFaces() const { return adjacentFaces; }


private:
    ClothTemplate(const ClothTemplate &other);
    ClothTemplate &operator=(const ClothTemplate &other);

    Eigen::VectorXd V;
    Eigen::MatrixX3i F;
    Eigen::SparseMatrix<double> invMass;
    vector< Vector4i > adjacentFaces;
    
    double density;
    vector< vector <int> > edgesToFaces;
    void init();
};

#endif // RIGIDBODYTEMPLATE_H
