#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <Eigen/Core>
#include <Eigen/Sparse>
class AABBNode;

using namespace Eigen;

//a triangle mesh that doesn't move. Only exists to interact with cloth.
class Obstacle {
public:
	Obstacle(const Eigen::MatrixX3d &verts, const Eigen::MatrixX3i &faces) : V(verts), F(faces) {
	}

	Eigen::MatrixX3d V;
	Eigen::MatrixX3i F;
	AABBNode* AABB;

	Eigen::Vector3d color;

	void render();


};


#endif
