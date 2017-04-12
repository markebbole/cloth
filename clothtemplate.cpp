#include "clothtemplate.h"




ClothTemplate::ClothTemplate(const Eigen::VectorXd &verts, const Eigen::MatrixX3i &faces, double d) : V(verts), F(faces), density(d) {

	init();
}


void ClothTemplate::init() {
	invMass.resize(V.size(), V.size());
	invMass.setZero();

	vector<Tr> Minvcoeffs;

	VectorXd masses(V.size()/3);
	masses.setZero();


	for(int i = 0; i < F.rows(); ++i) {
		Vector3i face = F.row(i);
		Vector3d p0 = V.segment<3>(3*face[0]);
		Vector3d p1 = V.segment<3>(3*face[1]);
		Vector3d p2 = V.segment<3>(3*face[2]);
		Vector3d cr = (p1-p0).cross(p2-p0);
		double triArea = density*cr.norm()/2.;

		masses(face[0]) += triArea/3.;
		masses(face[1]) += triArea/3.;
		masses(face[2]) += triArea/3.;
	}

	for(int i = 0; i < masses.size(); ++i) {
		Minvcoeffs.push_back(Tr(3*i,   3*i,   1./masses(i)));
		Minvcoeffs.push_back(Tr(3*i+1,   3*i+1,   1./masses(i)));
		Minvcoeffs.push_back(Tr(3*i+2,   3*i+2,   1./masses(i)));
	}

	invMass.setFromTriplets(Minvcoeffs.begin(), Minvcoeffs.end());
}