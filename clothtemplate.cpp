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

	map<epair, vector<Vector3i > > edgesToFaces;

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

		epair e1(face[0], face[1]);
		epair e2(face[1], face[2]);
		epair e3(face[2], face[0]);

		if(edgesToFaces.count(e1) > 0) {
			edgesToFaces[e1].push_back(face);
		} else {
			vector< Vector3i > faceList;
			faceList.push_back(face);
			edgesToFaces[e1] = faceList;
		}

		if(edgesToFaces.count(e2) > 0) {
			edgesToFaces[e2].push_back(face);
		} else {
			vector< Vector3i > faceList;
			faceList.push_back(face);
			edgesToFaces[e2] = faceList;
		}

		if(edgesToFaces.count(e3) > 0) {
			edgesToFaces[e3].push_back(face);
		} else {
			vector< Vector3i > faceList;
			faceList.push_back(face);
			edgesToFaces[e3] = faceList;
		}
	}

	for(auto it = edgesToFaces.begin(); it != edgesToFaces.end(); ++it) {
		std::pair<epair, vector< Vector3i >> p = *it;
		cout << "edge: " << p.first.a << " " << p.first.b << endl;
		cout << p.second.size() << endl;
		if(p.second.size() > 1) {
			std::pair<Vector3i, Vector3i> connectedFaces(p.second[0], p.second[1]);

		}
	}

	for(int i = 0; i < masses.size(); ++i) {
		Minvcoeffs.push_back(Tr(3*i,   3*i,   1./masses(i)));
		Minvcoeffs.push_back(Tr(3*i+1,   3*i+1,   1./masses(i)));
		Minvcoeffs.push_back(Tr(3*i+2,   3*i+2,   1./masses(i)));
	}

	invMass.setFromTriplets(Minvcoeffs.begin(), Minvcoeffs.end());
}