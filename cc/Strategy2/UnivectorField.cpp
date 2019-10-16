#include "UnivectorField.h"

using namespace Geometry;
using namespace field;

void UnivectorField::updateConstants(const UnivectorField::UVF_params params) {
//	UVF_params = params;
//	avoidField.updateParam(self.k0);
	//moveField.updateParams(self.kr, self.radius)
}

void UnivectorField::updateRobot(const Geometry::Point &ball) {
	Vector ball_to_goal = their::goal::back::center - ball;
}

void UnivectorField::univector(const Geometry::Point &ball) {
//	this->updateRobot(get_pose());
//	if(orientation == null){
//		orientation = {650.0, 250.0};
//	}

//	fi_auf = 0.0;
//	minDistance = UVF_params::dMin + 1;
//	Pose obstacles[];
//	Position centers[] = new Position[5];
//	//obstacles = nullptr; para desativar os obstaculos
//
//	for (int i = 0; i == obstacles.length; i++) {
//		centers[i] = obstacles[i].position;
//	}
}