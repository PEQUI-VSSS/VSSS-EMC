#include "Robot2.h"

void Robot2::go_to(Geometry::Point point, double velocity) {
	command = Command::Vector;
	Geometry::Vector direction = point - pose.position;
	target.orientation = direction.theta;
	target.velocity = velocity;
}

void Robot2::go_to_pose(Geometry::Point point, Geometry::Vector direction, double velocity) {
	command = Command::UVF;
	target.position = point;
	uvf_ref = point + direction.with_size(0.1);
	target.orientation = direction.theta;
	target.velocity = velocity;
}

void Robot2::go_to_and_stop(Geometry::Point point, double velocity) {
	if (distance(point, pose.position) >= OFFSET) {
		command = Command::Position;
		target.position = point;
		target.velocity = velocity;
	} else {
		command = Command::None;
	}
}


void Robot2::go_to_and_set_orientation(Geometry::Point point, double orientation, double velocity) {
	if (distance(point, pose.position) >= OFFSET) {
		command = Command::Position;
		target.position = point;
		target.velocity = velocity;
	} else {
		set_target_orientation({1, orientation});
	}
}

void Robot2::go_in_direction(Geometry::Vector vector, double velocity) {
	command = Command::Vector;
	target.orientation = vector.theta;
	target.velocity = velocity;
}

void Robot2::set_target_orientation(Geometry::Vector orientation) {
	command = Command::Orientation;
	target.orientation = orientation.theta;
}

void Robot2::spin(double angular_velocity) {
	command = Command::Angular_Vel;
	target.angular_velocity = angular_velocity;
}

//	Overloads using default target velocity
void Robot2::go_to(Geometry::Point point) {
	go_to(point, default_target_velocity);
}

void Robot2::go_to_pose(Geometry::Point point, Geometry::Vector direction) {
	go_to_pose(point, direction, default_target_velocity);
}

void Robot2::go_to_and_stop(Geometry::Point point) {
	go_to_and_stop(point, default_target_velocity);
}

void Robot2::go_in_direction(Geometry::Vector vector) {
	go_in_direction(vector, default_target_velocity);
}

void Robot2::set_pose(cv::Point position, double orientation) {
	pose.position = Geometry::from_cv_point(position);
	pose.orientation = -orientation;
}

void Robot2::set_pose(const Pose &new_pose) {
	// Note que não há conversão de cv::Point para Geometry::Point aqui
	pose = new_pose;
}

void Robot2::set_ID(char new_ID) {
	ID = new_ID;
}

