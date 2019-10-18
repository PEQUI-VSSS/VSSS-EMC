#include "Robot2.h"

using namespace Geometry;

constexpr float K0 = 0.12;
constexpr float dmin = 0.15;
constexpr float delta = 4.5;

Vector shifting_vector(Vector obs_vel, Vector robot_vel) {
	return (obs_vel - robot_vel) * K0;
}

Point virtual_obs(Point obs, Vector obs_vel, Point robot, Vector vel_robot) {
	auto s = shifting_vector(obs_vel, vel_robot);
	auto d = (obs - robot).size;
	if (d >= s.size) {
		return obs + s;
	} else {
		return obs + s * (d / s.size);
	}
}

double avoidance_field(Point obs, Vector obs_vel, Point robot, Vector vel_robot) {
	auto virtual_obs = virtual_obs(obs, obs_vel, robot, vel_robot);
	return (robot - virtual_obs).theta;
}

double apply_avoidance_field(double target_theta, Point obs, Vector obs_vel, Point robot, Vector vel_robot) {
	auto fi = avoidance_field(obs, obs_vel, robot, vel_robot);
	auto R = (obs - robot).size;
	auto gaussian = std::exp(std::pow(R - dmin, 2)/(2*std::pow(delta, 2)));
	if (R <= dmin) {
		return fi;
	} else {
		return fi * gaussian + target_theta * (1 - gaussian);
	}
}

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
	auto p = Geometry::distance(point, get_position());
	target.velocity = velocity;
}

void Robot2::go_to_and_stop(Geometry::Point point, double velocity) {
	if (distance(point, pose.position) >= TARGET_OFFSET) {
		command = Command::Position;
		target.position = point;
		target.velocity = velocity;
	} else {
		command = Command::None;
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
	target.velocity = default_target_velocity;
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
