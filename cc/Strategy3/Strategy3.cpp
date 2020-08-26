#include "Strategy3.hpp"
#include "Field.h"
#include "helper.hpp"

using namespace field;
using namespace Geometry;

Robot3 * find_robot_by_role(std::vector<Robot3> &robots, Role role) {
	auto robot_it = std::find_if(robots.begin(), robots.end(), [&role](const Robot3& robot){
		return robot.role == role;
	});
	if (robot_it == robots.end()) {
		return nullptr;
	} else {
		return &*robot_it;
	}
}

void Strategy3::run_strategy(std::vector<Robot3> &team, std::vector<Geometry::Point> &adversaries, Ball ball,
							 bool first_iteration) {
	Robot3* attacker_robot = find_robot_by_role(team, Role::Attacker);
	attacker.set_robot(attacker_robot);
	Robot3* defender_robot = find_robot_by_role(team, Role::Defender);
	defender.set_robot(defender_robot);
	Robot3* goalkeeper_robot = find_robot_by_role(team, Role::Goalkeeper);
	goalkeeper.set_robot(goalkeeper_robot);
	this->ball = ball;

	duration_ms since_last_transition = sc::now() - last_transition;
	if(since_last_transition.count() > 2000) {
		bool transitioned = transitions();
		if(transitioned)
			last_transition = sc::now();
	}

	if (attacker.has_robot()) {
		if (defender.has_robot() &&
				(ball.position.x < 0.3 * field_width) &&
				attacker->get_position().x > 0.3 * field_width) {

			if (at_location(defender->get_position(), Location::UpperField)) {
				auto pos = defender->get_position().y - 0.5;
				auto ynew = std::min(0.1, pos);
				attacker->go_to_and_stop({0.31 * field_width, ynew});
			} else {
				auto pos = defender->get_position().y + 0.15;
				auto ynew = std::min(field_height - 0.5, pos);
				attacker->go_to_and_stop({0.31 * field_width, ynew});
			}
		} else {
			attacker.run_strategy(ball);
		}

		if (goalkeeper.has_robot() && at_location(*goalkeeper.robot, Location::TheirField)) {
//			cobrança de penalti
			attacker->go_to(ball.position);
		} else {
			auto new_limit = defender::back::upper_limit.x + 0.09;

			if (at_location(attacker->get_position(), Location::OurBox)) {
				attacker->go_in_direction({0, 0.8});
			} else if (at_location(attacker->target.pose.position, Location::OurBox)) {
				attacker->stop();
			} else if (attacker->target.pose.position.x < new_limit && attacker->get_position().x > new_limit) {
				attacker->stop();
			}
		}
	}

	if (defender.has_robot()) {
		if (attacker.has_robot() && attacker->get_position().x < 0.3 * field_width) {
			defender->stop();
		} else {
			defender.run_strategy(ball);
		}

		if (at_location(defender->get_position(), Location::OurBox)) {
			defender->go_in_direction({0, 0.8});
		}
		if (at_location(defender->target.pose.position, Location::OurBox)) {
			defender->stop();
		}
	}

	if (goalkeeper.has_robot()) goalkeeper.run_strategy(ball);
}

bool Strategy3::transitions() {
	bool collided = trainstion_by_collision();


//	Todas as transições a seguir são entre atacantes e defensores
	if (!attacker.has_robot() || !defender.has_robot()) return false;

	Point goal = their::goal::back::center;

	Vector ball_to_goal = goal - ball.position;
	Vector robot_to_ball = ball.position - defender->get_position();
	double orientation_error_def = std::abs(wrap(robot_to_ball.theta - ball_to_goal.theta));
	double orientation_error_atk = std::abs(wrap(robot_to_ball.theta - ball_to_goal.theta));

	auto diferent_sides_up = at_location(ball.position, Location::UpperField) && at_location(defender->get_position(), Location::UpperField)
							 && at_location(attacker->get_position(), Location::LowerField);
	auto diferent_sides_down = at_location(ball.position, Location::LowerField) && at_location(defender->get_position(), Location::LowerField)
							   && at_location(attacker->get_position(), Location::UpperField);
	auto distancey = distance_y(attacker->get_position(), defender->get_position());
	auto distancex = distance_x(attacker->get_position(), defender->get_position());
	if (
			((distance(attacker->get_position(), ball.position) > distance(defender->get_position(), ball.position)) && distancey > 0.6)
			//			|| (diferent_sides_up || diferent_sides_down)
			&& (orientation_error_def + degree_to_rad(10) < orientation_error_atk)
//			&& defender.get_position().x > 0.3*field_height
			)
		swap_robots(attacker, defender);

		// Cruzamento
	else if (at_location(ball.position, Location::TheirBox) &&
			 distance(attacker->get_position(), ball.position) > distance(defender->get_position(), ball.position))
		swap_robots(attacker, defender);


		//Se a bola está bem atrás do atacante mas está na frente do defensor
	else if (distance_x(attacker->get_position(), ball.position) > 0.30 && is_ball_behind(attacker->get_position(), ball) &&
			 !at_location(attacker->get_position(), Location::TheirCornerAny))
		swap_robots(attacker, defender);

		//Se o defensor está mais perto da bola do que o atacante
	else if (distance(attacker->get_position(), ball.position) > distance(defender->get_position(), ball.position) &&
			 !at_location(attacker->get_position(), Location::TheirCornerAny))
		swap_robots(attacker, defender);


		//Caso a bola esteja atrás do defensor e do atacante
//	else if (is_ball_behind(attacker) && is_ball_behind(defender) && at_location(ball, Location::WideDangerZone))
//		swap_all_robots();
	else
		return false | collided;

	return true;
}



bool Strategy3::trainstion_by_collision() {
	if (is_collision(goalkeeper.robot, defender.robot)) {
		if (is_opposite_direction(goalkeeper.robot, defender.robot)) {
			swap_robots(goalkeeper, defender);
			std::cout << "troca de goleiro e defensor" << std::endl;
		}
	} else if (is_collision(defender.robot, attacker.robot)) {
		if (is_opposite_direction(defender.robot, attacker.robot)) {
			swap_robots(defender, attacker);
			std::cout << "troca de defensor e atacante" << std::endl;
		}
	} else if (is_collision(goalkeeper.robot, attacker.robot)) {
		if (is_opposite_direction(goalkeeper.robot, attacker.robot)) {
			swap_robots(goalkeeper, attacker);
			std::cout << "troca de goleiro e atacante" << std::endl;
		}
	} else {
		return false;
	}

	return true;
}

void Strategy3::swap_robots(RoleStrategy& role1, RoleStrategy& role2) {
	if (!role1.has_robot() || !role2.has_robot()) return;

	Robot3 * aux_robot1 = role1.robot;
	Role aux_role1 = role1->role;

	role1.robot = role2.robot;
	role1.robot->role = role2.robot->role;

	role2.robot = aux_robot1;
	role2->role = aux_role1;
}

void Strategy3::swap_all_robots() {
	// antigo goalkeeper -> atacker / antigo defender -> goalkeeper / antigo attacker -> defender
//	swap_robots(defender, attacker);
//	swap_robots(attacker, goalkeeper);
}

bool in_range(Robot3 &this_robot, Robot3 &other_robot) {
//	auto max_theta = degree_to_rad(70);
	auto robot1_to_robot2 = this_robot.get_position() - other_robot.get_position();
	auto theta_error = std::abs(wrap(this_robot.pose.orientation - robot1_to_robot2.theta));
	return distance(this_robot.get_position(), other_robot.get_position()) < 0.12 && theta_error < degree_to_rad(60);
}

bool Strategy3::is_collision(Robot3 *robot1, Robot3* robot2) {
	if (robot1 == nullptr || robot2 == nullptr) return false;
	return distance(robot1->get_position(), robot2->get_position()) < 0.12f;
	return is_collision_axis_y(robot1, robot2) && is_collision_axis_x(robot1, robot2);
}

bool Strategy3::is_collision_axis_x(Robot3 *robot1, Robot3 *robot2) {
	if (robot1 == nullptr || robot2 == nullptr) return false;

	double robot1_y = robot1->get_position().y;
	double robot2_y = robot2->get_position().y;

	double robot1_y_start = robot1_y - robot1->SIZE / 2;
	double robot1_y_end   = robot1_y + robot1->SIZE / 2;

	double robot2_y_start = robot2_y - robot2->SIZE / 2;
	double robot2_y_end   = robot2_y + robot2->SIZE / 2;

	return (robot1_y_start >= robot2_y_start && robot1_y_start <= robot2_y_end) ||
		   (robot1_y_end >= robot2_y_start && robot1_y_start <= robot2_y_end);
}

bool Strategy3::is_collision_axis_y(Robot3 *robot1, Robot3 *robot2) {
	if (robot1 == nullptr || robot2 == nullptr) return false;

	double robot1_x = robot1->get_position().x;
	double robot2_x = robot2->get_position().x;

	double robot1_x_start = robot1_x - robot1->SIZE / 2;
	double robot1_x_end   = robot1_x + robot1->SIZE / 2;

	double robot2_x_start = robot2_x - robot2->SIZE / 2;
	double robot2_x_end   = robot2_x + robot2->SIZE / 2;

	return (robot1_x_start >= robot2_x_start && robot1_x_start <= robot2_x_end) ||
		   (robot1_x_end >= robot2_x_start && robot1_x_start <= robot2_x_end);
}

bool Strategy3::is_opposite_direction(Robot3 *robot1, Robot3 *robot2) {
	if (robot1 == nullptr || robot2 == nullptr) return false;

	auto robot1_dir = robot1->get_direction();
	if(robot1_dir.size > 0) {
		auto robot1_to_robot2 = robot2->get_position() - robot1->get_position();
		return std::abs(wrap(robot1_to_robot2.theta - robot1_dir.theta)) < degree_to_rad(90);
	}

	auto robot2_dir = robot2->get_direction();
	if (robot2_dir.size > 0) {
		auto robot2_to_robot1 = robot1->get_position() - robot2->get_position();
		return std::abs(wrap(robot2_to_robot1.theta - robot2_dir.theta)) < degree_to_rad(90);
	}

	return false;
}
