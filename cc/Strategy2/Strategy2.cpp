#include "Strategy2.hpp"

using namespace field;
using namespace Geometry;

Strategy2::Strategy2(Attacker &attacker, Defender &defender, Goalkeeper& goalkeeper, Geometry::Point &ball,
		Geometry::Point &ball_est) :
		attacker(attacker),
		defender(defender),
		goalkeeper(goalkeeper),
		ball(ball),
		ball_est(ball_est) {
}

void Strategy2::run() {
	duration_ms since_last_transition = sc::now() - last_transition;
	if(since_last_transition.count() > 2000) {
		bool transitioned = transitions();
		if(transitioned)
			last_transition = sc::now();
	}

	execute_goalkeeper();
	execute_defender();
	execute_attacker();
}

bool Strategy2::transitions() {

	// Cruzamento
	if (at_location(ball, Location::TheirBox) &&
		distance(attacker.get_position(), ball) > distance(defender.get_position(), ball))
		swap_robots(attacker, defender);


	//Se a bola está bem atrás do atacante mas está na frente do defensor
	else if (distance_x(attacker.get_position(), ball) > 0.36 && is_ball_behind(attacker) &&
		!at_location(attacker, Location::TheirCornerAny))
		swap_robots(attacker, defender);

	//Se o defensor está mais perto da bola do que o atacante
	else if (distance_x(attacker.get_position(), ball) > distance_x(defender.get_position(), ball) &&
		!at_location(attacker, Location::TheirCornerAny))
		swap_robots(attacker, defender);


	//Caso a bola esteja atrás do defensor e do atacante
	else if (is_ball_behind(attacker) && is_ball_behind(defender) && at_location(ball, Location::WideDangerZone))
		swap_all_robots();
	else
		return false;

	return true;
}

void Strategy2::execute_goalkeeper() {
	if (distance(goalkeeper.get_position(), ball) < 0.10)
		goalkeeper.spin_shot(ball);
	else
		goalkeeper.protect_goal(ball, ball_est);

}

void Strategy2::execute_defender() {
	if (is_ball_est_ahead(their::area::front::center, -0.08)) {
		// Bola na na área adversária
		if (at_location(ball, Location::UpperField))
			defender.wait_at_target(defender::front::upper::wait_point, ball);
		else
			defender.wait_at_target(defender::front::lower::wait_point, ball);
	} else {
		defender.protect_goal(ball);
	}
}

void Strategy2::execute_attacker() {
	if (is_ball_behind(our::area::front::center)) {
		attacker.protect_goal(ball);
	} else if (at_location(attacker, Location::TheirCornerAny)) {
		attacker.crossing(ball);
	} else if (has_ball(attacker) && at_location(attacker, Location::TheirAreaSideAny)) {
		attacker.spin_shot(ball);
	} else if (at_location(attacker, Location::OurBox)) {
		// Cobrar penalti
		attacker.charged_shot(ball);
	} else {
		attacker.uvf_to_goal(ball);
	}
}

void Strategy2::swap_robots(Robot2& robot1, Robot2& robot2)
{
	const auto pose = robot1.get_pose();
	const auto ID = robot1.get_ID();
	const auto tag = robot1.tag;

	robot1.set_pose(robot2.get_pose());
	robot1.set_ID(robot2.get_ID());
	robot1.tag = robot2.tag;

	robot2.set_pose(pose);
	robot2.set_ID(ID);
	robot2.tag = tag;
}

void Strategy2::swap_all_robots() {
	// antigo goalkeeper -> atacker / antigo defender -> goalkeeper / antigo attacker -> defender
	swap_robots(defender, attacker);
	swap_robots(attacker, goalkeeper);
}

bool Strategy2::is_ball_behind(const Point& point) {
	return ball.x < point.x;
}

bool Strategy2::is_ball_behind(const Robot2& robot) {
	return is_ball_behind(robot.get_position());
}

bool Strategy2::is_ball_est_ahead(const Point& point, double offset) {
	return ball_est.x > point.x + offset;
}

bool Strategy2::has_ball(const Robot2& robot) {
	return distance(robot.get_position(), ball) < 0.08;
}
