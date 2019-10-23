#include "Attacker.h"

using namespace Geometry;
using namespace field;

void Attacker::decide_spin_shot(const Geometry::Point &ball) {
	double upper_y = get_position().y + SIZE/2;
	double upper_x = get_position().x + SIZE/2;
	double lower_x = get_position().x - SIZE/2;

	auto robot_to_ball = ball - get_position();

	if (at_location(get_position(), Location::UpperField)) {
		if (robot_to_ball.theta < -M_PI/2) {
			spin(35);
		} else {
			spin(-35);
		}
	} else {
		if (robot_to_ball.theta > M_PI/2) {
			spin(-35);
		} else {
			spin(35);
		}
	}

//	if ((at_location(get_position(), Location::UpperField) && (ball.y > upper_y || ball.x > upper_x))
//		|| (at_location(get_position(), Location::LowerField) && (ball.y > upper_y || ball.x < lower_x))) {
//		spin(-35.0); // Robô gira no sentido horário
//	} else {
//		spin(35);//Robô gira no sentido anti-horário
//	}
}

void Attacker::uvf_to_goal(const Geometry::Point &ball, const Geometry::Point &ball_est) {
	Point goal = their::goal::back::center;
	Vector ball_to_pred = ball_est - ball;
	Point target = ball + (ball_to_pred * 0.2);

	Vector ball_to_goal = goal - ball;
	Vector pred_to_goal = goal - target;
	Vector robot_to_ball = ball - get_position();
	double orientation_error = std::abs(wrap(robot_to_ball.theta - ball_to_goal.theta));
//	double orientation_error_backwards = std::abs(wrap(robot_to_ball.theta - (ball_to_goal.theta + M_PI)));
	auto error_smaller_than = [&](double theta) {
		return orientation_error < degree_to_rad(theta);
	};
	auto error_bigger_than = [&](double theta) {
		return orientation_error > degree_to_rad(theta);
	};

	switch (uvf_state) {
		case seek_ball:
			if (error_smaller_than(30) && get_position().x < target.x && robot_to_ball.size < 0.15)
				uvf_state = close_to_ball;
			break;
		case close_to_ball:
			if (error_bigger_than(70))
				uvf_state = seek_ball;
			else if (robot_to_ball.size < 0.7)
				uvf_state = has_ball;
			break;
		case has_ball:
			if (error_bigger_than(70) || robot_to_ball.size > 0.15)
				uvf_state = seek_ball;
			break;
	}

	switch (uvf_state) {
		case seek_ball:
			go_to_pose(target, ball_to_goal);
			break;
		case close_to_ball:
			go_to(ball);
			break;
		case has_ball:
			go_to(goal, 1.4);
			break;
	}
}

void Attacker::spin_shot(const Geometry::Point &ball) {
	decide_spin_shot(ball);
}

void Attacker::crossing(const Geometry::Point &ball){
	if (distance(ball, get_position()) >= SIZE) {
		// longe da bola: vai até a bola
		go_to(ball);
	} else {
		decide_spin_shot(ball);
	}
}

void Attacker::protect_goal(const Geometry::Point &ball) {
	if (distance(get_position(), ball) < BALL_OFFSET) {
		 // Se a bola chegar perto, gira para jogar a bola longe
		if (at_location(ball, Location::UpperField))
			spin(-35); // horário
		else
			spin(35); // anti-horário
	} else if (at_location(ball, Location::UpperField)) {
		// bloquear area (cima)
		go_to_and_stop(our::corner::upper::attacker::point);
	} else {
		// bloquear area (baixo)
		go_to_and_stop(our::corner::lower::attacker::point);
	}
}

void Attacker::charged_shot(const Geometry::Point &ball) {
	go_in_direction(ball - get_position(), 1.2);
}

void Attacker::side_spin_shot(Point ball){
	double distance_to_ball = distance(get_position(), ball);

	if(get_position().x < ball.x-0.01 && (distance_to_ball > 0.065) ){
		//para evitar o robo ficar dançando quando estiver na lateral correndo atrás da bola
		go_to(ball);
	}else
	if (distance_to_ball <= 0.065 && get_position().x < ball.x){
		if(ball.y > their::goal::front::center.y){
			spin(-35);//Robô gira no sentido anti-horárioo
		}else{
			spin(+35);// Robô gira no sentido horário
		}
	}else{
		if(ball.y > their::goal::front::center.y){
			Vector ball_to_side = {1,  degree_to_rad(45)};
			go_to_pose(ball, ball_to_side);
		} else {
			Vector ball_to_side = {1, - degree_to_rad(45)}; //ball_to_goal.theta - degree_to_rad(90)
			go_to_pose(ball, ball_to_side);
		}

	}
}

void Attacker::exit_goal(const Geometry::Point &ball) {
	if (!at_location(get_position(), Location::OurField) && ball.x > get_pose().position.x)
		// Se o atacante estiver dentro do gol adversário, mas a bola está na frente dele
		// só continua indo pro gol, não importa
		go_in_direction({1,0}, 1.4);
	else
		// Sai do gol
		go_to(center::point);
}
