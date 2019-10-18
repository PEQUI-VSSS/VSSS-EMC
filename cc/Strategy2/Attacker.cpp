#include "Attacker.h"

using namespace Geometry;
using namespace field;

void Attacker::decide_spin_shot(const Geometry::Point &ball) {
	double upper_y = get_position().y + SIZE/2;
	double upper_x = get_position().x + SIZE/2;
	double lower_x = get_position().x - SIZE/2;

	if ((at_location(get_position(), Location::UpperField) && (ball.y > upper_y || ball.x > upper_x))
		|| (at_location(get_position(), Location::LowerField) && (ball.y > upper_y || ball.x < lower_x))) {
		spin(-35.0); // Robô gira no sentido horário
	} else {
		spin(35);//Robô gira no sentido anti-horário
	}
}

void Attacker::uvf_to_goal(const Geometry::Point &ball) {
	Vector ball_to_goal = their::goal::back::center - ball;
	if(distance(ball, get_position()) > 0.03){
		go_to_pose(ball, ball_to_goal);
	}else{
	    go_in_direction(ball_to_goal);
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
