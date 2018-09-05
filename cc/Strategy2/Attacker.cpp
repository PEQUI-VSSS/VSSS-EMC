#include "Attacker.h"

using namespace Geometry;
using namespace field;

void p(const std::string &text) {
//	std::cout << text << std::endl;
}

void Attacker::uvf_to_goal(Point ball) {
	const Point goal = their::goal::back::center;
	Vector ball_to_goal = goal - ball;

	Vector robot_to_ball = ball - get_position();
	Vector robot_to_goal = goal - get_position();
	double theta_error = wrap(robot_to_goal.theta -  robot_to_ball.theta);
	double orientation_error = (robot_to_ball.theta - get_pose().orientation);

//	std::cout << orientation_error * 180.0/M_PI << std::endl;
//	std::cout << theta_error * 180.0/M_PI << std::endl;
//	std::cout << distance(ball, get_position()) << std::endl;

	if (std::abs(theta_error) < degree_to_rad(30) &&
			std::abs(orientation_error) < degree_to_rad(15) &&
			distance(ball, get_position()) < 0.07) {
//		std::cout << "go to" << std::endl;
		go_to(goal, 1.4);
	} else {
//		std::cout << "uvf" << std::endl;
		go_to_pose(ball, ball_to_goal);
//		go_to_pose(goal, {1, 0}, 1.4);
//	    go_in_direction(robot_to_goal);
	}
	p("uvf to goal");
}

void Attacker::spin_shot(Point ball){
	if (ball.y > get_position().y){
		spin(35);//Robô gira no sentido anti-horário
	}else{
		if(ball.y == get_position().y) {
		   if(ball.y > their::goal::front::center.y){
			   spin(35);//Robô gira no sentido anti-horárioo
		   }else{
			   spin(-35);// Robô gira no sentido horário
		   }
		}else{
			spin(-35);// Robô gira no sentido horário
		}
	}
	p("spin shot");
}

void Attacker::crossing(Point ball){

	if(ball.x > get_position().x) {
		if(distance(ball, get_position()) < size) {
			if (ball.y > get_position().y) {
				spin(35);
			}else{
				spin(-35);
			}
		}else{
			go_to(ball);
		}
	}else{
		uvf_to_goal(ball);
		//Se ele tiver a frente da bola, faz apenas o UVF,
		//porém a condição tem que ser verificada pela estratégia antes de chamar o crossing,
		//ele vai só entrar nesse else em caso de ser chamado em hora indevida.
	}
	p("crossing");
}

void Attacker::protect_goal(const Geometry::Point &ball) {
	if (distance(get_position(), ball) < 0.1) {
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
	p("protect goal");
}

void Attacker::charged_shot(const Geometry::Point &ball) {
	go_in_direction(ball - get_position(), 1.2);
	p("charged shot");
}
