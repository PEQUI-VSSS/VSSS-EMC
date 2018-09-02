#include "Attacker.h"

using namespace Geometry;
using namespace field;

void Attacker::uvf_to_goal(Point ball) {
	Vector ball_to_goal = their::goal::back::center - ball;

	if(distance(ball, get_position()) > 0.03){
		go_to_pose(ball, ball_to_goal);
	}else{
	    go_in_direction(ball_to_goal);
	}
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
}

void Attacker::charged_shot(const Geometry::Point &ball) {
	go_in_direction(ball - get_position(), 1.2);
}
