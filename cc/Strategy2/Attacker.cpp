#include "Attacker.h"

using namespace Geometry;
using namespace field;

//	Placeholders, devem ser substituidos pelas constantes do Field
const static Line back_line({10,10}, {10, 0});
const static Point goal_center({5, 10});

void Attacker::uvf_to_goal(Point ball) {
	Vector ball_to_goal = goal_center - ball;

	if(distance(ball,get_pose().position)>0.03){
		go_to_pose(ball, ball_to_goal);
	}else{
	    go_in_direction(ball_to_goal);
	}
	//	Inserir corrida para o gol ao chegar na bola
}

void Attacker::spin_shot(Point ball){
	if (ball.y>get_pose().position.y){
		spin(35);//Robô gira no sentido anti-horário
	}else{
		if(ball.y=get_pose().position.y) {
		   if(ball.y>goal_center.y){
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

	if(ball.x > get_pose().position.x) {
		go_to(ball);
		if(distance(ball, get_pose().position) < size) {
			if (ball.y > get_pose().position.y) {
				spin(35);
			}else{
				spin(-35);
			}
		}
	}else{
		uvf_to_goal(ball);
		//Se ele tiver a frente da bola, faz apenas o UVF,
		//porém a condição tem que ser verificada pela estratégia antes de chamar o crossing,
		//ele vai só entrar nesse else em caso de ser chamado em hora indevida.
	}
}

void Attacker::atk_mindcontrol(Point ball){

}

void Attacker::protect_goal(const Geometry::Point &ball) {
	if (distance(pose.position, ball) < 0.1) {
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
	go_in_direction(ball - pose.position, 1.2);
}
