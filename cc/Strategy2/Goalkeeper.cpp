#include "Goalkeeper.hpp"
#include "Field.h"

using namespace field;
using namespace Geometry;

void Goalkeeper::protect_goal(const Point& ball, const Point& ball_est) {
	if (at_location(ball, Location::OurUpperCorner)) {
		go_to_and_stop(our::area::upper::center);
	} else if (at_location(ball, Location::OurLowerCorner)) {
		go_to_and_stop(our::area::lower::center);
	} else if (ball_est.x < ball.x && distance(ball, ball_est) > 0.03) {
		// a bola está vindo para o nosso campo

		// Se goal_est estiver na pequena área, fica na projeção da estimativa da bola
		// se não fica no limite da pequena área
		if (match_y(ball_est, Location::OurBox))
			go_to_and_stop({our::area::box::upper_limit.x, ball_est.y},0.9);
		else if (at_location(ball_est, Location::UpperField))
			go_to_and_stop(our::area::box::upper_limit,0.9);
		else
			go_to_and_stop(our::area::box::lower_limit,0.9);
	} else {
		// fica na projeção da bola, sempre em frente ao gol
		if (match_y(ball, Location::OurBox))
			go_to_and_set_orientation({our::area::box::upper_limit.x, ball.y},
									  degree_to_rad(90), 0.9);

		else if (at_location(ball, Location::UpperField))
			go_to_and_set_orientation(our::area::box::upper_limit,
									  degree_to_rad(90), 0.9);

		else go_to_and_set_orientation(our::area::box::lower_limit,
									   degree_to_rad(90), 0.9);
	}

}

void Goalkeeper::spin_shot(const Point& ball) {
	if (ball.x > get_position().x) { // bola na frente/lateral do goleiro
		if (ball.y > get_position().y) // bola acima do goleiro
			spin(-35.0); // horário
		else
			spin(35.0); // anti-horário
	} else {
		// FIXME: Caso contrário o goleiro deve ficar parado para evitar fazer gol contra. Poderia fazer outra coisa?
		go_to_and_stop(get_position());
	}
}

