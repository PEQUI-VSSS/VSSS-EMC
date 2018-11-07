#include "Defender.hpp"

using namespace Geometry;
using namespace field;

void Defender::wait_at_target(Geometry::Point target, Geometry::Point &ball) {
	if (distance(get_position(), target) > TARGET_OFFSET)
		go_to_and_stop(target);
	else
		set_target_orientation(ball - get_position());
}

void Defender::protect_goal(const Geometry::Point &ball) {
	if (ball.x >= get_position().x) {
		go_to_and_stop({defender::back::upper_limit.x, ball.y});
	} else if (at_location(ball, Location::UpperField)){
		go_to_and_stop(defender::back::upper_limit);
	} else {
		go_to_and_stop(defender::back::lower_limit);
	}
}

void Defender::exit_goal() {
		go_to(center::point);
}
