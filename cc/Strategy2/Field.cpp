#include "Field.h"

using namespace field;

// Função que será chamada pela estratégia para saber a localização dos agentes
bool field::at_location(const Geometry::Point &position, const Location location) {
	switch (location) {
		case Location::OurField:
			return position.x < center::point.x;
		case Location::TheirField:
			return position.x >= center::point.x;
		case Location::UpperField:
			return position.y >= center::point.y;
		case Location::LowerField:
			return position.y < center::point.y;
		case Location::OurGoal:
			return position.x < our::goal::front::center.x;
		case Location::TheirGoal:
			return position.x > their::goal::front::center.x;
		case Location::OurBox:
			return position.x >= our::goal::front::center.x && position.x <= our::area::front::center.x
				   && position.y >= our::area::box::lower_limit.y && position.y <= our::area::box::upper_limit.y;
		case Location::TheirBox:
			return position.x <= their::goal::front::center.x && position.x >= their::area::front::center.x
				   && position.y >= their::area::lower::center.y && position.y <= their::area::upper::center.y;
		case Location::OurUpperCorner:
			return position.x < our::area::front::center.x && position.y > our::area::upper::center.y;
		case Location::OurLowerCorner:
			return position.x < our::area::front::center.x && position.y < our::area::lower::center.y;
		case Location::TheirCornerAny:
			return position.x >= their::area::front::center.x && (position.y >= their::area::upper::center.y
																  || position.y <= their::area::lower::center.y);
		case Location::WideDangerZone:
			return position.x < our::free_ball::upper::point.x;
		case Location::TheirAreaSideAny:
			return position.x > their::area::front::center.x &&
				   ((position.y >= their::area::lower::center.y && position.y
																   <= their::goal::front::lower_limit.y) ||
					(position.y <= their::area::upper::center.y && position.y >= their::goal::front::upper_limit.y));
		case Location::AnySide:
			return (position.y <= 0.1) || (position.y >= field_height-0.1);
		case Location::AnyGoal:
			return (position.x <= (our::goal::front::center.x + 0.03) || position.x >= (their::goal::front::center.x - 0.03));
		default:
			return false;
	}
}

bool field::at_location(const Robot2 &robot, const Location location) {
	return at_location(robot.get_position(), location);
}

// Verifica se o eixo Y da posição condiz com o eixo Y de uma localização do campo
bool field::match_y(const Geometry::Point &position, const Location &location) {
	switch (location) {
		case Location::OurBox:
			return position.y >= our::area::box::lower_limit.y && position.y <= our::area::box::upper_limit.y;
		default:
			return false;
	}
}

