#ifndef VSSS_TEAM_HPP
#define VSSS_TEAM_HPP

#include "Robot3.hpp"
#include "Strategy3.hpp"
#include <vector>

enum class RobotColor {
	Blue, Yellow
};

std::string to_string(RobotColor color);

class Team {
	public:
	std::vector<Robot3> robots;
	std::unique_ptr<Strategy> strategy; // IA ou manual
	bool controlled = false;
	bool inverted_field = false;
	RobotColor robot_color;

	Team(int robot_number, int starting_tag, bool controlled, bool inverted_field, RobotColor color) :
							controlled(controlled), inverted_field(inverted_field), robot_color(color) {
		for(int i = starting_tag; i < starting_tag + robot_number; i++) {
			robots.push_back(Robot3(i, 'A' + i));
		}
	};

//	Times adversários usam coordenadas diferentes
	std::vector<Geometry::Point> get_inverted_robot_positions();
};

#endif //VSSS_TEAM_HPP
