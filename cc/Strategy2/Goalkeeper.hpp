//
// Created by daniel on 17/08/18.
//

#ifndef VSSS_GOALKEEPER_HPP
#define VSSS_GOALKEEPER_HPP

#include "Robot2.h"
#include "Geometry/Geometry.h"

class Goalkeeper : public Robot2 {
	private:
		Role get_role() override { return Role::Goalkeeper; };
		std::string get_role_name() override { return "Goalkeeper"; };

	public:
		std::vector<Adversary> *adversaries;
		// Beahviors
		void protect_goal(const Geometry::Point& ball, const Geometry::Point& ball_est);
		void spin_shot(const Geometry::Point& ball);
		void exit_goal();
};

#endif //VSSS_GOALKEEPER_HPP
