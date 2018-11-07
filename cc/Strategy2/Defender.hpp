#ifndef VSSS_DEFENDER_HPP
#define VSSS_DEFENDER_HPP

#include "Robot2.h"
#include "Geometry/Geometry.h"
#include "Field.h"

class Defender : public Robot2 {
	private:
		Role get_role() override { return Role::Defender; };
		std::string get_role_name() override { return "Defender"; };

	public:
		// Behaviors
		void protect_goal(const Geometry::Point &ball);
		void wait_at_target(Geometry::Point target, Geometry::Point &ball);
		void exit_goal();
};

#endif //VSSS_DEFENDER_HPP
