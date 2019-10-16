//
// Created by vsss on 15/10/19.
//

#ifndef VSSS_ADVERSARY_HPP
#define VSSS_ADVERSARY_HPP

#include <Geometry/Geometry.h>

class Adversary {
	public:
		Geometry::Point position{0, 0};
		Geometry::Vector velocity{0, 0}; // size: velocity, theta: direction of movement

		void update(Geometry::Point new_position, double time) {
			auto direction = new_position - position;
			position = new_position;
			velocity = {direction.size/time, direction.theta};

			std::cout << "p: " << position << ", v: " << velocity << std::endl;
		}
};

#endif //VSSS_ADVERSARY_HPP
