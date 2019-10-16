#ifndef VSSS_UNIVECTOR_FIELD_H
#define VSSS_UNIVECTOR_FIELD_H

#include "Geometry/Geometry.h"
#include "Field.h"

class UnivectorField {
	public:

		struct UVF_params {	// Parametros utilizados no UVF
			double radius;
			double kr;
			double k0;
			double dMin;
			double lDelta;
		};

		void updateConstants(const UnivectorField::UVF_params params);


		void updateRobot(const Geometry::Point &ball);


		void updateTarget(const Geometry::Point &ball);


		void updateObstacles(Geometry:: Point ball);


		void univector(const Geometry::Point& ball);

	//private:
	//	Role get_role() override { return Role::Attacker; };

	//	void decide_spin_shot(const Geometry::Point &ball);
};

#endif //VSSS_ATTACKER_H
