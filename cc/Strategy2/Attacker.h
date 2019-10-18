#ifndef VSSS_ATTACKER_H
#define VSSS_ATTACKER_H

#include "boost/variant.hpp"
#include "Robot2.h"
#include "Geometry/Geometry.h"
#include "Field.h"

class Attacker : public Robot2 {
	public:

		std::vector<Adversary> *adversaries;

		/**	Robô vai para a bola usando o UVF
		 *	@param ball posição da bola
		 *	robô se move com velocidade "default_target_velocity" */
		void uvf_to_goal(const Geometry::Point &ball);

		/** Robô gira com sentido horário ou anti-horário dependendo da posição da bola
		 *	@param ball posição da bola
		 */
		void spin_shot(const Geometry::Point &ball);

		/**	Robô trava a bola no canto e faz um cruzamento
		 * @param ball posição da bola
		*/
		void crossing(const Geometry::Point &ball);

		/**	Robô vai para a bola na lateral e gira no sentido do gol
		 * @param ball posição da bola
		*/
		void side_spin_shot(Geometry:: Point ball);


		void protect_goal(const Geometry::Point& ball);
		void charged_shot(const Geometry::Point& ball);
		void exit_goal(const Geometry::Point &ball);

	private:
		Role get_role() override { return Role::Attacker; };
		std::string get_role_name() override { return "Attacker"; };

		void decide_spin_shot(const Geometry::Point &ball);
};

#endif //VSSS_ATTACKER_H
