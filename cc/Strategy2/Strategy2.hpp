#ifndef VSSS_STRATEGY2_HPP
#define VSSS_STRATEGY2_HPP

#include "Geometry/Geometry.h"
#include "Strategy2/Attacker.h"
#include "Strategy2/Defender.hpp"
#include "Strategy2/Goalkeeper.hpp"
#include "Strategy2/Field.h"
#include "Adversary.hpp"
#include <chrono>

class Strategy2 {
	public:
		std::vector<Adversary> adversaries;


	private:
		using sc = std::chrono::system_clock;
		using duration_ms = std::chrono::duration<double, std::milli>;

		Attacker &attacker;
		Defender &defender;
		Goalkeeper &goalkeeper;
		Geometry::Point &ball;
		Geometry::Point &ball_est;

		sc::time_point last_transition = sc::now();

//		Returns true if a transition happens
		bool transitions();
		void swap_robots(Robot2 &robot1, Robot2 &robot2);
		void swap_all_robots();
		bool has_ball(const Robot2 &robot);
		bool is_ball_behind(const Geometry::Point &point);
		bool is_ball_behind(const Robot2 &robot);
		bool is_ball_est_ahead(const Geometry::Point &point, double offset = 0);
		void execute_goalkeeper();
		void execute_attacker();
		void execute_defender();

	public:

		Strategy2(Attacker &attacker, Defender &defender, Goalkeeper &goalkeeper, Geometry::Point &ball,
				  Geometry::Point &ball_est);
		void run();
};

#endif //VSSS_STRATEGY2_HPP
