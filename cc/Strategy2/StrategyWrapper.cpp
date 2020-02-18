//
// Created by thiago on 01/02/2020.
//

#include "StrategyWrapper.hpp"

void init() {
	ls_x.init(15, 1);
	ls_y.init(15, 1);
	attacker.tag = 1;
	defender.tag = 2;
	goalkeeper.tag = 3;
}

Robot2 * get_robot(int tag) {
	for (auto robot : robots) {
		if (robot->tag == tag) {
			return robot;
		}
	}
	std::cout << "ERRO do thiago!!" << std::endl;
}

void update_ball_est() {
	ls_x.addValue(ball.x);
	ls_y.addValue(ball.y);
	ball_est.x = ls_x.estimate(10);
	ball_est.y = ls_y.estimate(10);
}

// data[0] e data[1]: x e y em metros, origem no canto inferior esquerdo
// data[2]: orientacao de -pi a pi
// data[3] e data[4]: vel da roda esquerda e direita
// time: tempo em segundos entre cada iteracao do controle
void run(float robot1data[5], float robot2data[5], float robot3data[5], float ballpos[2], float time, float out[6]) {
	auto * robot1 = get_robot(1);
	robot1->set_pose_m({robot1data[0], robot1data[1]}, robot1data[2]);
	auto * robot2 = get_robot(2);
	robot2->set_pose_m({robot2data[0], robot2data[1]}, robot2data[2]);
	auto * robot3 = get_robot(3);
	robot3->set_pose_m({robot3data[0], robot3data[1]}, robot3data[2]);

	ball = {ballpos[0], ballpos[1]};
	update_ball_est();

	strategy.run();

	// robot1 = get_robot(1);
	// robot2 = get_robot(2);
	// robot3 = get_robot(3);
	auto vel1 = robot1->run_control(robot1data[3], robot1data[4], time);
	auto vel2 = robot2->run_control(robot2data[3], robot2data[4], time);
	auto vel3 = robot3->run_control(robot3data[3], robot3data[4], time);
	out[0] = vel1.left;
	out[1] = vel1.right;
	out[2] = vel2.left;
	out[3] = vel2.right;
	out[4] = vel3.left;
	out[5] = vel3.right;
}
