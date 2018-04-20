#include "Strategy.hpp"

using std::vector;
using std::cout;
using std::endl;

Strategy::Strategy() {
	LS_ball_x.init(PREDICAO_NUM_SAMPLES, 1);
	LS_ball_y.init(PREDICAO_NUM_SAMPLES, 1);
	ticks = 0;
}

void Strategy::set_constants(int w, int h) {
	width = w;
	height = h;

	ABS_GOAL_TO_GOAL_WIDTH = width;
	ABS_ROBOT_SIZE = static_cast<int>(round(0.08 * float(width) / 1.70));
	COORD_MID_FIELD_X = ABS_GOAL_TO_GOAL_WIDTH / 2;

	ABS_FIELD_HEIGHT = height;
	ABS_GOAL_SIZE_Y = static_cast<int>(round(0.40 * float(height) / 1.30));
	ABS_GOAL_SIZE_X = static_cast<int>(round(0.10 * float(width) / 1.70));
	COORD_GOAL_DEF_FRONT_X = ABS_GOAL_SIZE_X;
	COORD_GOAL_ATK_FRONT_X = ABS_GOAL_TO_GOAL_WIDTH - ABS_GOAL_SIZE_X;
	COORD_GOAL_MID_Y = ABS_FIELD_HEIGHT / 2;
	COORD_GOAL_UP_Y = ABS_FIELD_HEIGHT / 2 - ABS_GOAL_SIZE_Y / 2;
	COORD_GOAL_DWN_Y = ABS_FIELD_HEIGHT / 2 + ABS_GOAL_SIZE_Y / 2;

	ABS_BOX_SIZE_Y = static_cast<int>(round(0.70 * float(height) / 1.30));
	ABS_BOX_SIZE_X = static_cast<int>(round(0.15 * float(width) / 1.70));
	COORD_BOX_DEF_X = ABS_GOAL_SIZE_X + ABS_BOX_SIZE_X;
	COORD_BOX_ATK_X = ABS_GOAL_TO_GOAL_WIDTH - (ABS_GOAL_SIZE_X + ABS_BOX_SIZE_X);
	COORD_BOX_UP_Y = static_cast<int>(round((ABS_FIELD_HEIGHT - ABS_BOX_SIZE_Y) / 2));
	COORD_BOX_DWN_Y = static_cast<int>(round(ABS_BOX_SIZE_Y + (ABS_FIELD_HEIGHT - ABS_BOX_SIZE_Y) / 2));

	//variáveis ajustáveis
	corner_atk_limit = COORD_BOX_ATK_X - ABS_ROBOT_SIZE;
	def_corner_line = COORD_BOX_DEF_X;
	collision_radius = ABS_ROBOT_SIZE / 2;
	fixed_pos_distance = ABS_ROBOT_SIZE;
	max_approach = static_cast<int>(ABS_ROBOT_SIZE * 2.5);
	max_collision_count = 30;
	goalie_line = COORD_GOAL_DEF_FRONT_X + ABS_ROBOT_SIZE / 2;
	transition_y_distance = ABS_ROBOT_SIZE * 4;

	fuzzy_init();
}

double Strategy::distance(cv::Point A, cv::Point B) {
	double dist = sqrt(pow(double(A.y - B.y), 2) + pow(double(A.x - B.x), 2));
	return dist;
}

double Strategy::distance_meters(cv::Point A, cv::Point B) {
	double dist = sqrt(pow(double(A.y - B.y), 2) + pow(double(A.x - B.x), 2));
	dist = 1.7 * dist / ABS_GOAL_TO_GOAL_WIDTH;
	return dist;
}

double Strategy::angle_atan2(cv::Point A, cv::Point B) {
	return atan2(double(A.y - B.y), -double(A.x - B.x));
}

void Strategy::get_variables() {
	goalie_line = testFrame.getValue(0); // index da barrinha
}

void Strategy::get_past(int i) {
	past_position[i] = robots[i].position;
	past_transangle[i] = robots[i].transAngle;
}

void Strategy::get_targets(vector<Robot> *pRobots, cv::Point *advRobots) {
	if(!robots.empty()) {
		robots_ini = robots;
		adv_ini = advRobots;
		cout<<"velocidade: " <<speed_vector(robots_ini[1].position,(*pRobots)[1].position)<<endl;

	}
	robots = *pRobots;
	adv = advRobots;

	get_variables();


	for (int i = 0; i < 3; i++) { //pegar posições e índices
		robots[i].cmdType = VECTOR;
		robots[i].fixedPos = false;
		robots[i].using_pot_field = false;
		robots[i].vmax = robots[i].vdefault;

		switch (robots[i].role) {
			case GOALKEEPER:
				gk = i;
				break;

			case DEFENDER:
				def = i;
				break;

			case ATTACKER:
				atk = i;
				break;

			case OPPONENT:
				opp = i;
				break;
		}
	}

	set_flags(); // analisar situação atual e setar flags

	// danger_zone_1 = false; //só para testes com um robô
	// danger_zone_2 = false; //só para testes com um robô
	// half_transition_enabled = false; //só para testes com um robô
	// full_transition_enabled = false; //só para testes com um robô
	// transition_mindcontrol_enabled = false;

	gk_routine(gk);
	def_routine(def);
	atk_routine(atk);

	// opp_gk_routine(opp);

	for (int i = 0; i < 3; i++) {
		if (!robots[i].using_pot_field) position_to_vector(i);
		fixed_position_check(i);
		collision_check(i);
	}

	overmind();
	Transitions();

	transition_timeout--;
	if (transition_timeout < 0) transition_timeout = 0;

	if (transition_mindcontrol && transition_mindcontrol_enabled) {
		transition_mindcontrol = false; // reseta a flag

		double tmp_distance, minor_distance;

		// pega a menor distância pra bola
		atk = 0;
		minor_distance = distance(robots[atk].position, Ball);
		for (int i = atk + 1; i < robots.size(); i++) {
			if ((tmp_distance = distance(robots[i].position, Ball)) < minor_distance) {
				minor_distance = tmp_distance;
				atk = i;
			}
		}

		// pega a menor distância pro gol
		gk = (atk == 0 ? 1 : 0);
		minor_distance = distance(robots[gk].position, cv::Point(0, COORD_GOAL_MID_Y));
		for (int i = gk + 1; i < robots.size(); i++) {
			if (i == atk)
				continue;
			if ((tmp_distance = distance(robots[i].position, cv::Point(0, COORD_GOAL_MID_Y))) < minor_distance) {
				minor_distance = tmp_distance;
				gk = i;
			}
		}

		// seta o robô restante pra defensor
		// não precisa inicializar pois é sempre setado no for
		for (int i = 0; i < robots.size(); i++) {
			if (i == atk || i == gk)
				continue;
			def = i;
		}

		// seta o atacante
		set_role(atk, ATTACKER);
		// seta o goleiro
		set_role(gk, GOALKEEPER);
		// seta o defensor
		set_role(def, DEFENDER);
	} else {
		if (half_transition && half_transition_enabled && transition_timeout == 0) {

			half_transition = false;
			half_transition_enabled = false;
			transition_timeout = 90; //3 segundos
			for (int i = 0; i < 3; i++) {
				robots[i].status = NORMAL_STATE;
				switch (robots[i].role) {
					case GOALKEEPER:
						break;
					case DEFENDER:
						robots[i].role = ATTACKER;
						robots[i].vmax = robots[i].vdefault;
						atk = i;
						break;
					case ATTACKER:
						robots[i].role = DEFENDER;
						robots[i].vmax = robots[i].vdefault;
						def = i;
						break;
					default:
						break;
				}
			}
		}
		if (full_transition && full_transition_enabled && transition_timeout == 0) {

			full_transition = false;
			full_transition_enabled = false;
			transition_timeout = 90; //3 segundos
			for (int i = 0; i < 3; i++) {
				robots[i].status = NORMAL_STATE;
				switch (robots[i].role) {
					case GOALKEEPER:
						robots[i].role = ATTACKER;
						robots[i].vmax = robots[i].vdefault;
						atk = i;
						break;
					case DEFENDER:
						robots[i].role = GOALKEEPER;
						robots[i].vmax = robots[i].vdefault;
						gk = i;
						break;
					case ATTACKER:
						robots[i].role = DEFENDER;
						robots[i].vmax = robots[i].vdefault;
						def = i;
						break;
					default:
						break;
				}
			}
		}
	}

	// devolve o vetor de robots com as alterações
	*pRobots = robots;
} // get_targets

void Strategy::overmind() {
	cv::Point goal = cv::Point(COORD_GOAL_ATK_FRONT_X, COORD_GOAL_MID_Y);
	double m1 = double(goal.y - Ball.y) / double(goal.x - Ball.x);
	double m2 = double(robots[atk].position.y - Ball.y) / double(robots[atk].position.x - Ball.x);

	double phi = atan((m2 - m1) / (1 + m2 * m1));

	if (robots[atk].cmdType == SPEED || robots[atk].status == CORNER_STATE) atk_mindcontrol = false;

	if (atk_mindcontrol) {
		robots[atk].cmdType = VECTOR;
		robots[atk].transAngle = lock_angle;
		robots[atk].vmax = 1.4;
		timeout++;
		if (timeout >= 30 || distance(robots[atk].position, Ball) > ABS_ROBOT_SIZE * 1.5) {
			timeout = 0;
			atk_mindcontrol = false;
			robots[atk].vmax = robots[atk].vdefault;
		}
	}

	if ((distance(robots[atk].position, Ball) < ABS_ROBOT_SIZE * 2) && (abs(phi) * 180 / PI < 20) &&
		(Ball.x > robots[atk].position.x) &&
		((((robots[atk].orientation - atan(m1)) * 180 / PI) < 20 &&
		  ((robots[atk].orientation - atan(m1)) * 180 / PI) > -20) ||
		 (((robots[atk].orientation - atan(m1)) * 180 / PI) < -165 ||
		  ((robots[atk].orientation - atan(m1)) * 180 / PI) > 165))
			) {

		if (!atk_mindcontrol) {
			lock_angle = atan2(double(robots[atk].position.y - Ball.y), -double(robots[atk].position.x - Ball.x));
		}

		atk_mindcontrol = true;
	}

	//Defender Overmind-------------------------
	if (def_mindcontrol) {

		cv::Point v = cv::Point(Ball.x - Ball_Est.x, Ball.y - Ball_Est.y);
		double module = sqrt(pow(v.x, 2) + pow(v.y, 2));
		double approach = distance(Ball, Ball_Est) *
						  (distance(Ball, robots[atk].position) / (COORD_MID_FIELD_X + COORD_MID_FIELD_X / 4));
		robots[atk].target.x = static_cast<int>(Ball.x - (v.x / module) * approach);
		robots[atk].target.y = static_cast<int>(Ball.y - (v.y / module) * approach);
		robots[atk].cmdType = VECTOR;
		position_to_vector(atk);
		robots[atk].vmax = 1.4;
		timeout++;
		if (timeout >= 30 || Ball.x < robots[atk].position.x) {
			timeout = 0;
			def_mindcontrol = false;
			robots[atk].vmax = robots[atk].vdefault;
		}
	}

	// Goalkeeper overmind-------------------
	if (!full_transition_enabled && robots[atk].position.x < COORD_BOX_DEF_X + ABS_ROBOT_SIZE / 2 &&
		robots[atk].position.y < COORD_BOX_DWN_Y + ABS_ROBOT_SIZE / 2 &&
		robots[atk].position.y > COORD_BOX_UP_Y - ABS_ROBOT_SIZE / 2) {

		robots[gk].fixedPos = true;
		robots[gk].target = cv::Point(static_cast<int>(COORD_BOX_DEF_X + ABS_ROBOT_SIZE * 1.5), COORD_GOAL_MID_Y);
		position_to_vector(gk);
		fixed_position_check(gk);
	}
}

void Strategy::Transitions() {
	/**** SITUAÇÕES DE TROCA ****/

	cv::Point goal = cv::Point(COORD_GOAL_ATK_FRONT_X, COORD_GOAL_MID_Y);

	double ballGoal = double(goal.y - Ball_Est.y) / double(goal.x - Ball_Est.x);
	double defBall = double(robots[def].position.y - Ball_Est.y) / double(robots[def].position.x - Ball_Est.x);
	double phiDef = atan((defBall - ballGoal) / (1 + defBall * ballGoal));

	// troca se está travado
	if (robots[gk].position.x >= COORD_MID_FIELD_X) {
		if (!atk_mindcontrol) {
			lock_angle = atan2(double(robots[atk].position.y - Ball.y), -double(robots[atk].position.x - Ball.x));
		}
		atk_mindcontrol = true;
	} else if (player_blocked()) {
		// verifica distancias e seta roles
		transition_mindcontrol = true;
	}
		// faz meio que um cruzamento
	else if (Ball.y > COORD_GOAL_UP_Y && Ball.y < COORD_GOAL_DWN_Y &&
			 Ball.x > corner_atk_limit && distance(robots[atk].position, Ball) > ABS_ROBOT_SIZE * 1.5) {
		half_transition = true;
		def_mindcontrol = true;
	}
		// se a bola tá bem atrás do atacante mas tá na frente do defensor
	else if (danger_zone_1 && Ball.x < robots[atk].position.x - transition_y_distance &&
			 robots[atk].status != CORNER_STATE) {
		half_transition = true;
	}
		// se a bola tá atrás do atacante e está atrás do defensor, goleiro tora o pau
	else if (danger_zone_2) {
		if (Ball.x < COORD_MID_FIELD_X / 2 && !offensive_adv() && Ball.x >
																  robots[def].position.x) { // senão se a bola está atrás do def e atk e não tem adv com a bola, full transition
			full_transition = true;
		}
	}
		// troca caso o angulo do defensor pro gol for bom
	else if ((abs(phiDef) * 180 / PI < 20) &&
			 (Ball.x > robots[def].position.x) &&
			 distance(robots[def].position, Ball) < distance(robots[atk].position, Ball)) {

		half_transition = true;
	}
}

void Strategy::set_flags() {
	danger_zone_1 = false;
	danger_zone_2 = false;
	half_transition = false;
	full_transition = false;
	if (Ball.x > COORD_MID_FIELD_X / 3) {
		full_transition_enabled = true;
		half_transition_enabled = true;
	} else {
		half_transition_enabled = false;
	}
	if (Ball.x < robots[atk].position.x - ABS_ROBOT_SIZE * 2 && Ball.x > robots[def].position.x) {
		danger_zone_1 = true;
	} else if (Ball.x < robots[atk].position.x && Ball.x < robots[def].position.x) {
		danger_zone_2 = true;
	}
}

void Strategy::fixed_position_check(int i) { // Para posição fixa
	if (robots[i].fixedPos && (robots[i].cmdType == VECTOR || robots[i].cmdType == POSITION)) {
		robots[i].vmax = robots[i].vdefault *
						 pow(distance(robots[i].position, robots[i].target) / (ABS_GOAL_TO_GOAL_WIDTH / 4), 2);
		if (robots[i].vmax > robots[i].vdefault) robots[i].vmax = robots[i].vdefault;
		if (robots[i].vmax < 0.3) robots[i].vmax = 0.3;

		if (distance(robots[i].target, robots[i].position) <= fixed_pos_distance) {
			robots[i].target = robots[i].position;
			robots[i].vmax = 0;
		}
	}
}

bool Strategy::offensive_adv() {
	bool isDangerous = false;
	for (int i = 0; i < 3; i++) {
		// se o adversário mais próximo da bola está a pelo menos duas vezes a distância de nosso gk a bola, não é perigoso
		if (distance(adv[i], Ball) < distance(robots[gk].position, Ball) * 2) {
			isDangerous = true;
			break;
		}
	}
	return isDangerous;
}

void Strategy::collision_check(int i) {
	if (!robots[i].fixedPos && robots[i].role != GOALKEEPER && robots[i].status != CORNER_STATE) {
		if (distance(robots[i].position, past_position[i]) <= collision_radius) {
			collision_count[i]++;
		} else {
			get_past(i);
			collision_count[i] = 0;
		}
		if (collision_count[i] >= max_collision_count) {
			robots[i].transAngle = atan2(sin(past_transangle[i] + PI), cos(past_transangle[i] + PI));
		}
	} else if (robots[i].fixedPos || robots[i].role == GOALKEEPER) {
		if (distance(robots[i].position, past_position[i]) <= collision_radius &&
			distance(robots[i].position, robots[i].target) > fixed_pos_distance) {
			collision_count[i]++;
		} else {
			get_past(i);
			collision_count[i] = 0;
		}
		if (collision_count[i] >= max_collision_count) {
			robots[i].transAngle = atan2(sin(past_transangle[i] + PI), cos(past_transangle[i] + PI));
		}
	}
}

bool Strategy::player_blocked() {
	bool at_least_one_blocked = false;

	// decrementa o timer se tiver
	if (transition_overmind_timeout > 0) {
		transition_overmind_timeout--;
		return false;
	}

	// verifica se os robôs estão colidindo e incrementa o tempo
	for (int i = 0; i < robots.size(); i++) {
		for (int j = i + 1; j < robots.size(); j++) {
			if (distance(robots[i].position, robots[j].position) < ABS_ROBOT_SIZE * 1.5) {
				at_least_one_blocked = true;
			}
		}
	}

	if (!at_least_one_blocked) {
		frames_blocked = 0;
	} else {
		frames_blocked++;
		if (frames_blocked > 15) {
			transition_overmind_timeout = 60; // dois segundos
			return true;
		}
	}

	return false;
}

void Strategy::set_role(int i, int role) {
	robots[i].status = NORMAL_STATE;
	robots[i].role = role;
	robots[i].vmax = robots[i].vdefault;
}

bool Strategy::is_near(int i, cv::Point point) {
	return distance(robots[i].position, point) < fixed_pos_distance;
}

void Strategy::position_to_vector(int i) {
	robots[i].transAngle = angle_atan2(robots[i].position, robots[i].target);
}

void Strategy::go_to_x(int i, double x) {
	robots[i].target.x = static_cast<int>(x);
}

void Strategy::go_to_y(int i, double y) {
	robots[i].target.y = static_cast<int>(y);
}

void Strategy::go_to(int i, cv::Point point) {
	robots[i].target = point;
}

double Strategy::look_at_ball(int i) {
	double target_angle = angle_atan2(Ball, robots[i].position); // ângulo da bola em relação ao robô
	robots[i].cmdType = ORIENTATION;
	robots[i].targetOrientation = target_angle;
}

double Strategy::potField(int robot_index, cv::Point goal, int behavior = BALL_NOT_OBS) {
	robots[robot_index].using_pot_field = true;
	float gain_rep = 10;
	float gain_att = 1;
	float rep_radius = 0.2;
	float ball_rep_radius = 0.3;
	float rot_angle = 0;
	ball_angle = 0;
	ball_mag = 0;
	pot_thetaX = 0;
	pot_thetaY = 0;
	pot_theta = 0;
	pot_goalTheta = 0;
	pot_goalMag = 0;
	for (int i = 0; i < 3; i++) {
		if (distance_meters(robots[robot_index].position, adv[i]) > 0 &&
			distance_meters(robots[robot_index].position, adv[i]) < rep_radius) {
			pot_magnitude[i] =
					gain_rep * pow(1 / (distance_meters(robots[robot_index].position, adv[i]) - 1 / rep_radius), 2) / 2;
			pot_angle[i] = atan2(double(robots[robot_index].position.y - adv[i].y),
								 -double(robots[robot_index].position.x - adv[i].x));

			if (pot_rotation_decision(robot_index, goal, adv[i]) == 0)
				pot_magnitude[i] = 0;
			else if (pot_rotation_decision(robot_index, goal, adv[i]) > 0) {
				pot_angle[i] = atan2(sin(pot_angle[i] + PI / 2), cos(pot_angle[i] + PI / 2));
				//	printf("HORARIO ");
			} else {
				pot_angle[i] = atan2(sin(pot_angle[i] + 3 * PI / 2), cos(pot_angle[i] + 3 * PI / 2));

				//printf("ANTI-HORARIO ");
			}
			//printf("adv[%d] x %d, y %d| magnitude [%d] %f | angle[%d] %f\n",i,adv[i].x,adv[i].y,i,pot_magnitude[i],i,pot_angle[i]*180/PI);
		} else {
			pot_magnitude[i] = 0;
			pot_angle[i] = 0;
		}
	}
	int j = 3;
	for (int i = 0; i < 3; i++) {
		if (i != robot_index) {
			if (distance_meters(robots[robot_index].position, robots[i].position) < rep_radius) {
				pot_magnitude[j] = gain_rep * pow(1 /
												  (distance_meters(robots[robot_index].position, robots[i].position) -
												   1 / rep_radius), 2) / 2;
				pot_angle[j] = atan2(double(robots[robot_index].position.y - robots[i].position.y),
									 -double(robots[robot_index].position.x - robots[i].position.x));
				if (pot_rotation_decision(robot_index, goal, robots[i].position) == 0)
					pot_magnitude[j] = 0;
				else if (pot_rotation_decision(robot_index, goal, robots[i].position) > 0) {
					pot_angle[j] = atan2(sin(pot_angle[j] + PI / 2), cos(pot_angle[j] + PI / 2));
					//	printf("HORARIO ");

				} else {
					pot_angle[j] = atan2(sin(pot_angle[j] + 3 * PI / 2), cos(pot_angle[j] + 3 * PI / 2));
					//	printf("ANTI-HORARIO ");
				}
				//	printf("robots[%d] x %d, y %d|magnitude [%d] %f | angle[%d] %f\n",i,robots[i].position.x,robots[i].position.y,j,pot_magnitude[j],j,pot_angle[j]*180/PI);
			} else {
				pot_magnitude[j] = 0;
				pot_angle[j] = 0;
			}

			j++;
		}
	}

	switch (behavior) {
		case BALL_IS_OBS:
			if (distance_meters(robots[robot_index].position, Ball) > 0 &&
				distance_meters(robots[robot_index].position, Ball) < ball_rep_radius) {

				ball_mag = (gain_rep *
							pow(1 / (distance_meters(robots[robot_index].position, Ball) - 1 / (ball_rep_radius)), 2) /
							2);

				ball_angle = atan2(double(robots[robot_index].position.y - Ball.y),
								   -double(robots[robot_index].position.x - Ball.x));
				if (pot_rotation_decision(robot_index, goal, Ball) == 0)
					ball_mag = 0;
				else if (pot_rotation_decision(robot_index, goal, Ball) > 0) {
					ball_angle = atan2(sin(ball_angle + PI / 2), cos(ball_angle + PI / 2));
				} else {
					ball_angle = atan2(sin(ball_angle + 3 * PI / 2), cos(ball_angle + 3 * PI / 2));
				}
				pot_thetaY += ball_mag * sin(ball_angle);
				pot_thetaX += ball_mag * cos(ball_angle);
			} else {
				ball_mag = 0;
				ball_angle = 0;
			}
			break;

		case BALL_ONLY_OBS:
			for (double &i : pot_magnitude) {
				i = 0;
			}
			if (distance_meters(robots[robot_index].position, Ball) > 0 &&
				distance_meters(robots[robot_index].position, Ball) < ball_rep_radius) {

				ball_mag = (gain_rep *
							pow(1 / (distance_meters(robots[robot_index].position, Ball) - 1 / ball_rep_radius), 2) /
							2);

				ball_angle = atan2(double(robots[robot_index].position.y - Ball.y),
								   -double(robots[robot_index].position.x - Ball.x));
				if (pot_rotation_decision(robot_index, goal, Ball) == 0)
					ball_mag = 0;
				else if (pot_rotation_decision(robot_index, goal, Ball) > 0) {
					ball_angle = atan2(sin(ball_angle + PI / 2), cos(ball_angle + PI / 2));
				} else {
					ball_angle = atan2(sin(ball_angle + 3 * PI / 2), cos(ball_angle + 3 * PI / 2));
				}
				pot_thetaY = ball_mag * sin(ball_angle);
				pot_thetaX = ball_mag * cos(ball_angle);
			} else {
				ball_mag = 0;
				ball_angle = 0;
			}
			break;

		case NO_OBS:
			for (double &i : pot_magnitude) {
				i = 0;
			}
			break;

		case BALL_NOT_OBS:
			break;
		default:
			break;
	}

	for (int i = 0; i < 5; i++) {
		pot_thetaY += pot_magnitude[i] * sin(pot_angle[i]);
		pot_thetaX += pot_magnitude[i] * cos(pot_angle[i]);
	}

	pot_goalTheta = atan2(double(robots[robot_index].position.y - goal.y),
						  -double(robots[robot_index].position.x - goal.x));
	pot_goalMag = gain_att * pow(distance_meters(robots[robot_index].position, goal), 2) / 2;

	pot_thetaY += pot_goalMag * sin(pot_goalTheta);
	pot_thetaX += pot_goalMag * cos(pot_goalTheta);
	pot_theta = atan2(pot_thetaY, pot_thetaX);
	return pot_theta;
}

int Strategy::pot_rotation_decision(int robot_index, cv::Point goal, cv::Point obst) {
	int HORARIO = 1;
	int ANTI_HORARIO = -1;
	bool goal_ahead = robots[robot_index].position.x > obst.x && obst.x > goal.x;
	bool goal_behind = robots[robot_index].position.x < obst.x && obst.x < goal.x;

	if (obst.y < ABS_ROBOT_SIZE * 2) {
		if (goal_ahead) return HORARIO;
		if (goal_behind) return ANTI_HORARIO;
	} else if (obst.y > (ABS_FIELD_HEIGHT - ABS_ROBOT_SIZE * 2)) {
		if (goal_ahead) return ANTI_HORARIO;
		if (goal_behind) return HORARIO;
	}
	double angle_Obst_Robot = atan2(double(obst.y - robots[robot_index].position.y),
									-double(obst.x - robots[robot_index].position.x));
	double angle_Obst_Goal = atan2(double(obst.y - goal.y),
								   -double(obst.x - goal.x));
	if (angle_Obst_Robot > 0 && angle_Obst_Robot <= PI / 2) { // ROBO NO PRIMEIRO QUAD

		if (angle_Obst_Goal > 0 && angle_Obst_Goal <= PI / 2) { // OBJETIVO NO PRIMEIRO QUAD
			return 0;
		} else if (angle_Obst_Goal > PI / 2 && angle_Obst_Goal <= PI) { // OBJETIVO NO SEGUNDO QUAD
			return ANTI_HORARIO;
		} else if (angle_Obst_Goal > -PI && angle_Obst_Goal <= -PI / 2) {// OBJETIVO NO TERCEIRO QUAD
			return HORARIO;
		} else { // OBJETIVO NO QUARTO QUAD
			if (fabs(angle_Obst_Robot) + fabs(angle_Obst_Goal) < PI)
				return HORARIO;
			else
				return ANTI_HORARIO;
		}
	} else if (angle_Obst_Robot > PI / 2 && angle_Obst_Robot <= PI) { // ROBO NO SEGUNDO QUAD

		if (angle_Obst_Goal > 0 && angle_Obst_Goal <= PI / 2) { // OBJETIVO NO PRIMEIRO QUAD
			return HORARIO;
		} else if (angle_Obst_Goal > PI / 2 && angle_Obst_Goal <= PI) { // OBJETIVO NO SEGUNDO QUAD
			return 0;
		} else if (angle_Obst_Goal > -PI && angle_Obst_Goal <= -PI / 2) {// OBJETIVO NO TERCEIRO QUAD
			return ANTI_HORARIO;
		} else { // OBJETIVO NO QUARTO QUAD
			if (fabs(angle_Obst_Robot) + fabs(angle_Obst_Goal) < PI)
				return HORARIO;
			else
				return ANTI_HORARIO;
		}
	} else if (angle_Obst_Robot > -PI && angle_Obst_Robot <= -PI / 2) {// ROBO NO TERCEIRO QUAD

		if (angle_Obst_Goal > 0 && angle_Obst_Goal <= PI / 2) { // OBJETIVO NO PRIMEIRO QUAD
			if (fabs(angle_Obst_Robot) + fabs(angle_Obst_Goal) < PI)
				return ANTI_HORARIO;
			else
				return HORARIO;
		} else if (angle_Obst_Goal > PI / 2 && angle_Obst_Goal <= PI) { // OBJETIVO NO SEGUNDO QUAD
			return HORARIO;
		} else if (angle_Obst_Goal > -PI && angle_Obst_Goal <= -PI / 2) {// OBJETIVO NO TERCEIRO QUAD
			return 0;
		} else { // OBJETIVO NO QUARTO QUAD

			return ANTI_HORARIO;
		}
	} else { // ROBO NO QUARTO QUAD

		if (angle_Obst_Goal > 0 && angle_Obst_Goal <= PI / 2) { // OBJETIVO NO PRIMEIRO QUAD
			return ANTI_HORARIO;
		} else if (angle_Obst_Goal > PI / 2 && angle_Obst_Goal <= PI) { // OBJETIVO NO SEGUNDO QUAD
			if (fabs(angle_Obst_Robot) + fabs(angle_Obst_Goal) < PI)
				return ANTI_HORARIO;
			else
				return HORARIO;
		} else if (angle_Obst_Goal > -PI && angle_Obst_Goal <= -PI / 2) {// OBJETIVO NO TERCEIRO QUAD
			return HORARIO;
		} else { // OBJETIVO NO QUARTO QUAD

			return 0;
		}
	}
}

void Strategy::spin_anti_clockwise(int i, double speed = 0.8) {
	robots[i].cmdType = SPEED;
	robots[i].vmax = speed;
	robots[i].Vr = robots[i].vmax;
	robots[i].Vl = -robots[i].vmax;
}

void Strategy::spin_clockwise(int i, double speed = 0.8) {
	robots[i].cmdType = SPEED;
	robots[i].vmax = speed;
	robots[i].Vr = -robots[i].vmax;
	robots[i].Vl = robots[i].vmax;
}

void Strategy::def_wait(int i) {
	robots[i].fixedPos = true;
	if (distance(robots[i].position, robots[i].target) <= fixed_pos_distance) {
		look_at_ball(i);
	}
}

void Strategy::fuzzy_init() {
	cout << "init " << endl;

	controller.importRules("cc/Fuzzy/RULES_VSS2017.txt");
	cout << "rules " << endl;

	controller.defineVariables("cc/Fuzzy/Membership_VSS2017.txt");
	cout << "variables " << endl;
}

int Strategy::Fuzzy_Troca() {

	vector<float> input;
	// cout << "vector " << endl;
	if (Ball.x < 0) Ball.x = 0;
	if (Ball.y < 0) Ball.y = 0;
	if (robots[atk].position.x < 0) robots[atk].position.x = 0;
	if (robots[atk].position.y < 0) robots[atk].position.y = 0;
	if (robots[def].position.x < 0) robots[def].position.x = 0;
	if (robots[def].position.y < 0) robots[def].position.y = 0;
	//
	// input.push_back(float(Ball.x * 1.7/(ABS_GOAL_TO_GOAL_WIDTH)));
	// input.push_back(float(Ball.y * 1.3/ABS_FIELD_HEIGHT));
	// input.push_back(float(robots[atk].position.x * 1.7/(ABS_GOAL_TO_GOAL_WIDTH)));
	// input.push_back(float(distance(robots[atk].position, Ball) * 1.7/(ABS_GOAL_TO_GOAL_WIDTH)));
	// input.push_back(float(robots[def].position.x * 1.7/(ABS_GOAL_TO_GOAL_WIDTH)));
	// input.push_back(float(distance(robots[def].position, Ball) * 1.7/(ABS_GOAL_TO_GOAL_WIDTH)));

	// cout << "var " << endl;
	// cout << "XB " << input[0]<<endl;
	// cout << "YB " << input[1]<<endl;
	// cout << "XA " << input[2]<<endl;
	// cout << "DAB " << input[3]<<endl;
	// cout << "XD " << input[4]<<endl;
	// cout << "DDB " << input[5]<<endl;

	input.push_back(float(distance(robots[gk].position, Ball) * 1.7 / (ABS_GOAL_TO_GOAL_WIDTH)));
	input.push_back(float(distance(robots[atk].position, Ball) * 1.7 / (ABS_GOAL_TO_GOAL_WIDTH)));
	input.push_back(float(distance(robots[def].position, Ball) * 1.7 / (ABS_GOAL_TO_GOAL_WIDTH)));

	vector<float> out = controller.ControladorFuzzy(input);
	std::cout << "resultado: " << out[0] << std::endl;

	if (out.size() > 0) {
		switch (int(out[0])) {
			case 0:
				//	std::cout << "NÃO TROCA NADA, PORRA: " << out[0] << std::endl;
				break;
			case 1:
				//	std::cout << "TROCA A PORRA TODA: " << out[0] << std::endl;
				break;
			case 2:
				//	std::cout << "TROCA ATK DEF: " << out[0] << std::endl;
				break;
		}
	}
	return out[0];
}

void Strategy::pot_field_around(int i) {
	robots[i].using_pot_field = true;
	cv::Point targets_temp;
	cv::Point goal = cv::Point(COORD_GOAL_ATK_FRONT_X, COORD_GOAL_MID_Y);
	double m1 = double(goal.y - Ball_Est.y) / double(goal.x - Ball_Est.x);
	double m2 = double(robots[i].position.y - Ball_Est.y) / double(robots[i].position.x - Ball_Est.x);
	double ball_goal = distance(Ball_Est, goal);
	double r1 = ball_goal + max_approach;

	double dist_up = distance(Ball_Est, cv::Point(COORD_GOAL_ATK_FRONT_X, COORD_GOAL_UP_Y));
	double dist_dwn = distance(Ball_Est, cv::Point(COORD_GOAL_ATK_FRONT_X, COORD_GOAL_DWN_Y));

	double beta = acos((pow(dist_up, 2) + pow(dist_dwn, 2) - pow(ABS_GOAL_SIZE_Y, 2)) / (2 * dist_up * dist_dwn));

	cv::Point v = cv::Point(goal.x - Ball_Est.x, goal.y - Ball_Est.y);
	double module = sqrt(pow(v.x, 2) + pow(v.y, 2));

	double phi = atan((m2 - m1) / (1 + m2 * m1));

	targets_temp.x = static_cast<int>(Ball_Est.x - double(v.x / module) * max_approach);
	targets_temp.y = static_cast<int>(Ball_Est.y - double(v.y / module) * max_approach);

	//crop
	if (targets_temp.y < 0) targets_temp.y = 0;
	if (targets_temp.y > ABS_FIELD_HEIGHT) targets_temp.y = ABS_FIELD_HEIGHT;
	if (targets_temp.x < 0) targets_temp.x = 0;
	if (targets_temp.x > COORD_GOAL_ATK_FRONT_X) targets_temp.x = COORD_GOAL_ATK_FRONT_X;

	robots[i].target = targets_temp;
	robots[i].transAngle = potField(i, targets_temp, BALL_ONLY_OBS);
	if (robots[i].position.x > Ball.x + ABS_ROBOT_SIZE) {
		action1 = false;
	}

	if (is_near(i, targets_temp) || abs(phi) < beta / 2 || action1) {

		v = cv::Point(robots[i].position.x - Ball.x, robots[i].position.y - Ball.y);

		robots[i].transAngle = atan2(double(robots[i].position.y - Ball.y), -double(robots[i].position.x - Ball.x));
		action1 = true;
	}
}

void Strategy::crop_targets(int i) {
	if (robots[i].target.y < 0) robots[i].target.y = 0;
	if (robots[i].target.y > ABS_FIELD_HEIGHT) robots[i].target.y = ABS_FIELD_HEIGHT;
	if (robots[i].target.x < 0) robots[i].target.x = 0;
	if (robots[i].target.x > COORD_GOAL_ATK_FRONT_X) robots[i].target.x = COORD_GOAL_ATK_FRONT_X;

	if (robots[i].target.x < COORD_BOX_DEF_X && robots[i].target.y > COORD_BOX_UP_Y &&
		robots[i].target.y < COORD_GOAL_DWN_Y) {
		robots[i].target.x = def_corner_line;
	}
}

void Strategy::smart_ball(int i, int max_distance) {
	cv::Point v = cv::Point(Ball.x - Ball_Est.x, Ball.y - Ball_Est.y); //vetor a estimativa à bola
	double vector_module = sqrt(pow(v.x, 2) + pow(v.y, 2)); //módulo do vetor
	double approach = distance(Ball, Ball_Est) * (distance(Ball, robots[atk].position) /
												  (max_distance)); // porcentagem que a distancia do roboà bola representa da máxima
	robots[i].target.x = static_cast<int>(Ball.x - (v.x / vector_module) * approach);
	robots[i].target.y = static_cast<int>(Ball.y - (v.y / vector_module) * approach);
}

void Strategy::atk_routine(int i) {

	switch (robots[i].status) {

		case NORMAL_STATE:
			if (Ball.x > corner_atk_limit && (Ball.y > COORD_GOAL_DWN_Y || Ball.y < COORD_GOAL_UP_Y)) {
				robots[i].status = CORNER_STATE; // bola no canto
			} else if (Ball.x < def_corner_line) {
				robots[i].status = DEF_CORNER_STATE;
			} else if (Ball.y > ABS_FIELD_HEIGHT - ABS_ROBOT_SIZE * 1.5 || Ball.y < ABS_ROBOT_SIZE * 1.5) {
				robots[i].status = SIDEWAYS;
			}
			pot_field_around(i);
			crop_targets(i);

			break;

		case SIDEWAYS:
			if (Ball.y > ABS_FIELD_HEIGHT - ABS_ROBOT_SIZE * 1.5) {
				if (distance(robots[i].position, Ball) > ABS_ROBOT_SIZE) {
					go_to(i, Ball);
				} else if (Ball.x >= robots[i].position.x + ABS_ROBOT_SIZE / 2) {
					// smart_ball(i, COORD_GOAL_MID_Y);
					go_to(i, Ball);
				} else {
					spin_anti_clockwise(i);
				}
				if (Ball.x < robots[i].position.x) {
					go_to(i, cv::Point(Ball.x - max_approach, Ball.y - max_approach / 2));
				}
			} else if (Ball.y < ABS_ROBOT_SIZE * 1.5) {
				if (distance(robots[i].position, Ball) > ABS_ROBOT_SIZE) {
					go_to(i, Ball);
				} else if (Ball.x >= robots[i].position.x + ABS_ROBOT_SIZE / 2) {
					// smart_ball(i, COORD_GOAL_MID_Y);
					go_to(i, Ball);
				} else {
					spin_clockwise(i);
				}
				if (Ball.x < robots[i].position.x) {
					go_to(i, cv::Point(Ball.x - max_approach, Ball.y - max_approach / 2));
				}
			} else {
				robots[i].status = NORMAL_STATE; // voltar ao estado normal
			}
			break;

		case CORNER_STATE:
			if (Ball.x > corner_atk_limit) {
				if (Ball.y < COORD_GOAL_MID_Y) { // acima ou abaixo do gol, para saber para qual lado girar
					if (Ball.y <
						COORD_GOAL_UP_Y) { // acima ou abaixo da trave, escolher o giro para levar a bola para o gol ou para faze-la entrar
						if (Ball.x > robots[i].position.x &&
							distance(Ball, robots[i].position) < ABS_ROBOT_SIZE)
							spin_clockwise(i); // giro que leva a bola ao gol
						else
							go_to(i,
								  Ball); // se Ball.y < robots[i].position.y -> faz a bola travar no canto pra girar certo
					} else {
						if (Ball.y > robots[i].position.y &&
							distance(Ball, robots[i].position) < ABS_ROBOT_SIZE)
							spin_anti_clockwise(i); // giro para faze-la entrar
						else go_to(i, Ball);
					}
				} else {
					if (Ball.y > COORD_GOAL_DWN_Y) {
						if (Ball.x > robots[i].position.x &&
							distance(Ball, robots[i].position) < ABS_ROBOT_SIZE)
							spin_anti_clockwise(i);
						else
							go_to(i,
								  Ball); // se Ball.y > robots[i].position.y -> faz a bola travar no canto pra girar certo
					} else {
						if (Ball.y > robots[i].position.y &&
							distance(Ball, robots[i].position) < ABS_ROBOT_SIZE)
							spin_clockwise(i); // giro para faze-la entrar
						else go_to(i, Ball);
					}
				}
			} else {
				robots[i].status = NORMAL_STATE; // voltar ao estado normal
			}
			break;

		case DEF_CORNER_STATE:
			robots[i].fixedPos = true;
			if (Ball.x < def_corner_line) {
				if (Ball.y > COORD_GOAL_MID_Y) {
					go_to(i, cv::Point(goalie_line, COORD_BOX_DWN_Y + ABS_ROBOT_SIZE));
					if (distance(Ball, robots[i].position) < ABS_ROBOT_SIZE &&
						Ball.y > robots[i].position.y)
						spin_anti_clockwise(i);
					//if(distance(robots[i].position, robots[i].target) <= fixed_pos_distance) robots[i].target = Ball;
					if (is_near(i, robots[i].target) && Ball.y > robots[i].position.y) {
						go_to(i, Ball);
					}
				} else {
					go_to(i, cv::Point(goalie_line, COORD_BOX_DWN_Y - ABS_ROBOT_SIZE));
					if (distance(Ball, robots[i].position) < ABS_ROBOT_SIZE &&
						Ball.y < robots[i].position.y)
						spin_clockwise(i);
					//if(distance(robots[i].position, robots[i].target) <= fixed_pos_distance) robots[i].target = Ball;
					if (is_near(i, robots[i].target) && Ball.y < robots[i].position.y) {
						go_to(i, Ball);
					}
				}
			} else {
				robots[i].status = NORMAL_STATE;
			}
			crop_targets(i);
			break;
		default:
			break;
	}
}

void Strategy::def_routine(int i) {
	switch (robots[i].status) {
		case NORMAL_STATE:
			if (Ball_Est.x > corner_atk_limit) {
				int y_pos = Ball.y > COORD_GOAL_MID_Y ? COORD_GOAL_UP_Y : COORD_GOAL_DWN_Y;
				go_to(i, cv::Point(COORD_MID_FIELD_X + COORD_MID_FIELD_X / 4, y_pos));
				//robots[i].target = cv::Point(COORD_MID_FIELD_X + COORD_MID_FIELD_X/2, COORD_GOAL_MID_Y);

				//robots[i].transAngle = potField(i, cv::Point(COORD_MID_FIELD_X, COORD_GOAL_MID_Y), BALL_IS_OBS);
			} else if (Ball_Est.x > COORD_MID_FIELD_X) {
				go_to(i, cv::Point(COORD_MID_FIELD_X - COORD_MID_FIELD_X / 4, COORD_GOAL_MID_Y));
				//robots[i].transAngle = potField(i, cv::Point(COORD_MID_FIELD_X, COORD_GOAL_MID_Y), BALL_IS_OBS);
			} else {
				// fixa uma posição em x
				int x_pos = COORD_BOX_DEF_X + ABS_ROBOT_SIZE;

				int ballTarget = Ball.y > COORD_GOAL_MID_Y ? COORD_GOAL_UP_Y : COORD_GOAL_DWN_Y;
				double m = double(Ball.y - ballTarget) / double(Ball.x - ABS_ROBOT_SIZE);
				int y_proj = static_cast<int>(Ball.y - m * (Ball.x - x_pos));

				// faz o crop do alvo se ele for muito discrepante (se a bola se aproximar muito do gol)
				if (y_proj > COORD_GOAL_DWN_Y)
					y_proj = COORD_GOAL_DWN_Y;
				else if (y_proj < COORD_GOAL_UP_Y)
					y_proj = COORD_GOAL_UP_Y;

				go_to(i, cv::Point(x_pos, y_proj));

				//robots[i].target = cv::Point(COORD_BOX_DEF_X + ABS_ROBOT_SIZE * 2, COORD_GOAL_MID_Y);
				//robots[i].transAngle = potField(i, cv::Point(COORD_MID_FIELD_X/2, COORD_GOAL_MID_Y), BALL_IS_OBS);
			}
			def_wait(i);
			break;

		case STEP_BACK:
			break;

		case TRANSITION_STATE:
			break;
	}
}

void Strategy::gk_routine(int i) {
	switch (robots[i].status) {
		case NORMAL_STATE:
			// robots[i].fixedPos = true;
			robots[i].cmdType = POSITION;
			go_to(i, cv::Point(goalie_line, Ball.y));
			//if bola com velocidade && vindo pra defesa && fora da area

			if (distance(Ball, Ball_Est) > ABS_ROBOT_SIZE && Ball.x > Ball_Est.x && (Ball.x > COORD_BOX_DEF_X)) {
				double m = double(Ball.y - Ball_Est.y) / double(Ball.x - Ball_Est.x);
				double pred_y = Ball.y - m * (Ball.x - (goalie_line + ABS_ROBOT_SIZE));

				if (pred_y > COORD_GOAL_DWN_Y) pred_y = COORD_GOAL_DWN_Y;
				if (pred_y < COORD_GOAL_UP_Y) pred_y = COORD_GOAL_UP_Y;

				if (Ball.x > COORD_MID_FIELD_X) {
					go_to(i, cv::Point(goalie_line, static_cast<int>(pred_y)));
				} else {
					cv::Point v = cv::Point(goalie_line - goalie_line,
											Ball.y - pred_y); // vetor da predição à projeção (na linha do gol)
					double vector_module = sqrt(pow(v.x, 2) + pow(v.y, 2));//modulo

					double approach = abs(Ball.y - pred_y) * (Ball.x - (robots[i].position.x + ABS_ROBOT_SIZE)) /
									  (COORD_MID_FIELD_X - robots[i].position.x);

					go_to(i, cv::Point(goalie_line, static_cast<int>(pred_y + (v.y / vector_module) * approach)));
				}
			} else {
				go_to(i, cv::Point(goalie_line, Ball.y));
			}

			if (robots[i].target.y > COORD_GOAL_DWN_Y) go_to(i, cv::Point(goalie_line, COORD_GOAL_DWN_Y));

			if (robots[i].target.y < COORD_GOAL_UP_Y) go_to(i, cv::Point(goalie_line, COORD_GOAL_UP_Y));

			robots[i].vmax = 0.3 + ((robots[i].vdefault - 0.3) * (distance(robots[i].position, robots[i].target)) /
									ABS_GOAL_SIZE_Y);
			if (robots[i].vmax > robots[i].vdefault) robots[i].vmax = robots[i].vdefault;
			if (robots[i].vmax < 0.35) robots[i].vmax = 0.35;
			// fixed_lookup(i);



			if (distance(robots[i].position, Ball) < ABS_ROBOT_SIZE * 1.3) {
				if (Ball.y <= COORD_GOAL_UP_Y) {
					spin_clockwise(i, 0.8);
				} else if (Ball.y >= COORD_GOAL_DWN_Y) {
					spin_anti_clockwise(i, 0.8);
				}
			}
			if (distance(robots[i].position, Ball) < ABS_ROBOT_SIZE) {
				if (Ball.y <= robots[i].position.y) {
					spin_clockwise(i, 0.8);
				} else if (Ball.y >= robots[i].position.y) {
					spin_anti_clockwise(i, 0.8);
				}
			}
			// chute pra tirar a bola da defesa
			/*if(abs(Ball.x - robots[i].position.x) <= ABS_ROBOT_SIZE && Ball.y < COORD_BOX_DWN_Y && Ball.y > COORD_BOX_UP_Y) {
				if (Ball.y <= robots[i].position.y) {
					robots[i].target.y = 0;
					robots[i].target.x = goalie_line + ABS_ROBOT_SIZE/2;
					robots[i].vmax = 0.9;
				} else {
					robots[i].target.y = 480;
					robots[i].target.x = goalie_line + ABS_ROBOT_SIZE/2;
					robots[i].vmax = 0.9;
				}
			}*/

			if (distance(robots[i].position, robots[i].target) < ABS_ROBOT_SIZE * 3 / 4) {
				// robots[i].cmdType = POSITION;
				// robots[i].target = robots[i].position;
				// robots[i].cmdType = ORIENTATION;
				// robots[i].targetOrientation = PI/2;
				// // if( tan(robots[i].orientation) > tan(robots[i].targetOrientation - 5*PI/180) && tan(robots[i].orientation) < tan(robots[i].targetOrientation + 5*PI/180) )
				// // {
				// // 	robots[i].cmdType = POSITION;
				robots[i].vmax = 0;
				// // }
			}

			if (abs(robots[i].target.x - robots[i].position.x) < fixed_pos_distance / 2 &&
				abs(robots[i].position.x - goalie_line) < fixed_pos_distance / 2) {
				go_to_x(i, robots[i].position.x);
			}

			break;
		default:
			break;
	}
}

void Strategy::opp_gk_routine(int i) {
	switch (robots[i].status) {
		case NORMAL_STATE:
			robots[i].fixedPos = true;
			robots[i].target.x = COORD_GOAL_ATK_FRONT_X - ABS_ROBOT_SIZE / 2;
			robots[i].target.y = Ball_Est.y;

			if (Ball_Est.y > COORD_GOAL_DWN_Y) robots[i].target.y = COORD_GOAL_DWN_Y;
			if (Ball_Est.y < COORD_GOAL_UP_Y) robots[i].target.y = COORD_GOAL_UP_Y;
			// if(danger_zone_2) robots[i].status = ADVANCING_STATE;
			break;

			// case ADVANCING_STATE:
			// robots[i].target = Ball;
			// if(robots[i].position.x > COORD_BOX_DEF_X || robots[i].position.y < COORD_BOX_UP_Y || robots[i].position.y > COORD_BOX_DWN_Y) {
			// 	full_transition = true;
			// }
			// break;
		default:
			break;
	}
}

//Estimativa da Bola
cv::Point Strategy::get_Ball_Est() {
	return Ball_Est;
}

void Strategy::set_Ball(cv::Point b) {
	Ball = b;
	LS_ball_x.addValue(Ball.x);
	LS_ball_y.addValue(Ball.y);

	Ball_Est.x = static_cast<int>(LS_ball_x.estimate(10));
	Ball_Est.y = static_cast<int>(LS_ball_y.estimate(10));
}

double Strategy::hyperbolic_field(int i, cv::Point P, bool clockwise){ //de = raio do espiral , p distancia da origem ate o ponteiro
	double hb_field;
	double p = distance_meters(robots[i].position, P);
	double theta = atan2((double)P.y,(double)P.x);
	if (p>de){
		if(clockwise)
		hb_field = theta + ((PI/2)*(2-(de+kr	)/(p+kr)));
		else
		hb_field = theta - ((PI/2)*(2-(de+kr	)/(p+kr)));
	}else{
		if(clockwise)
		hb_field = theta + ((PI/2)*sqrt(p/de));
		else
		hb_field = theta - ((PI/2)*sqrt(p/de));
	}
	return hb_field;
}

double Strategy::move_to_goal_field(int i, cv::Point P){
	double theta_pr_cw = hyperbolic_field(i, cv::Point(P.x, P.y + de), true);
	double theta_pl_ccw = hyperbolic_field(i, cv::Point(P.x, P.y - de), false);
	double theta_pr_ccw = hyperbolic_field(i, cv::Point(P.x, P.y + de), false);
	double theta_pl_cw = hyperbolic_field(i, cv::Point(P.x, P.y - de), true);

	cv::Point Nl = cv::Point(cos(theta_pl_ccw), sin(theta_pl_ccw));
	cv::Point Nr = cv::Point(cos(theta_pr_cw), sin(theta_pr_cw));

	double phi;

	if(-de<= P.y && P.y<de){
		cv::Point aux = (((P.y+de)*Nl)+((P.y-de)*Nr))/(2*de);
		phi = atan2(aux.x, aux.y);
	}else if(P.y < -de){
		phi = theta_pl_cw;
	}else if (P.y>= de) {
		phi = theta_pr_ccw;
	}
	return phi;
}

double Strategy::avoid_field(int i,cv::Point Obs,cv::Point Vrobo,cv::Point Vobs ){
	cv::Point s = kz*(Vobs - Vrobo);
	double d = distance_meters(robots[i].position, Obs);
	cv::Point Pelinha;
	if (d>= sqrt(pow(s.x,2)+pow(s.y,2))){
		 Pelinha = Obs+s;
	}else if (d< sqrt(pow(s.x,2)+pow(s.y,2))){
		Pelinha = Obs+((d/sqrt(pow(s.x,2)+pow(s.y,2)))*s);
	}

	double theta_avoid = angle_atan2(robots[i].position, Obs-Pelinha);
}

double Strategy::composed_field(int i,cv::Point P){
	for(int j = 0; j<3;j++){
		double theta;
		double R = distance_meters(adv[j],robots[i].position);
		if(R<=dmin){
			theta=avoid_field(i,adv[j],speed_vector(robots_ini[i].position, robots[i].position),speed_vector(adv_ini[i], adv[i]));
		}else if(R>dmin){
			double gaussiana= exp(-pow(R-dmin,2)/(2*delta));
			theta = (avoid_field(i,adv[j],speed_vector(robots_ini[i].position, robots[i].position),speed_vector(adv_ini[i], adv[i]))*gaussiana)+(move_to_goal_field(i,P)*(1-gaussiana));
		}
	}

}

cv::Point Strategy::speed_vector(cv::Point P_ini, cv::Point P_final){ //esta em pixels/segundo tem que ajeitar
	double precTick = ticks;
	ticks = (double) cv::getTickCount();
	double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
	cout<< "dt : "<< dT <<endl;
	cout<< "f : "<<  P_final.x <<endl;
	cout<< "i : "<<  P_ini.x <<endl;
	double dx = P_final.x - P_ini.x;
	double dy = P_final.y - P_ini.y;
	cout<< "dx : "<< dx <<endl;
	cv::Point vel;
	vel.x = dx/dT;
	vel.y = dy/dT;
	return vel;
}
