#ifndef STRATEGY_HPP_
#define STRATEGY_HPP_
#define PI 3.14159265453

#include "Fuzzy/FuzzyController.hpp"
#include "opencv2/opencv.hpp"
#include "LS.h"
#include "../pack-capture-gui/capture-gui/Robot.hpp"
#include <cmath>
#include "TestFrame.hpp"
#include "Constants.hpp"

#define PREDICAO_NUM_SAMPLES 15

//role
#define GOALKEEPER 0
#define DEFENDER 1
#define ATTACKER 2
#define OPPONENT 3

//cmdType
#define POSITION 0
#define SPEED 1
#define ORIENTATION 2
#define VECTOR 3
#define UVF 4

//state
#define NORMAL_STATE 0
#define CORNER_STATE 1
#define    STEP_BACK 2
#define    TRANSITION_STATE 4
#define    SIDEWAYS 5
#define DEF_CORNER_STATE 6


//potField
#define BALL_IS_OBS 0
#define BALL_ONLY_OBS 1
#define BALL_NOT_OBS 2
#define NO_OBS 3

#define THETA_TOLERATION 3

using namespace CONST;

class Strategy {
	public:
		TestFrame testFrame;

		cv::Point Ball;
		LS LS_ball_x;
		LS LS_ball_y;
		cv::Point Ball_Est;
		cv::Point target = cv::Point(-1, -1);

		double pot_thetaX = 0;
		double pot_thetaY = 0;
		double pot_theta = 0;
		double pot_goalTheta = 0;
		double pot_goalMag;
		double pot_magnitude[5];
		double pot_angle[5];
		double ball_angle = 0;
		double ball_mag = 0;

		std::vector<Robot> robots;
		cv::Point *adv;
		int collision_count[3];
		double past_transangle[3];
		cv::Point past_position[3];
		int atk, def, gk, opp;
		double lock_angle;

		FuzzyController controller;

		int frames_blocked;

		bool half_transition = false;
		bool full_transition = false;
		bool danger_zone_1 = false;
		bool danger_zone_2 = false;
		bool half_transition_enabled = false;
		bool full_transition_enabled = false;
		bool transition_mindcontrol_enabled = true;

		int t = 0;
		int timeout = 0;
		int transition_timeout = 0;
		int transition_overmind_timeout = 0;
		bool action1 = false;
		bool transition_mindcontrol = false;
		bool atk_mindcontrol = false;
		bool def_mindcontrol = false;

		//univector field
		double kr;
		double de;
		double ticks;

//		Ativa ou desativa transitions, controlada pela interface
		bool enable_transtions = true;

/*
Tipos de funções da estratégia:

- Math - Funções deste tipo realizam cálculos matemáticos comumente usados nos
diversos comportamentos e rotinas.

- IA - Funções não necessariamente matemáticas, mas que utilizam técnicas
complexas de planejamento de rotas, manipulaço de variáveis, entre outros. Estas
não controlam diretamente os jogadores, apenas entregam aos comportamentos os
valores necessários para tal, como angulação ou alvos.

- Caomportamentos - Estes são conjuntos de cálculos e condições que por fim
levam a uma ação, ou serquência breve de ações, a serem realizadas pelo robô.
Estas funções geralmente são responsáveis por alterar as variáveis dos agentes
como alvos(target), comandos(cmdType), orientação(targetOrientation) etc.

- Rotinas - Um conjunto de comportamentos, orquestrados a partir de condições
e estados referentes ao jogo são chamados de rotinas. Elas não devem alterar
variáveis dos jogadores diretamente, apenas chamando comportamentos.

- Auxiliares - Funções que não se encaixam nos outros padrões cujo propósito é
simplificar apenas uma outra função.

*/

		Strategy();

		/* set_constants
		Central --- Inicializa todas as variáveis de medida de campo, uma vez que
			dependem das dimensões da imagem.
		Input: 			Dimensões em pixeis(int)
		Output:			-----
		*/
		void set_constants(int w, int h);

		/* distance
		Math --- Calcula distancia entre dois pontos A e B
		Input: 			Dois pontos(cv::Point)
		Output:			Distância entre os pontos em pixeis
		*/
		double distance(cv::Point A, cv::Point B);

		/* distance_meters
		Math --- Calcula distancia em metros entre dois pontos A e B
		Input: 			Dois pontos(cv::Point)
		Output:			Distância entre os pontos em metros
		*/
		double distance_meters(cv::Point A, cv::Point B);

		/* angle_atan2
		Math --- Calcula o ângulo de A em relação a B
				Positivo em entido horáio e negativo anti-horário (igual a orientação)
		Input: 			Dois pontos(cv::Point)
		Output:			ângulo de A em relação a B em radianos
		*/
		double angle_atan2(cv::Point A, cv::Point B);

		/* get_variables
		Central --- Pega da interface as variáveis necessárias.
		Input: 			-----
		Output:			-----
		*/
		void get_variables();

		/* get_past
		Auxiliar --- Guarda posições passadas dos robôs para 'collision_check'
		Input: 			índice(int) do robô
		Output:			-----
		*/
		void get_past(int i);

		/* get_targets
		Central --- Função principal da estratégia. Ela chama todas as outras de
			maneira a determinar o fluxo de execução, além de fornecer os alvos aqui
			decididos para o resto do sistema.
		Input: 			-----
		Output:			-----
		*/
		void get_targets(std::vector<Robot> *pRobots, cv::Point *advRobots);

		/* overmind
		Central --- Sobrescreve rotinas baseando-se no escopo total do jogo. Esta
			função utiliza de todas as variáveis e informações disponíveis, como
			indices sobre todos os jogadores para realizar movintos mais complexos.
		Input: 			-----
		Output:			-----
		*/
		void overmind();

		/* Transitions
		Central ---  utiliza de todas as variáveis e informações disponíveis, como
		indices sobre todos os jogadores para chamar por transições de jogadores.
		Input: 			-----
		Output:			-----
		*/
		void Transitions();

		/* set_flags
		Central --- Aciona ou desliga flags de acordo com a situação atual do jogo
		Input: 			-----
		Output:			-----
		*/
		void set_flags();

		/* offensive_adv
		Central --- Identifica se ha algum adversário próximo a bola avançando ao gol
		Input: 			-----
		Output:			bool se há ou não adversário
		*/
		bool offensive_adv();

		/* fixed_position_check
		Central --- Caso o robo em questão esteja com a variável fixedPos == True
			entao as medidas são tomadas como, alteração de velocidade e alvos.
		Input: 			Indice(int) do robô
		Output:			-----
		*/
		void fixed_position_check(int i);

		/* collision_check
		Central --- Identifica se o robô em questão sofreu uma colisão e esta paralisado
		Input: 			Indice(int) do robô
		Output:			-----
		*/
		void collision_check(int i);

		/* player_blocked
		Central --- Identifica se ha robos bloqueados entre si
		Input: 			-----
		Output:			True(bool) caso haja pelo menos dois robôs bloqueados
		*/
		bool player_blocked();

		/* set_role
		Central --- muda o papel de um robô
		Input: 			indice do robo(int) e o papel desejado(int) dentre
			GOALKEEPER 0, DEFENDER 1, ATTACKER 2, OPPONENT 3
		Output:			-----
		*/
		void set_role(int i, int role);

		/* is_near
		Math --- Utiliza a distancia entre um robo e um ponto especifico para analisar
			se é menor que o limiar pre estabelecido(fixed_pos_distance).
		Input: 			indice do robo(int) e ponto(cv::Point).
		Output:			True ou False dependendo da distância.
		*/
		bool is_near(int i, cv::Point point);

		/* position_to_vector
		Comportamento --- transforma o alvo(target) em ângulo a ser seguido pelo
			robo(transAngle) para casos de utilização do VECTOR.
		Input: 			índice(int) do robô
		Output:			-----
		*/
		void position_to_vector(int i);

		/* go_to_x
		Comportamento --- Simplismente altera o alvo x do robô
		Input: 			índice(int) do robô e o valor do alvo(double)
		Output:			-----
		*/
		void go_to_x(int i, double x);

		/* go_to_y
		Comportamento --- Simplismente altera o alvo y do robô
		Input: 			índice(int) do robô e o valor do alvo(double)
		Output:			-----
		*/
		void go_to_y(int i, double y);

		/* go_to
		Comportamento --- Simplismente altera o alvo do robô
		Input: 			índice(int) do robô e o ponto alvo(cv::Point)
		Output:			-----
		*/
		void go_to(int i, cv::Point point);

		/* look_at_ball
		Comportamento --- Coloca robô em modo de ORIENTATION e aponta-o para a bola
		Input: 			índice(int) do robô
		Output:			-----
		*/
		double look_at_ball(int i);

		/* potField
		IA --- Utiliza Campos Potenciais para levar o robô a um objetivo
			desviando de obstáculos. O campo pode ter quatro comportamentos diferentes
			- BALL_IS_OBS -0- Bola considerada como obstáculo
			- BALL_ONLY_OBS -1- Bola considerada o único obstáculo
			- BALL_NOT_OBS -2- Bola não é considerada obstáculo (DEFAULT)
			- NO_OBS -3- Sem obstáculos
		Input: 			índice(int) do robô, ponto bjetivo(cv::Point) e modo (int)
		Output:			Angulo a ser seguido pelo robô no modo VECTOR
		*/
		double potField(int robot_index, cv::Point goal, int behavior);

		/* pot_rotation_decision
		Auxiliar --- Faz a decisão do sentido em que o robô deve girar para a função
			de Campos Potenciais(potField)
		Input: 			índice(int) do robô, ponto bjetivo(cv::Point) e obstaculo(cv::Point)
		Output:			direção(int) onde HORARIO = 1; ANTI_HORARIO = -1;
		*/
		int pot_rotation_decision(int robot_index, cv::Point goal, cv::Point obst);

		/* spin_anti_clockwise
		Comportamento --- Gira o robô no sentido anti-horário
		Input: 			índice(int) do robô e o valor da velocidade(double) de giro
		Output:			-----
		*/
		void spin_anti_clockwise(int i, double speed = 0.8);

		/* spin_clockwise
		Comportamento --- Gira o robô no sentido horário
		Input: 			índice(int) do robô e o valor da velocidade(double) de giro
		Output:			-----
		*/
		void spin_clockwise(int i, double speed = 0.8);

		/* def_wait
		Comportamento --- Espera o robô chegar ao alvo(não determinado na função)
			em seguida o faz olhar para a bola.
		Input: 			índice(int) do robô
		Output:			-----
		*/
		void def_wait(int i);

		/* fuzzy_init
		Auxiliar --- Inicializa as regras e memberships para serem utilizadas pelo Fuzzy
		Input: 			-----
		Output:			-----
		*/
		void fuzzy_init();

		/* Fuzzy_Troca
		IA --- Utiliza as regras e memberships inicializadas em 'fuzzy_init' para
			definir quando os jogadores devem trocar de papéis.
		Input: 			-----
		Output:			tipo de transição(int)
		 						0 = nenhum troca
		 						1 = troca todos
		 						2 = apenas ataque e defesa
		*/
		int Fuzzy_Troca();

		/* pot_field_around
		Comportamento --- Utiliza Campos Potenciais(potField) para fazer o robô dar
			a volta na bola e pegá-la por trás.
		Input: 			índice(int) do robô
		Output:			-----
		*/
		void pot_field_around(int i);

		/* crop_targets
		Comportamento --- Remove alvos fora do campo ou dentro da área aliada e os
			coloca em uma região 'aceitável'
		Input: 			índice(int) do robô
		Output:			-----
		*/
		void crop_targets(int i);

		/* smart_ball
		Comportamento --- Posiciona alvos entre a bola e sua estimativa de acordo
		com a relação da distancia entre o robo e a bola, e a distancia máxima a
		ser considerada.
		Input: 			índice(int) do robô e distancia maxima(int) a ser considerada
		Output:			-----
		*/
		void smart_ball(int i, int max_distance);

		/* atk_routine
		Rotina --- Aciona comportamentos do atacante dependendo das circunstâncias.
			Rotina utilizada na LARC 2017
		Input: 			índice(int) do robô que receberá tais comportamentos
		Output:			-----
		*/
		void atk_routine(int i);

		/* def_routine
		Routine --- Aciona comportamentos do defensor dependendo das circunstâncias.
			Rotina utilizada na LARC 2017
		Input: 			índice(int) do robô que receberá tais comportamentos
		Output:			-----
		*/
		void def_routine(int i);

		/* gk_routine
		Rotina --- Aciona comportamentos do goleiro dependendo das circunstâncias.
			Rotina utilizada na LARC 2017
		Input: 			índice(int) do robô que receberá tais comportamentos
		Output:			-----
		*/
		void gk_routine(int i);

		/* opp_gk_routine
		Rotina --- Faz com que o golerio se posicione no lado oposto do campo para
			treinamentos.
		Input: 			índice(int) do robô que receberá tais comportamentos
		Output:			-----
		*/
		void opp_gk_routine(int i);

		/* get_Ball_Est
		Central --- Apenas entrega a estimativa caso solicitada. Utilizada fora da estratégia.
		Input: 			-----
		Output:			Estimativa da bola(cv::Point)
		*/
		void uvf_routine(int i);


		cv::Point get_Ball_Est();

		/* set_Ball
		Central --- Inicializa e adquire a estimativa da bola.
		Input: 			Bola(cv::Point)
		Output:			-----
		*/
		void set_Ball(cv::Point b);
		/* speed vector (vetor_velocidade)
		Calcula a velocidade de um ponto
		Input: 		Ponto anterior e ponto atual
		Output: 	vetor velocidade (cv::Point)
		 * */


		/**	@brief Configura full_transition_enaled, permitindo que seja verdadeira
		 *	apenas se enable_transitions for verdadeiro (controlado pela interface)
		 *	@param enable Valor desejado de full_transition_enable */
		void set_full_transition_enabled(bool enable);

		/**	@brief Configura half_transition_enaled, permitindo que seja verdadeira
		 *	apenas se enable_transitions for verdadeiro (controlado pela interface)
		 *	@param enable Valor desejado de half_transition_enable */
		void set_half_transition_enabled(bool enable);

		/**	@brief Configura transition_mindcontrol_enaled, permitindo que seja verdadeira
		 *	apenas se enable_transitions for verdadeiro (controlado pela interface)
		 *	@param enable Valor desejado de transition_mindcontrol_enable */
		void set_transition_mindcontrol_enabled(bool enable);

		cv::Point speed_vector(cv::Point P_ini, cv::Point P_final);

		double hyperbolic_field(int i, cv::Point P, bool clockwise);
		double move_to_goal_field(int i, cv::Point P);
		double avoid_field(int i, cv::Point Obs, cv::Point Vrobo, cv::Point Vobs);
		double composed_field(int i, cv::Point P);
		void get_uvf_targets(std::vector<Robot> &pRobots);
		void atk_uvf_routine(int i);
		double wrap(double theta);
};

#endif /* STRATEGY_HPP_ */
