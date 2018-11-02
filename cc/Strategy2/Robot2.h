#ifndef VSSS_ROBOT_H
#define VSSS_ROBOT_H

#include "Geometry/Geometry.h"

class Robot2 {
	public:
		struct Pose {	// Define as variáveis de estado do robô.
			Geometry::Point position;
			double orientation;
			double velocity;
			double angular_velocity;
		};

		struct UVF_params {	// Parametros utilizados no UVF
			double n;
			double reference_distance;
		};

		enum class Command { // Comandos que o Messenger pode enviar para o robô
			Position, Vector, UVF, Orientation, Angular_Vel, None
		};

		enum class Role {
				Attacker = 0, Defender = 1, Goalkeeper = 2, None = 3
		};

  	private:
		Pose pose = { {0, 0}, 0, 0, 0.0 };	// Pose atual do robô
		Pose target = { {0, 0}, 0, 0, 0.0 }; //	Objetivo do robô. Uso de variáveis depende do comando utilizado
		Command command = Command::None; // Tipo de comando que será enviado pelo Messenger
		UVF_params uvf_data = { 1, 0.1 }; // Parâmetros utilizados no UVF

	public:
		Geometry::Point uvf_ref;

    	const double size = 0.08;
		char ID = 'A';
		unsigned int tag = 0;
		double default_target_velocity = 0.8; // Velocidade padrão do robô
		const double OFFSET = 0.03; // tolerância para saber se o robô chegou no ponto

		/**	Robô vai para um ponto e continua se movendo com mesma velocidade"
		 *	@param point Ponto em que o robô deve passar
		 *	@param velocity	Velocidade do movimento. Caso não seja definida,
		 *	robô se move com velocidade "default_target_velocity" */
		void go_to(Geometry::Point point, double velocity);
		/**	Robô vai para um ponto com a orientacao final desejada,
		 *	e continua se movendo com mesma velocidade"
		 *	@param point Ponto em que o robô deve passar
		 *	@param direction Vetor na direcao da orientacão desejada
		 *	@param velocity Velocidade do movimento. Caso não seja definida,
		 *	robô se move com velocidade "default_target_velocity" */
		void go_to_pose(Geometry::Point point,
						Geometry::Vector direction, double velocity);
		/**	Robô vai para um ponto e fica parado ao chegar
		 *	@param point Ponto em que o robô deve chegar e parar
		 *	@param velocity Velocidade do movimento. Caso não seja definida,
		 *	robô se move com velocidade "default_target_velocity" */
		void go_to_and_stop(Geometry::Point point, double velocity);
		void go_to_and_set_orientation(Geometry::Point point, double orientation, double velocity);
		/**	Robô se move na direcao do vetor
		 *	@param vector Vetor que define a orientacao
		 *	em que o robô deve se mover
		 *	@param velocity Velocidade do movimento. Caso não seja definida,
		 *	robô se move com velocidade "default_target_velocity" */
		void go_in_direction(Geometry::Vector vector, double velocity);
		/**	Robô continua parado no mesmo lugar, com orientacao igual
		 *	a do vetor definido
		 *	@param orientation Vetor que define a orentacao desejada do robô */
		void set_target_orientation(Geometry::Vector orientation);
		/**	Robô gira com a velocidade angular desejada
		 *	@param angular_velocity Velocidade angular do giro do robô
		 *	Valores positivos significam giro anti-horário,
		 *	valores negativos significam giro horário */
		void spin(double angular_velocity);

		/**	Fazem o mesmo que foi descrito nos métodos acima,
		 *	mas utilzam a velocidade padrão (definida na variável default_target_velocity) **/
		void go_to(Geometry::Point point);
		void go_to_pose(Geometry::Point point, Geometry::Vector direction);
		void go_to_and_stop(Geometry::Point point);
		void go_in_direction(Geometry::Vector vector);

		Pose get_pose() const { return  pose; };
		char get_ID() const { return ID; };
		void set_ID(char new_ID);
		Geometry::Point get_position() const { return pose.position; };
		Pose get_target() const { return target; }
		Command get_command() const { return command; }
		void set_pose(cv::Point position, double orientation);
		void set_pose(const Pose &new_pose);
		virtual Role get_role() { return Role::None; };
		virtual std::string get_role_name() { return "None"; };
};

#endif //VSSS_ROBOT_H
