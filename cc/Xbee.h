#ifndef VSSS_XBEE_H
#define VSSS_XBEE_H

#include <xbee.h>
#include <vector>
#include <string>
#include <stack>
#include <cstring>
#include <unordered_map>
#include <iostream>
#include <fstream>

struct message {
	char id;
	std::string data;
};

struct ack_count {
	int lost;
	int total;
	double lost_rate;
};

struct robot_xbee {
	char id;
	uint16_t addr;
	xbee_con *con;
	ack_count acks;
};

class Xbee {
	private:
		struct xbee *xbee;
		std::unordered_map<char, robot_xbee> robots;
		std::string get_string(xbee_pkt *pkt);
		void update_ack(char id, int ack);

		std::ofstream logs;
//		Código de serializacão de variáveis

//		Caso base da recursão
		static constexpr unsigned byte_size() {return 0;}
//		Calcula tamanho do buffer necessário para enviar todos os valores
		template <typename First, typename... Others>
		static constexpr unsigned byte_size(const First & first_value,
									 const Others &... other_values) {
			return sizeof first_value + byte_size(other_values...);
		}

//		Caso base da recursão
//		static void set_buffer(uint8_t* buffer) {}
//		Preenche o buffer com os valores a serem enviados, de forma sequencial
//		template <typename First, typename... Others>
//		static void set_buffer(uint8_t* buffer, const First & first_value,
//						const Others &... other_values) {
//			std::memcpy(buffer, &first_value, sizeof(first_value));
//			set_buffer(buffer + sizeof(first_value), other_values...);
//		}

	public:
		Xbee(const std::string &port, int baud);
		~Xbee();
		void add_robot(char id, uint16_t addr);
		int send(char id, const std::string &message);
		std::string send_get_answer(char id, const std::string &message);
		std::vector<message> send_get_answer(const std::string &message);
		std::vector<message> get_messages();
		void save_logs();
		ack_count get_ack_count(char id);
		void reset_lost_acks();
		void set_ack_enabled(char id, bool enable);
		void set_ack_enabled(bool enable);
		bool is_ack_enabled(char id);

//		Envia os valores de forma sequencial: Um byte indicando tipo de mensagem
//		e bytes seguintes são o conteudo da mensagem
//		template <typename... Data>
//		void send(char ID, uint8_t type, const Data &... data) {
//			uint8_t buffer[byte_size(type, data...)];
//			set_buffer(buffer, type, data...);
//			uint8_t ack;
//			xbee_connTx(robots[ID].con, &ack, buffer, sizeof buffer);
//
//			Le resposta para debugar
//			if (ID == 'B') {
//				auto msgs = get_messages();
//				for (auto& m : msgs) {
//					std::cout << m.data << std::endl;
//				}
//			}
//		}
};

#endif //VSSS_XBEE_H
