#include <iostream>
#include "Xbee.h"

using std::string;
using std::pair;
using std::vector;

Xbee::Xbee(const string &port, int baud) : logs("logs_from_robots.txt"){
	xbee_err ret;

	if ((ret = xbee_setup(&xbee, "xbee1", port.c_str(), baud)) != XBEE_ENONE) {
		printf("Xbee setup error: %d (%s)\n", ret, xbee_errorToStr(ret));
	} else {
		std::cout << "Xbee connected on " << port << std::endl;
	}
}

Xbee::~Xbee() {
	for (pair<const char, robot_xbee> &r : robots) {
		xbee_conEnd(r.second.con);
	}
	robots.clear();
	xbee_shutdown(xbee);
}

void Xbee::add_robot(char id, uint16_t addr) {
	struct xbee_con *con;
	struct xbee_conAddress address{};

	memset(&address, 0, sizeof(address));
	address.addr16_enabled = 1;
	address.addr16[0] = (uint8_t) (addr >> 8);
	address.addr16[1] = (uint8_t) addr;

	xbee_err ret;
	if ((ret = xbee_conNew(xbee, &con, "16-bit Data", &address)) != XBEE_ENONE) {
		printf("Xbee connection error: %d (%s)\n", ret, xbee_errorToStr(ret));
		return;
	}

	robots[id] = {id, addr, con, {0, 0, 0}};
	set_ack_enabled(id, false);
}

int Xbee::send(char id, const string &message) {
	if (robots.count(id) == 0) return -1;

	uint8_t ack;
	xbee_conTx(robots[id].con, &ack, message.c_str());
	update_ack(id, ack);
	return ack;
}

vector<message> Xbee::get_messages() {
	struct xbee_pkt *pkt;
	vector<message> msgs;
	for (auto &r : robots) {
		if (xbee_conRx(r.second.con, &pkt, nullptr) == XBEE_ENONE && pkt->dataLen > 0) {
			msgs.push_back({r.second.id, get_string(pkt)});
		}
	}
	return msgs;
}

string Xbee::send_get_answer(char id, const string &message) {
	if (robots.count(id) == 0) return "erro";
	struct xbee_pkt *pkt;
	uint8_t ack;
	bool ack_enabled = is_ack_enabled(id);

	set_ack_enabled(id, true);
	xbee_conTx(robots[id].con, &ack, message.c_str());
	set_ack_enabled(id, ack_enabled);

	update_ack(id, ack);
	if (ack == 0 && xbee_conRxWait(robots[id].con, &pkt, nullptr) == XBEE_ENONE && pkt->dataLen > 0)
		return get_string(pkt);
	else
		return "erro";
}

vector<message> Xbee::send_get_answer(const std::string &message) {
	vector<struct message> msgs;
	for (auto &r : robots) {
		string ret = send_get_answer(r.second.id, message);
		if (!ret.empty()) msgs.push_back({r.second.id, ret});
	}
	return msgs;
}

void Xbee::set_ack_enabled(char id, bool enable) {
	if (robots.count(id) == 0) return;
	struct xbee_con *con = robots[id].con;
	struct xbee_conSettings settings{};

	xbee_conSettings(con, nullptr, &settings);
	settings.disableAck = (uint8_t) !enable;
	xbee_conSettings(con, &settings, nullptr);
	if (!enable) robots[id].acks = {0, 0, 0};
}

void Xbee::set_ack_enabled(bool enable) {
	for (auto &robot : robots) {
		set_ack_enabled(robot.second.id, enable);
	}
}

bool Xbee::is_ack_enabled(char id) {
	if (robots.count(id) == 0) return false;
	struct xbee_conSettings settings{};

	xbee_conSettings(robots[id].con, nullptr, &settings);
	return !settings.disableAck;
}

void Xbee::update_ack(char id, int ack) {
	if (!is_ack_enabled(id)) return;
	ack_count &acks = robots[id].acks;
	if (ack != 0) acks.lost++;
	acks.total++;
	acks.lost_rate = double(acks.lost) / double(acks.total) * 100;
}

string Xbee::get_string(struct xbee_pkt *pkt) {
	string str(pkt->data, pkt->data + pkt->dataLen);
	return str;
}

ack_count Xbee::get_ack_count(char id) {
	return robots[id].acks;
}

void Xbee::reset_lost_acks() {
	for (auto &r : robots) {
		r.second.acks = {0, 0, 0};
	}
}

void Xbee::save_logs() {
	struct xbee_pkt *pkt;
	for (auto &r : robots) {
		if (xbee_conRx(r.second.con, &pkt, nullptr) == XBEE_ENONE && pkt->dataLen > 0) {
			logs << get_string(pkt) << std::endl;
		}
	}
}
