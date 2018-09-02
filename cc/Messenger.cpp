#include "Messenger.h"

using std::string;
using std::vector;
using Pose = Robot2::Pose;
using Command = Robot2::Command;

void Messenger::start_xbee(const string &port, int baud) {
	xbee = new Xbee(port, baud);
	add_robots();
}

void Messenger::stop_xbee() {
	if (!xbee) return;
	delete xbee;
	xbee = nullptr;
}

void Messenger::add_robots() {
	if (!xbee) return;
	xbee->add_robot('A', 0x88a0);
	xbee->add_robot('B', 0xb24a);
	xbee->add_robot('C', 0x215c);
	xbee->add_robot('D', 0x35f6);
	xbee->add_robot('E', 0x97e7);
	xbee->add_robot('F', 0x6b0d);
}

void Messenger::send_msg(char id, string msg) {
	if (!xbee) return;
	xbee->send(id, msg);
}

void Messenger::send_old_format(string cmd) {
	if (!xbee) return;
	char id = cmd[0];
	string msg = cmd.substr(2, cmd.find('#') - 2);
	xbee->send(id, msg);
}

void Messenger::send_commands(const std::array<Robot2*, 3> &robots) {
	if (!xbee || ++send_cmd_count <= frameskip) return;
	for (Robot2* robot : robots) {
		send_command(robot->ID, robot->get_target(), robot->get_command(), robot->uvf_ref);
	}
	update_msg_time();
	send_cmd_count = 0;
}

constexpr float robot_size = 0.0675f;
void Messenger::send_command(char id, Pose target, Command command, Geometry::Point uvf_ref) {
	if(!xbee) return;
	const string msg = [&] {
		switch (command) {
			case Command::Position:
				return "P" + rounded_str(target.position.x * 100)
							 + ";" + rounded_str(target.position.y * 100)
									 + ";" + rounded_str(target.velocity);
			case Command::Angular_Vel:
				return rounded_str(target.angular_velocity*robot_size/2)
					   + ";" + rounded_str(-target.angular_velocity*robot_size/2);
			case Command::Vector:
				return ("V" + rounded_str(target.orientation * 180.0f/M_PI)
							  + ";" + rounded_str(target.velocity));
			case Command::UVF:
				return "U" + rounded_str(target.position.x * 100) + ";" + rounded_str(target.position.y * 100)
					   + ";" + rounded_str(uvf_ref.x * 100) + ";" + rounded_str(uvf_ref.y * 100)
					   + ";" + rounded_str(1.8) + ";" + rounded_str(target.velocity);
			case Command::Orientation:
				return "O" + rounded_str(target.orientation * 180/M_PI)
							 + ";" + rounded_str(target.velocity);
			default:
				return string();
		}
	}();
	if (!msg.empty()) {
		xbee->send(id, msg);
//		if(id == 'A') std::cout << msg << std::endl;
	}
}

void Messenger::send_ekf_data(const Robot2 &robot) {
	if(!xbee) return;
	auto robot_pose = robot.get_pose();
	string msg = "E" + rounded_str(robot_pose.position.x * 100) + ";"
				 + rounded_str(robot_pose.position.y * 100)
				 + ";" + rounded_str(robot_pose.orientation * 180/M_PI);
	xbee->send(robot.get_ID(), msg);
//	if(robot.get_ID() == 'A') std::cout << msg << std::endl;
}

double Messenger::get_battery(char id) {
	if (!xbee) return -1;
	string msg = xbee->send_get_answer(id, "B");
	if (msg.empty() || msg[0] != 'B') return -1;
	return ((stod(msg.substr(1)) - 6.4) / 2.0) * 100;
}

string Messenger::rounded_str(double num) {
	double rounded_num = round(num * 100) / 100;
	std::ostringstream ss;
	ss << rounded_num;
	return ss.str();
}

void Messenger::set_ack_enabled(bool enable) {
	if (!xbee) return;
	xbee->set_ack_enabled(enable);
}

ack_count Messenger::get_ack_count(char id) {
	if (!xbee) return {-1, -1, -1};
	else return xbee->get_ack_count(id);
}

void Messenger::reset_lost_acks() {
	if (!xbee) return;
	xbee->reset_lost_acks();
}

void Messenger::update_msg_time() {
	auto now = std::chrono::system_clock::now();
	std::chrono::duration<double, std::milli> time_diff = now - previous_msg_time;
	time_between_msgs = time_diff.count();
	previous_msg_time = now;
}

Messenger::Messenger()
//		: ekf_data_file("ekf_data.csv")
{
	setlocale(LC_ALL, "C");
	send_cmd_count = 0;
	frameskip = DEFAULT_FRAMESKIP;
	previous_msg_time = std::chrono::system_clock::now();
}
