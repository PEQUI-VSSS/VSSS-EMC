#include "Simulation.hpp"

Simulation::Simulation(std::array<Robot2 *, 3> &robots, Ball &ball,
		Strategy2 &strategy_ref, capture::V4LInterface& interface_ref):
		ros_robots({RosRobot{nh, "1"}, RosRobot{nh, "2"}, RosRobot{nh, "3"}}),
		ball_position_sub(nh, "ball/position", 1),
		sync(*ros_robots[0].pose_sub, *ros_robots[1].pose_sub,
			 *ros_robots[2].pose_sub, ball_position_sub, 10),
		robots(robots), ball(ball), strategy(strategy_ref),
		interface(interface_ref) {

	sync.registerCallback(&Simulation::ros_callback, this);
}

double quat_to_euler(double a, double b, double c, double d) {
	return std::atan2(2 * (a * d + b * c),
					  1 - 2 * (std::pow(c, 2) + std::pow(d, 2)));
}

void Simulation::ros_callback(const PoseStampedPtr &robot1_msg, const PoseStampedPtr &robot2_msg,
							  const PoseStampedPtr &robot3_msg, const PointStampedPtr &ball_msg) {

	const std::array<const PoseStampedPtr *, 3> msgs_ptr{&robot1_msg, &robot2_msg, &robot3_msg};

	for (auto& robot : robots) {
		const auto& msg = msgs_ptr[robot->tag]->get();
		const auto& position = msg->pose.position;
		const auto& quat = msg->pose.orientation;
		auto theta = quat_to_euler(quat.w, quat.x, quat.y, quat.z);
		robot->set_pose_simu({position.x, position.y}, theta);
	}

	auto ball_position = ball_msg.get()->point;
	ball.position = {ball_position.x, ball_position.y};

	if (interface.get_start_game_flag()) {
		ball.update_estimate();

		strategy.run();

		for (auto& robot : robots) {
			auto target = robot->get_target();

			vsss_msgs::Control control_msg;
			control_msg.command = (uint8_t) robot->get_command();
			control_msg.pose.x = target.position.x;
			control_msg.pose.y = target.position.y;
			control_msg.pose.theta = target.orientation;
			control_msg.velocity.linear.x = target.velocity;
			control_msg.velocity.angular.z = target.angular_velocity;

			ros_robots[robot->tag].control_pub.publish(control_msg);
		}

		interface.robotGUI.update_robot_functions();
		interface.robotGUI.update_speed_progressBars();
	}

	interface.updateRobotLabels();
	interface.update_ball_position(ball.position);
}

void Simulation::start_ros_thread() {
	assert(ros_thread == std::nullopt);
//	stop camera before run simulation
	if (0 != interface.bt_start.get_label().compare("start")) {
		interface.__event_bt_start_clicked();
	}
	run_ros_thread = true;
	ros_thread = std::optional<std::thread>{[&] {
		while (run_ros_thread) ros::spinOnce();
	}};
}

void Simulation::stop_ros_thread() {
	run_ros_thread = false;
	auto &thread = ros_thread.value();
	while (!thread.joinable());
	thread.join();
	ros_thread = std::nullopt;
}