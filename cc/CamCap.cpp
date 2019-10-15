#include <Strategy2/Robot2.h>
#include "CamCap.hpp"
#include "ImageArt.hpp"

using namespace vision;
using namespace art;

using std::cout;
using std::endl;
using std::vector;

void CamCap::update_positions(const std::map<unsigned int, Vision::RecognizedTag> &tags) {
	for (const auto &robot : robots) {
		auto search = tags.find(robot->tag);
		if (search != tags.end())
			robot->set_pose(search->second.position, search->second.orientation);
	}

	interface.updateRobotLabels();

	const Vision::Ball cv_ball = interface.visionGUI.vision->getBall();
	ball = Geometry::from_cv_point(cv_ball.position);
	interface.update_ball_position(ball);
}

bool CamCap::start_signal(bool b) {
	if (b) {
		cout << "Start Clicked!" << endl;

		if (data) {
			interface.imageView.disable_image_show();
			free(data);
			data = nullptr;
		}

		width = interface.vcap.format_dest.fmt.pix.width;
		height = interface.vcap.format_dest.fmt.pix.height;

		interface.visionGUI.setFrameSize(width, height);

		// Liberar os botões de edit
		robotGUI.enable_main_buttons();

		data = (unsigned char *) calloc(interface.vcap.format_dest.fmt.pix.sizeimage, sizeof(unsigned char));

		interface.imageView.set_frame_size(width, height);
		con = Glib::signal_idle().connect(sigc::mem_fun(*this, &CamCap::capture_and_show));

		interface.__event_bt_quick_load_clicked();
	} else {
		cout << "Stop Button Clicked!" << endl;
		con.disconnect();

		// Travar os botões de edit
		robotGUI.enable_main_buttons(false);
	}

	return true;
} // start_signal

bool CamCap::capture_and_show() {

	bool chessBoardFound = false;
	std::vector<cv::Vec2f> foundPoints;

	if (!data) return false;

	if (frameCounter == 0) {
		timer_start = std::chrono::high_resolution_clock::now();
	}
	frameCounter++;

	interface.vcap.grab_rgb(data);
	interface.imageView.set_data(data);
	interface.imageView.refresh();

	cv::Mat imageView(height, width, CV_8UC3, data);

	interface.imageView.split_flag = interface.visionGUI.getIsSplitView();

	if (interface.imageView.sector != -1) {
		interface.visionGUI.selectFrame(interface.imageView.sector);
		interface.imageView.sector = -1;
	}

	if (interface.visionGUI.vision->flag_cam_calibrated) {
		cv::Mat temp;
		imageView.copyTo(temp);
		cv::undistort(temp, imageView, interface.visionGUI.vision->getcameraMatrix(),
					  interface.visionGUI.vision->getdistanceCoeficents());
	}

	interface.imageView.imageWarp.run(imageView);

	bool is_game_on = interface.get_start_game_flag();
	bool is_test_on_click_on = interface.controlGUI.test_controller.is_active();

	if (interface.CamCalib_flag_event && !is_game_on && !is_test_on_click_on &&
		!interface.visionGUI.vision->flag_cam_calibrated) {

		chessBoardFound = cv::findChessboardCorners(imageView, CHESSBOARD_DIMENSION, foundPoints,
													cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
	}

	std::map<unsigned int, Vision::RecognizedTag> tags = interface.visionGUI.vision->run(imageView);
	auto adv = interface.visionGUI.vision->advRobots;

	interface.visionGUI.recorder.run(imageView);

	calculate_ball_est();
	update_positions(tags);

	interface.updateFPS(fps_average);

	if (interface.controlGUI.ekf_always_send || is_game_on || is_test_on_click_on) {
		notify_data_ready(true);
	}

	if (interface.visionGUI.getIsSplitView()) {
		interface.imageView.set_data(interface.visionGUI.vision->getSplitFrame().clone().data);
		interface.imageView.refresh();
	} else if (interface.visionGUI.CIELAB_calib_event_flag) {
		interface.imageView.set_data(interface.visionGUI.vision->getThreshold(interface.visionGUI.Img_id).data);
		interface.imageView.refresh();
	}

	interface.controlGUI.update_dropped_frames();

	// ----------- ESTRATEGIA -----------------//
	if (is_game_on) {
		circle(imageView, Ball_Est, 7, cv::Scalar(255, 140, 0), 2);
//		strategyGUI.strategy.get_uvf_targets( interface.robot_list );

		strategy.run();

		interface.controlGUI.update_msg_time();
		notify_data_ready(false);

		robotGUI.update_speed_progressBars();
		robotGUI.update_robot_functions();
	} // ---------- TEST ON CLICK --------------//
	else if (is_test_on_click_on) {
		interface.controlGUI.test_controller.run();
		interface.controlGUI.update_msg_time();
		notify_data_ready(false);
	}
	// -----------------------------------------//

	if (!interface.visionGUI.CIELAB_calib_event_flag && !interface.visionGUI.getIsSplitView()) {
		if (chessBoardFound) {
			cv::TermCriteria termCriteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
			cv::Mat grayFrame;
			cv::cvtColor(imageView, grayFrame, cv::COLOR_RGB2GRAY);
			cv::cornerSubPix(grayFrame, foundPoints, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
		}

		if (interface.visionGUI.getIsDrawing()) {
			cv::drawChessboardCorners(imageView, CHESSBOARD_DIMENSION, foundPoints, chessBoardFound);

			interface.imageView.imageArt.draw(imageView,
											  interface.visionGUI.vision->getBall(),
											  tags,
											  interface.visionGUI.vision->get_adv_robots(),
											  robots,
											  is_game_on);
		} // if !interface.draw_info_flag
	} // if !draw_info_flag


	if (frameCounter == 10) {
		auto timer_finish = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = timer_finish - timer_start;
		fps_average = static_cast<int>(10 / elapsed.count());
		frameCounter = 0;
	}

	return true;
} // capture_and_show

void CamCap::send_cmd_thread() {
	boost::unique_lock<boost::mutex> lock(data_ready_mutex);
	while (true) {
		try {
			data_ready_cond.wait(lock, [this]() { return data_ready_flag; });
		} catch (...) {
			lock.unlock();
			return;
		}
		data_ready_flag = false;
		if (ekf_data_ready) {
			for (auto &robot : this->robots)
				interface.controlGUI.messenger.send_ekf_data(*robot);
			ekf_data_ready = false;
		} else {
			interface.controlGUI.messenger.send_commands(this->robots);
		}
	}
}

void CamCap::notify_data_ready(bool send_ekf_data) {
	data_ready_flag = true;
	ekf_data_ready = send_ekf_data;
	data_ready_cond.notify_all();
}

double CamCap::distance(cv::Point a, cv::Point b) {
	return sqrt(pow(double(b.x - a.x), 2) + pow(double(b.y - a.y), 2));
}


void CamCap::calculate_ball_est() {
	ls_x.addValue(ball.x);
	ls_y.addValue(ball.y);

	ball_est.x = ls_x.estimate(10);
	ball_est.y = ls_y.estimate(10);
}

CamCap::CamCap(bool isLowRes) : robots{&attacker, &defender, &goalkeeper}, data(nullptr), width(0), height(0), frameCounter(0),
														  msg_thread(&CamCap::send_cmd_thread, this),
														  strategy(attacker, defender, goalkeeper, ball, ball_est),
														  robotGUI(robots, isLowRes),
								interface(robots,
										  ball, robotGUI, isLowRes)
														   {

	ls_x.init(15, 1);
	ls_y.init(15, 1);

	fm.set_label("imageView");
	fm.add(interface.imageView);

	notebook.append_page(interface, "Capture");
	notebook.append_page(interface.visionGUI, "Vision");
	notebook.append_page(interface.controlGUI, "Control");
	if (isLowRes) {
		// Caso esteja em baixa resolução, será criado uma aba Robot.
		// Caso contrário, robotGUI será colocado na info_box da interface.
		notebook.append_page(robotGUI, "Robot");
	}
	notebook.append_page(strategyGUI, "Strategy");

	camera_vbox.pack_start(fm, false, true, 5);
	camera_vbox.pack_start(info_fm, false, true, 5);
	info_fm.add(interface.info_hbox);

	pack_start(camera_vbox, true, true, 10);
	pack_start(notebook, false, false, 10);

	interface.signal_start().connect(sigc::mem_fun(*this, &CamCap::start_signal));
}

CamCap::~CamCap() {
	con.disconnect();
	interface.imageView.disable_image_show();
	free(data);

	data = nullptr;

	msg_thread.interrupt();
	if (msg_thread.joinable()) msg_thread.join();
}