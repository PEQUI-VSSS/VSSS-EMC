#include "CamCap.hpp"

using std::cout;
using std::endl;
using std::vector;

bool CamCap::checkForLowRes() {
	return screenWidth < 1600;
}

void CamCap::updateAllPositions() {
	Robot robot;
	cv::Point ballPosition;

	for (int i = 0; i < interface.visionGUI.vision->getRobotListSize(); i++) {
		robot = interface.visionGUI.vision->getRobot(i);
		interface.robot_list[i].position = robot.position;
		interface.robot_list[i].orientation = robot.orientation;
		interface.robot_list[i].secundary = robot.secundary;
	}

	ballPosition = interface.visionGUI.vision->getBall();
	interface.ballX = ballPosition.x;
	interface.ballY = ballPosition.y;

	interface.updateRobotLabels();
	interface.updateFPS(fps_average);
} // updateAllPositions

void CamCap::updateKalmanFilter() {
	// KALMAN FILTER
	if (KF_FIRST) {
		//KALMAN FILTER INIT
		for (int i = 0; i < 3; i++) {
			KF_Robot[i].KF_init(interface.visionGUI.vision->getRobotPos(i));
		}
		KF_Robot[3].KF_init(interface.visionGUI.vision->getBall());
		for (int i = 0; i < interface.visionGUI.vision->getAdvListSize(); i++) {
			KF_Robot[i + 4].KF_init(interface.visionGUI.vision->getBall());
		}
		KF_FIRST = false;
	}
	//robot_kf_est_ini = robot_kf_est;
	//Ball_kf_est_ini = Ball_kf_est;
	robot_kf_est[0] = KF_Robot[0].KF_Prediction(interface.visionGUI.vision->getRobotPos(0));
	robot_kf_est[1] = KF_Robot[1].KF_Prediction(interface.visionGUI.vision->getRobotPos(1));
	robot_kf_est[2] = KF_Robot[2].KF_Prediction(interface.visionGUI.vision->getRobotPos(2));
	Ball_kf_est = KF_Robot[3].KF_Prediction(interface.visionGUI.vision->getBall());

	robot_kf_est[0] = KF_Robot[0].KF_Prediction(interface.visionGUI.vision->getRobotPos(0));
	robot_kf_est[1] = KF_Robot[1].KF_Prediction(interface.visionGUI.vision->getRobotPos(1));
	robot_kf_est[2] = KF_Robot[2].KF_Prediction(interface.visionGUI.vision->getRobotPos(2));
	for (int i = 3; i < 3 + interface.visionGUI.vision->getAdvListSize(); i++) {
		robot_kf_est[i - 1] = KF_Robot[i].KF_Prediction(interface.visionGUI.vision->getRobotPos(2));
	}
} // updateKalmanFilter

bool CamCap::start_signal(bool b) {
	if (b) {
		cout << "Start Clicked!" << endl;

		if (data) {
			interface.imageView.disable_image_show();
			free(data);
			data = 0;
		}
		/*GdkScreen* screen = gdk_screen_get_default();
		if (interface.vcap.format_dest.fmt.pix.width > gdk_screen_get_width(screen)/2 || interface.vcap.format_dest.fmt.pix.height > gdk_screen_get_height(screen)/2)
		{
		width = gdk_screen_get_width(screen)/2;
		height = gdk_screen_get_height(screen)/2;
		strategyGUI.strategy.set_constants(width,height);
		}
		else
		{*/
		width = interface.vcap.format_dest.fmt.pix.width;
		height = interface.vcap.format_dest.fmt.pix.height;
		strategyGUI.strategy.set_constants(width, height);
		//}

		interface.visionGUI.setFrameSize(width, height);

		// Liberar os botões de edit
		interface.robots_id_edit_bt.set_state(Gtk::STATE_NORMAL);
		interface.robots_speed_edit_bt.set_state(Gtk::STATE_NORMAL);
		interface.robots_function_edit_bt.set_state(Gtk::STATE_NORMAL);

		data = (unsigned char *) calloc(interface.vcap.format_dest.fmt.pix.sizeimage, sizeof(unsigned char));

		interface.imageView.set_size_request(width, height);
		con = Glib::signal_idle().connect(sigc::mem_fun(*this, &CamCap::capture_and_show));

		cout << "Start Clicked! 1" << endl;
	} else {
		cout << "Stop Button Clicked!" << endl;
		con.disconnect();

		// Travar os botões de edit
		interface.robots_id_edit_bt.set_state(Gtk::STATE_INSENSITIVE);
		interface.robots_speed_edit_bt.set_state(Gtk::STATE_INSENSITIVE);
		interface.robots_function_edit_bt.set_state(Gtk::STATE_INSENSITIVE);
	}

	interface.__event_bt_quick_load_clicked();
	interface.visionGUI.quickLoadGMM();

	return true;
} // start_signal

bool CamCap::capture_and_show() {

	bool chessBoardFound = false;
	std::vector<cv::Vec2f> foundPoints;

	if (!data) return false;

	if (frameCounter == 0) {
		timer.start();
	}
	frameCounter++;

	interface.vcap.grab_rgb(data);
	interface.imageView.set_data(data, width, height);
	interface.imageView.refresh();

	cv::Mat imageView(height, width, CV_8UC3, data);

	if (interface.imageView.hold_warp) {
		interface.warped = true;
		interface.bt_adjust.set_state(Gtk::STATE_NORMAL);
		interface.imageView.warp_event_flag = false;
		interface.imageView.warp_event_flag = false;
		interface.imageView.hold_warp = false;
	}

	if (interface.reset_warp_flag) {
		interface.imageView.warp_counter = 0;
		interface.reset_warp_flag = false;
	}

	interface.imageView.split_flag = interface.visionGUI.getIsSplitView();
	interface.imageView.PID_test_flag = control.PID_test_flag;
	interface.imageView.adjust_event_flag = interface.adjust_event_flag;
	interface.imageView.gmm_sample_flag = interface.visionGUI.getSamplesEventFlag();

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

	if (interface.warped) {
		interface.bt_warp.set_active(false);
		interface.bt_warp.set_state(Gtk::STATE_INSENSITIVE);
		warp_transform(imageView);
		interface.imageView.warp_event_flag = false;

		if (interface.invert_image_flag) {
			cv::flip(imageView, imageView, -1);
		}
	}

	if (interface.CamCalib_flag_event && !interface.get_start_game_flag() &&
		!interface.visionGUI.vision->flag_cam_calibrated) {

		chessBoardFound = cv::findChessboardCorners(imageView, CHESSBOARD_DIMENSION, foundPoints,
													CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
	}

	if (interface.imageView.gmm_ready_flag) {
		interface.visionGUI.gmm->setFrame(imageView);
		interface.visionGUI.gmm->pushSample(interface.imageView.gmm_clicks);
		interface.visionGUI.incrementSamples();
		interface.imageView.gmm_ready_flag = false;
	}

	updateKalmanFilter();

	if (!interface.visionGUI.getIsHSV()) { // GMM
		if (interface.visionGUI.gmm->getIsTrained()) {
			interface.visionGUI.gmm->run(imageView);
			interface.visionGUI.vision->recordVideo(imageView);
			if (interface.visionGUI.gmm->getDoneFlag()) {
				for (auto &window : interface.visionGUI.gmm->windowsList) {
					rectangle(imageView, window.getPosition(),
							  window.getEnd(), cv::Scalar(250, 155, 0));
				}
			}

			interface.visionGUI.vision->runGMM(interface.visionGUI.gmm->getAllThresholds(),
											   interface.visionGUI.gmm->getWindowsList());

			if (interface.visionGUI.getGaussiansFrameFlag()) {
				interface.imageView.set_data(interface.visionGUI.gmm->getGaussiansFrame().data, width, height);
				interface.imageView.refresh();
			} else if (interface.visionGUI.getFinalFrameFlag()) {
				interface.imageView.set_data(interface.visionGUI.gmm->getFinalFrame().data, width, height);
				interface.imageView.refresh();
			} else if (interface.visionGUI.getThresholdFrameFlag()) {
				interface.imageView.set_data(
						interface.visionGUI.gmm->getThresholdFrame(interface.visionGUI.getGMMColorIndex()).data,
						width, height);
				interface.imageView.refresh();
			}
		}
	} else { // HSV Simples
		interface.visionGUI.vision->run(imageView);
		if (interface.visionGUI.getIsSplitView()) {
			interface.imageView.set_data(interface.visionGUI.vision->getSplitFrame().clone().data, width, height);
			interface.imageView.refresh();
		} else if (interface.visionGUI.HSV_calib_event_flag) {
			interface.imageView.set_data(interface.visionGUI.vision->getThreshold(interface.visionGUI.Img_id).data,
										 width, height);
			interface.imageView.refresh();
		}
	}

	if (!interface.visionGUI.HSV_calib_event_flag) {
		if (chessBoardFound) {

			cv::TermCriteria termCriteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
			cv::Mat grayFrame;
			cv::cvtColor(imageView, grayFrame, cv::COLOR_RGB2GRAY);
			cv::cornerSubPix(grayFrame, foundPoints, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
			cv::drawChessboardCorners(imageView, CHESSBOARD_DIMENSION, foundPoints, chessBoardFound);
		}

		if (interface.visionGUI.getIsDrawing() && !interface.visionGUI.getIsSplitView()) {
			cv::Point aux_point;

			if (interface.imageView.PID_test_flag) {
				for (auto &robot : interface.robot_list) {
					if (robot.target.x != -1 && robot.target.y != -1) {
						// linha branca no alvo sendo executado
						line(imageView, robot.position, robot.target,
							 cv::Scalar(255, 255, 255), 2);
						// linha roxa no alvo final
						line(imageView, robot.position,
							 cv::Point(static_cast<int>(interface.imageView.tar_pos[0]),
									   static_cast<int>(interface.imageView.tar_pos[1])),
							 cv::Scalar(255, 0, 255), 2);
					}
					// círculo branco no alvo sendo executado
					circle(imageView, robot.target, 9, cv::Scalar(255, 255, 255), 2);
					// círculo roxo no alvo final
					circle(imageView, cv::Point(static_cast<int>(interface.imageView.tar_pos[0]),
												static_cast<int>(interface.imageView.tar_pos[1])), 7,
						   cv::Scalar(255, 0, 255), 2);
					// círculo vermelho no obstáculo
					circle(imageView, obstacle, 17, cv::Scalar(255, 0, 0), 2);
					// círculo verde nos desvios
					circle(imageView, deviation1, 7, cv::Scalar(0, 255, 0), 2);
					circle(imageView, deviation2, 7, cv::Scalar(0, 255, 0), 2);
				}
				if (Selec_index != -1) {
					circle(imageView, interface.robot_list[Selec_index].position, 17, cv::Scalar(255, 255, 255), 2);
				}
			}

			if (interface.visionGUI.getDrawSamples()) {
				std::vector<cv::Point> points = interface.visionGUI.gmm->getSamplePoints();
				for (unsigned long i = 0; i < points.size(); i = i + 2) {
					rectangle(imageView, points.at(i), points.at(i + 1), cv::Scalar(0, 255, 255));
				}
			}

			circle(imageView, interface.visionGUI.vision->getBall(), 7, cv::Scalar(255, 255, 255), 2);

			for (int i = 0; i < interface.visionGUI.vision->getRobotListSize(); i++) {
				// robo 1
				line(imageView, interface.visionGUI.vision->getRobot(i).position,
					 interface.visionGUI.vision->getRobot(i).secundary, cv::Scalar(255, 255, 0), 2);
				putText(imageView, std::to_string(i + 1),
						cv::Point(interface.visionGUI.vision->getRobot(i).position.x - 5,
								  interface.visionGUI.vision->getRobot(i).position.y - 17), cv::FONT_HERSHEY_PLAIN,
						1, cv::Scalar(255, 255, 0), 2);
				circle(imageView, interface.visionGUI.vision->getRobot(i).position, 15, cv::Scalar(255, 255, 0), 2);
				// linha da pick-a
				if (interface.visionGUI.vision->getRobot(i).rearPoint != cv::Point(-1, -1))
					line(imageView, interface.visionGUI.vision->getRobot(i).secundary,
						 interface.visionGUI.vision->getRobot(i).rearPoint, cv::Scalar(255, 0, 0), 2);


				// vetor que todos os robos estão executando
				aux_point.x = static_cast<int>(round(100 * cos(interface.robot_list[i].transAngle)));
				aux_point.y = static_cast<int>(-round(100 * sin(interface.robot_list[i].transAngle)));
				aux_point += interface.robot_list[i].position;
			}

			for (int i = 0; i < 5; i++) {
				aux_point.x = static_cast<int>(round(100 * cos(strategyGUI.strategy.pot_angle[i])));
				aux_point.y = static_cast<int>(-round(100 * sin(strategyGUI.strategy.pot_angle[i])));
				aux_point += interface.robot_list[2].position;
				if (strategyGUI.strategy.pot_magnitude[i] != 0) {
				}
			}
			aux_point.x = static_cast<int>(round(100 * cos(strategyGUI.strategy.pot_goalTheta)));
			aux_point.y = static_cast<int>(-round(100 * sin(strategyGUI.strategy.pot_goalTheta)));
			aux_point += interface.robot_list[2].position;

			for (int i = 0; i < interface.visionGUI.vision->getAdvListSize(); i++)
				circle(imageView, interface.visionGUI.vision->getAdvRobot(i), 15, cv::Scalar(0, 0, 255), 2);
		} // if !interface.draw_info_flag
	} // if !draw_info_flag

	updateAllPositions();
	control.update_dropped_frames();

	if (interface.imageView.PID_test_flag && !interface.get_start_game_flag()) {
		control.button_PID_Test.set_active(true);
		PID_test();
	} else {
		for (auto &robot : interface.robot_list) {
			robot.target = cv::Point(-1, -1);
		}
		Selec_index = -1;
		control.PID_test_flag = false;
	}

	if (interface.imageView.PID_test_flag && interface.get_start_game_flag())
		control.button_PID_Test.set_active(false);


	// ----------- ESTRATEGIA -----------------//
	if (interface.get_start_game_flag()) {
		strategyGUI.strategy.set_Ball(interface.visionGUI.vision->getBall());
		Ball_Est = strategyGUI.strategy.get_Ball_Est();
		circle(imageView, Ball_Est, 7, cv::Scalar(255, 140, 0), 2);
		strategyGUI.strategy.get_targets(&(interface.robot_list), (interface.visionGUI.vision->getAllAdvRobots()));
		for (int i = 0; i < 3; i++) {
			circle(imageView, interface.robot_list[i].target, 7, cv::Scalar(127, 255, 127), 2);
			putText(imageView, std::to_string(i + 1),
					cv::Point(interface.robot_list[i].target.x - 5, interface.robot_list[i].target.y - 17),
					cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(127, 255, 127), 2);
		} // for

		interface.update_speed_progressBars();
		interface.update_robot_functions();
	} // if start_game_flag
	// ----------------------------------------//


	if (frameCounter == 30) {
		timer.stop();
		fps_average = static_cast<int>(30 / timer.getCronoTotalSecs());
		// cout<<"CPU Time: "<<timer.getCPUTotalSecs()<<",	\"CPU FPS\": "<<30/timer.getCPUTotalSecs()<<endl;
		// cout<<"FPS Time: "<<timer.getCronoTotalSecs()<<", FPS: "<<30/timer.getCronoTotalSecs()<<endl;
		timer.reset();
		frameCounter = 0;
	}

	interface.robot_list[0].position = robot_kf_est[0];
	interface.robot_list[1].position = robot_kf_est[1];
	interface.robot_list[2].position = robot_kf_est[2];
	if (interface.get_start_game_flag() || interface.imageView.PID_test_flag) {
		control.update_msg_time();
		notify_data_ready();
	}

	return true;
} // capture_and_show

void CamCap::send_cmd_thread(vector<Robot> &robots) {
	boost::unique_lock<boost::mutex> lock(data_ready_mutex);
	while (true) {
		try {
			data_ready_cond.wait(lock, [this]() { return data_ready_flag; });
		} catch (...) {
			lock.unlock();
			return;
		}
		data_ready_flag = false;
		control.messenger.send_cmds(robots);
	}
}

void CamCap::notify_data_ready() {
	data_ready_flag = true;
	data_ready_cond.notify_all();
}

double CamCap::distance(cv::Point a, cv::Point b) {
	return sqrt(pow(double(b.x - a.x), 2) + pow(double(b.y - a.y), 2));
}

void CamCap::PID_test() {
	if (interface.get_start_game_flag()) return;

	double dist;
	int old_Selec_index;
	old_Selec_index = Selec_index;
	for (int i = 0; i < interface.robot_list.size() && i < 3; i++) {
		dist = sqrt(pow((interface.imageView.robot_pos[0] - interface.robot_list[i].position.x), 2) +
					pow((interface.imageView.robot_pos[1] - interface.robot_list[i].position.y), 2));
		if (dist <= 17) {
			Selec_index = i;
			interface.imageView.tar_pos[0] = -1;
			interface.imageView.tar_pos[1] = -1;
			interface.robot_list[Selec_index].target = cv::Point(-1, -1);
			fixed_ball[Selec_index] = false;
		}
	}
	if (Selec_index > -1) {
		if (sqrt(pow((interface.visionGUI.vision->getBall().x - interface.robot_list[Selec_index].target.x), 2) +
				 pow((interface.visionGUI.vision->getBall().y - interface.robot_list[Selec_index].target.y), 2)) <=
			7)
			fixed_ball[Selec_index] = true;

		if (fixed_ball[Selec_index])
			interface.robot_list[Selec_index].target = interface.visionGUI.vision->getBall();
		else
			interface.robot_list[Selec_index].target = cv::Point(static_cast<int>(interface.imageView.tar_pos[0]),
																 static_cast<int>(interface.imageView.tar_pos[1]));
	}

	for (int i = 0; i < interface.robot_list.size() && i < 3; i++) {
		if (fixed_ball[i])
			interface.robot_list[i].target = interface.visionGUI.vision->getBall();
		else {
			if (sqrt(pow((interface.robot_list[i].position.x - interface.robot_list[i].target.x), 2) +
					 pow((interface.robot_list[i].position.y - interface.robot_list[i].target.y), 2)) < 15) {
				interface.robot_list[i].target = cv::Point(-1, -1);
				interface.imageView.tar_pos[0] = -1;
				interface.imageView.tar_pos[1] = -1;
				interface.robot_list[i].Vr = 0;
				interface.robot_list[i].Vl = 0;
				interface.robot_list[i].vmax = 0;
			}
			if (interface.robot_list[i].target.x != -1 && interface.robot_list[i].target.y != -1) {
				interface.robot_list[Selec_index].target = cv::Point(static_cast<int>(interface.imageView.tar_pos[0]),
																	 static_cast<int>(interface.imageView.tar_pos[1]));
				interface.robot_list[Selec_index].vmax = interface.robot_list[Selec_index].vdefault;
				interface.robot_list[i].cmdType = VECTOR;
				interface.robot_list[i].transAngle = atan2(
						double(interface.robot_list[i].position.y - interface.robot_list[i].target.y),
						-double(interface.robot_list[i].position.x - interface.robot_list[i].target.x));
				//interface.robot_list[i].goTo(interface.robot_list[i].target,interface.visionGUI.vision->getBall());
			} else {
				interface.robot_list[i].Vr = 0;
				interface.robot_list[i].Vl = 0;
			}
		}
	}
} // PID_test

void CamCap::warp_transform(cv::Mat imageView) {
	cv::Point2f inputQuad[4];
	cv::Point2f outputQuad[4];
	cv::Mat lambda = cv::Mat::zeros(imageView.rows, imageView.cols, imageView.type());

	inputQuad[0] = cv::Point2f(interface.imageView.warp_mat[0][0] - interface.offsetL,
							   interface.imageView.warp_mat[0][1]);
	inputQuad[1] = cv::Point2f(interface.imageView.warp_mat[1][0] + interface.offsetR,
							   interface.imageView.warp_mat[1][1]);
	inputQuad[2] = cv::Point2f(interface.imageView.warp_mat[2][0] + interface.offsetR,
							   interface.imageView.warp_mat[2][1]);
	inputQuad[3] = cv::Point2f(interface.imageView.warp_mat[3][0] - interface.offsetL,
							   interface.imageView.warp_mat[3][1]);

	outputQuad[0] = cv::Point2f(0, 0);
	outputQuad[1] = cv::Point2f(width - 1, 0);
	outputQuad[2] = cv::Point2f(width - 1, height - 1);
	outputQuad[3] = cv::Point2f(0, height - 1);
	lambda = getPerspectiveTransform(inputQuad, outputQuad);
	warpPerspective(imageView, imageView, lambda, imageView.size());

	if (interface.imageView.adjust_rdy) {
		interface.bt_adjust.set_active(false);
		interface.bt_adjust.set_state(Gtk::STATE_INSENSITIVE);
		interface.adjust_event_flag = false;
		interface.imageView.adjust_event_flag = false;

		for (int i = 0; i < interface.imageView.adjust_mat[0][1]; i++) {
			for (int j = 0; j < 3 * interface.imageView.adjust_mat[0][0]; j++) {
				imageView.at<uchar>(i, j) = 0;
			}
		}

		for (int i = height; i > interface.imageView.adjust_mat[1][1]; i--) {
			for (int j = 0; j < 3 * interface.imageView.adjust_mat[1][0]; j++) {
				imageView.at<uchar>(i, j) = 0;
			}
		}

		for (int i = 0; i < interface.imageView.adjust_mat[2][1]; i++) {
			for (int j = 3 * width; j > 3 * interface.imageView.adjust_mat[2][0]; j--) {
				imageView.at<uchar>(i, j) = 0;
			}
		}

		for (int i = height; i > interface.imageView.adjust_mat[3][1]; i--) {
			for (int j = 3 * width; j > 3 * interface.imageView.adjust_mat[3][0]; j--) {
				imageView.at<uchar>(i, j) = 0;
			}
		}
	}
} // warp_transform

CamCap::CamCap(int screenW, int screenH) : data(0), width(0), height(0), frameCounter(0),
										   screenWidth(screenW), screenHeight(screenH),
										   msg_thread(&CamCap::send_cmd_thread, this,
													  boost::ref(interface.robot_list)) {

	isLowRes = checkForLowRes();

	if (isLowRes) {
		interface.~V4LInterface();
		new(&interface) capture::V4LInterface(isLowRes);
	}

	fixed_ball[0] = false;
	fixed_ball[1] = false;
	fixed_ball[2] = false;
	fm.set_label("imageView");
	fm.add(interface.imageView);

	notebook.append_page(interface, "Capture");
	notebook.append_page(interface.visionGUI, "Vision");
	notebook.append_page(control, "Control");
	notebook.append_page(strategyGUI, "Strategy");

	robot_kf_est.push_back(Ball_Est); // Robot 1
	robot_kf_est.push_back(Ball_Est); // Robot 2
	robot_kf_est.push_back(Ball_Est); // Robot 3
	for (int i = 0; i < interface.visionGUI.vision->getAdvListSize(); i++) {
		robot_kf_est.push_back(Ball_Est); // Adv
	}

	KalmanFilter kf;
	KF_Robot.push_back(kf); // Robot 1
	KF_Robot.push_back(kf); // Robot 2
	KF_Robot.push_back(kf); // Robot 3
	KF_Robot.push_back(kf); // Ball
	for (int i = 0; i < interface.visionGUI.vision->getAdvListSize(); i++) {
		KF_Robot.push_back(kf); // Adv
	}

	for (int i = 0; i < 3; i++) {
		virtual_robots_orientations[i] = 0;
		virtual_robots_positions[i] = cv::Point(200, 480 / 6 + (i + 1) * 480 /
															   6); // !TODO hardcoded, usar variáveis quando possível
	}

	for (int i = 0; i < 4; i++) {
		interface.imageView.adjust_mat[i][0] = -1;
		interface.imageView.adjust_mat[i][1] = -1;
	}

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
