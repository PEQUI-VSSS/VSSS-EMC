#ifndef CAMCAP_HPP_
#define CAMCAP_HPP_
#define PI 3.14159265453
//dimensões dos quadrados do tabuleiro
#define CALIBRATION_SQUARE_DIMENSION 0.02629f
// número de intersecções do tabuleiro
#define CHESSBOARD_DIMENSION cv::Size(6,9)

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>

#include "StrategyGUI.hpp"
#include "controlGUI.hpp"
#include "vision/vision.hpp"
#include "KalmanFilter.hpp"
#include "V4LInterface.hpp"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <cstdlib>
#include <ctgmath>
#include <gtkmm.h>
#include <cmath>
#include <fstream>
#include "CPUTimer.h"
#include "Constants.hpp"

#define MAX_THETA_TOLERATION 3
#define MAX_POSITIONING_VEL 0.1

class CamCap : public Gtk::HBox {

	public:
		int screenWidth, screenHeight;
		bool isLowRes;

		int width, height;
		int Selec_index = -1;
		int fps_average = 0;
		CPUTimer timer;

		bool fixed_ball[3];
		bool KF_FIRST = true;

		unsigned char *data;

		int frameCounter;
		std::vector<cv::Point2f> robot_kf_est;
		std::vector<cv::Point2f> robot_kf_est_ini;
		std::vector<KalmanFilter> KF_Robot;

		cv::Point2f Ball_Est;
		cv::Point2f Ball_kf_est;
		//cv::Point2f Ball_kf_est_ini;

		cv::Point virtual_robots_positions[3];
		float virtual_robots_orientations[3];
		int virtual_robot_selected = -1;

		StrategyGUI strategyGUI;
		ControlGUI control;
		capture::V4LInterface interface;

		Gtk::Frame fm;
		Gtk::Frame info_fm;
		Gtk::VBox camera_vbox;
		Gtk::Notebook notebook;

		sigc::connection con;

		/* PARA TESTE */
		cv::Point obstacle;
		cv::Point deviation1;
		cv::Point deviation2;

		boost::thread msg_thread;
		boost::condition_variable data_ready_cond;
		boost::mutex data_ready_mutex;
		bool data_ready_flag = false;

		bool checkForLowRes();
		void updateAllPositions();
		void updateKalmanFilter();
		bool start_signal(bool b);
		bool capture_and_show();
		void send_cmd_thread(std::vector<Robot> &robots);
		void notify_data_ready();
		double distance(cv::Point a, cv::Point b);
		void PID_test();
		void warp_transform(cv::Mat imageView);
		CamCap(int screenW, int screenH);
		~CamCap() override;
};

#endif /* CAMCAP_HPP_ */
