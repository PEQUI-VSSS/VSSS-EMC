#ifndef VISION_HPP_
#define VISION_HPP_
//dimensões dos quadrados do tabuleiro
#define CALIBRATION_SQUARE_DIMENSION 0.02629f
// número de intersecções do tabuleiro
#define CHESSBOARD_DIMENSION cv::Size(6,9)

#include "Geometry/Geometry.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include "tag.hpp"
#include "visionROI.hpp"

namespace vision
{
	// Todas as cores que serão encontradas pela visão:
	enum Color
	{
		Main,
		Green,
		Ball,
		Adv
	};

	// Limites Min e Max dos tresholds:
	enum Limit
	{
		Min,
		Max
	};

	class Vision {
	private:

		static const int MAX_ADV = 3;

	public:

		static const unsigned long MAX_COLORS = Color::Adv - Color::Main + 1;
//		Numero da tag é definido pela sua posicao no std::array retornado
		struct RecognizedTag {
			cv::Point position = {ROBOT_RADIUS/2, 0};
			double orientation = 0;
			cv::Point front_point = {ROBOT_RADIUS, 0};
			cv::Point rear_point = {0, 0};
		};

	private:

		// Frames
		cv::Mat in_frame, lab_frame;
		std::vector<cv::Mat> threshold_frame;
		cv::Mat splitFrame;

		// Robots
		cv::Point advRobots[MAX_ADV];

		// Ball
		cv::Point ball;

		// TAGS
		std::vector<std::vector<Tag>> tags;

		// HSV/CIELAB Calibration Parameters
		int cieL[MAX_COLORS][2];
		int cieA[MAX_COLORS][2];
		int cieB[MAX_COLORS][2];
		int dilate[MAX_COLORS];
		int erode[MAX_COLORS];
		int blur[MAX_COLORS];
		int areaMin[MAX_COLORS];

		// image size
		int width;
		int height;

		// record video flag
		bool bOnAir;

		// video
		cv::VideoWriter video;

		// threads
		boost::thread_group threshold_threads;

		void preProcessing();
		void posProcessing(unsigned long color);
		void segmentAndSearch(unsigned long color);
		void searchTags(unsigned long color);
		void searchGMMTags(std::vector<cv::Mat> thresholds);
		void findTags();
		void pick_a_tag(std::vector<VisionROI> *windowsList);
		std::array<RecognizedTag, 3> pick_a_tag();
		int in_sphere(cv::Point secondary, Tag *main_tag, std::vector<Tag> *secondary_tags, double *orientation);

	public:

		Vision(int w, int h);
		~Vision();

		std::array<RecognizedTag, 3> run(cv::Mat raw_frame);
		void runGMM(std::vector<cv::Mat> thresholds, std::vector<VisionROI> *windowsList);
		void recordVideo(cv::Mat frame);
		double calcDistance(cv::Point p1, cv::Point p2) const;
		void saveCameraCalibPicture(std::string in_name, std::string directory);
		void startNewVideo(std::string videoName);
		bool recordToVideo();
		bool finishVideo();
		bool isRecording() const { return bOnAir; };
		void savePicture(std::string in_name);

		//video
		bool video_rec_enable;

		//Cam calib
		std::vector<cv::Mat> savedCamCalibFrames;
		cv::Mat cameraMatrix;
		cv::Mat distanceCoeficents;
		bool flag_cam_calibrated = false;
		void getChessBoardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point3f>>& pts3d,std::vector<std::vector<cv::Point2f>>& pts2d) const;
		std::vector<cv::Mat> getCamCalibFrames() const { return savedCamCalibFrames; };
		cv::Mat getcameraMatrix() const { return cameraMatrix; };
		cv::Mat getdistanceCoeficents() const { return distanceCoeficents; };
		void setFlagCamCalibrated(bool value);
		void saveCamCalibFrame();
		void popCamCalibFrames();
		void collectImagesForCalibration();
		void cameraCalibration();
		std::vector<cv::Point3f> createKnownBoardPosition(cv::Size boardSize, float squareEdgeLenght);
		bool foundChessBoardCorners() const;
		void switchMainWithAdv();

		cv::Point getBall() const { return ball; };
		cv::Point getAdvRobot(int index) const;
		cv::Mat getSplitFrame();

		int getAdvListSize() const { return MAX_ADV; };
		cv::Mat getThreshold(unsigned long index);

		int getCIE_L(unsigned long index0, int index1) const { return cieL[index0][index1]; };
		int getCIE_A(unsigned long index0, int index1) const { return cieA[index0][index1]; };
		int getCIE_B(unsigned long index0, int index1) const { return cieB[index0][index1]; };
		int getErode(unsigned long index) const { return erode[index]; };
		int getDilate(unsigned long index) const { return dilate[index]; };
		int getBlur(unsigned long index) const { return blur[index]; };
		int getAmin(unsigned long index) const { return areaMin[index]; };

		void setFrameSize(int inWidth, int inHeight);

		void setCIE_L(unsigned long index0, int index1, int inValue);
		void setCIE_A(unsigned long index0, int index1, int inValue);
		void setCIE_B(unsigned long index0, int index1, int inValue);
		void setErode(unsigned long index, int inValue);
		void setDilate(unsigned long index, int inValue);
		void setBlur(unsigned long index, int inValue);
		void setAmin(unsigned long index, int inValue);
	};
}


#endif /* VISION_HPP_ */
