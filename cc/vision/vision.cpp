#include <Geometry/Geometry.h>
#include <Strategy2/Field.h>
#include "vision.hpp"

using namespace vision;

std::array<Vision::RecognizedTag, 3> Vision::run(cv::Mat raw_frame) {
	in_frame = raw_frame.clone();

	if (bOnAir) recordToVideo();

	preProcessing();
	findTags();
	//findElements();
	return pick_a_tag();
}

void Vision::recordVideo(const cv::Mat frame) {
	in_frame = frame.clone();
	if (bOnAir) recordToVideo();
}

void Vision::runGMM(std::vector<cv::Mat> thresholds, std::vector<VisionROI> *windowsList) {
	searchGMMTags(std::move(thresholds));
	pick_a_tag(windowsList);
}

void Vision::preProcessing() {
	cv::cvtColor(in_frame, lab_frame, cv::COLOR_RGB2Lab);
}

void Vision::findTags() {
	for (int color = 0; color < MAX_COLORS; color++) {
		threshold_threads.add_thread(new boost::thread(&Vision::segmentAndSearch, this, color));
	}
	threshold_threads.join_all();
}

void Vision::segmentAndSearch(const unsigned long color) {
	cv::Mat frame = lab_frame.clone();

	inRange(frame, cv::Scalar(cieL[color][Limit::Min], cieA[color][Limit::Min], cieB[color][Limit::Min]),
			cv::Scalar(cieL[color][Limit::Max], cieA[color][Limit::Max], cieB[color][Limit::Max]), threshold_frame.at(color));

	posProcessing(color);
	searchTags(color);
}

void Vision::posProcessing(const unsigned long color) {
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

	if (blur[color] > 0) {
		cv::medianBlur(threshold_frame.at(color), threshold_frame.at(color), blur[color]);
	}
	cv::erode(threshold_frame.at(color), threshold_frame.at(color), erodeElement, cv::Point(-1, -1),
			  erode[color]);
	cv::dilate(threshold_frame.at(color), threshold_frame.at(color), dilateElement, cv::Point(-1, -1),
			   dilate[color]);
}

void Vision::searchGMMTags(std::vector<cv::Mat> thresholds) {
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	for (unsigned long color = 0; color < MAX_COLORS; color++) {
		tags.at(color).clear();

		cv::Mat tmp;
		cv::cvtColor(thresholds.at(color), tmp, CV_RGB2GRAY);

		cv::findContours(tmp, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

		for (const auto &contour : contours) {
			double area = contourArea(contour);
			if (area >= 10) {
				cv::Moments moment = moments((cv::Mat) contour);
				tags.at(color).emplace_back(cv::Point(static_cast<int>(moment.m10 / area),
													  static_cast<int>(moment.m01 / area)), area);

				// seta as linhas para as tags principais do pick-a-tag
				if (color == Color::Main) {
					cv::Vec4f line;
					cv::fitLine(cv::Mat(contour), line, 2, 0, 0.01, 0.01);
					unsigned long tagsInVec = tags.at(color).size() - 1;
					tags.at(color).at(tagsInVec).setLine(line);
				}
			}
		}
	}
}

void Vision::searchTags(const unsigned long color) {
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	tags.at(color).clear();

	cv::findContours(threshold_frame.at(color), contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

	for (const auto &contour : contours) {
		double area = contourArea(contour);
		if (area >= areaMin[color]) {
			cv::Moments moment = moments((cv::Mat) contour);
			auto moment_x = static_cast<int>(moment.m10 / area);
			auto moment_y = static_cast<int>(moment.m01 / area);
			// seta as linhas para as tags principais do pick-a-tag
			if (color == Color::Main) {
				tags.at(color).emplace_back(cv::Point(moment_x, moment_y), area);

				// tem que ter jogado a tag no vetor antes de mexer nos valores dela
				cv::Vec4f line;
				cv::fitLine(cv::Mat(contour), line, 2, 0, 0.01, 0.01);
				unsigned long tagsInVec = tags.at(color).size() - 1;
				tags.at(color).at(tagsInVec).setLine(line);
			} else if (color == Color::Adv) {
				if (tags.at(color).size() >= 3) {
					// pega o menor índice
					unsigned long smaller = 0;
					for (unsigned long j = 1; j < 3; j++) {
						if (tags.at(color).at(j).area < tags.at(color).at(smaller).area)
							smaller = j;
					}
					if (smaller < tags.at(color).size() && area > tags.at(color).at(smaller).area)
						tags.at(color).at(smaller) = Tag(cv::Point(moment_x, moment_y), area);
				} else {
					tags.at(color).emplace_back(cv::Point(moment_x, moment_y), area);
				}
			} else {
				tags.at(color).emplace_back(cv::Point(moment_x, moment_y), area);
			}
		}
	}
}

/// <summary>
/// Seleciona um conjunto de tags para representar cada robô. Adaptado aos ROIs utilizados pelo GMM
/// </summary>
/// <description>
/// P.S.: Aqui eu uso a flag 'isOdd' para representar quando um robô tem as duas bolas laterais.
/// </description>
void Vision::pick_a_tag(std::vector<VisionROI> *windowsList) {
//	int dist, tmpSide;
//
//	// Define inicialmente que o objeto de cada janela não foi encontrado
//	for (int i = 0; i < windowsList->size(); i++) {
//		windowsList->at(i).setIsLost(true);
//	}
//
//	// OUR ROBOTS
//	for (int i = 0; i < tags.at(Color::Main).size() && i < 3; i++) {
//		// cria um robô temporário para armazenar nossas descobertas
//		Robot robot;
//		std::vector<Tag> tempTags;
//
//		// Posição do robô
//		robot.position = tags.at(Color::Main).at(i).position;
//
//		// Cálculo da orientação de acordo com os pontos rear e front
//		robot.orientation = atan2((tags.at(Color::Main).at(i).frontPoint.y - robot.position.y) * field::field_height / height,
//								  (tags.at(Color::Main).at(i).frontPoint.x - robot.position.x) * field::field_width / width);
//
//		// Armazena a tag
//		tempTags.push_back(tags.at(Color::Main).at(i));
//
//		// Para cada tag principal, verifica quais são as secundárias correspondentes
//		for (int j = 0; j < tags.at(Color::Green).size(); j++) {
//			// já faz a atribuição verificando se o valor retornado é 0 (falso); além disso, altera a orientação caso esteja errada
//			if (tmpSide = in_sphere(tags.at(Color::Green).at(j).position, Tag(cv::Point_(), 0), &tempTags, &robot)) {
//				// identifica se já tem mais de uma tag
//				if (tempTags.size() > 1) {
//					robot.isOdd = true;
//				}
//				tags.at(Color::Green).at(j).left = (tmpSide > 0) ? true : false;
//				// calculos feitos, joga tag no vetor
//				tempTags.push_back(tags.at(Color::Green).at(j));
//			}
//		}
//
//
//		// Dá nome aos bois (robôs)
//		if (robot.isOdd) { // isOdd representa que este tem as duas bolas
//			robot_list.at(2).position = robot.position; // colocar em um vetor
//			robot_list.at(2).secundary = tempTags.at(0).frontPoint; // colocar em um vetor
//			robot_list.at(2).orientation = robot.orientation;
//			robot_list.at(2).rearPoint = tempTags.at(0).rearPoint;
//			windowsList->at(2).setIsLost(false);
//			windowsList->at(2).setCenter(robot.position);
//		} else if (tempTags.size() > 1 && tempTags.at(1).left) {
//			robot_list.at(0).position = robot.position; // colocar em um vetor
//			robot_list.at(0).secundary = tempTags.at(0).frontPoint; // colocar em um vetor
//			robot_list.at(0).orientation = robot.orientation;
//			robot_list.at(0).rearPoint = tempTags.at(0).rearPoint;
//			windowsList->at(0).setIsLost(false);
//			windowsList->at(0).setCenter(robot.position);
//		} else {
//			robot_list.at(1).position = robot.position; // colocar em um vetor
//			robot_list.at(1).secundary = tempTags.at(0).frontPoint; // colocar em um vetor
//			robot_list.at(1).orientation = robot.orientation;
//			robot_list.at(1).rearPoint = tempTags.at(0).rearPoint;
//			windowsList->at(1).setIsLost(false);
//			windowsList->at(1).setCenter(robot.position);
//		}
//	} // OUR ROBOTS
//
//	// ADV ROBOTS
//	for (int i = 0; i < tags.at(ADV).size() && i < MAX_ADV; i++) {
//		advRobots[i] = tags.at(ADV).at(i).position;
//		windowsList->at(4 + i).setIsLost(false);
//		windowsList->at(4 + i).setCenter(advRobots[i]);
//	}
//
//	// BALL POSITION
//	if (!tags[BALL].empty()) {
//		ball = tags.at(BALL).at(0).position;
//		windowsList->at(3).setIsLost(false);
//		// std::cout << "Ball: " << ball.x << ", " << ball.y << std::endl;
//		windowsList->at(3).setCenter(ball);
//	}
}

/// <summary>
/// Seleciona um conjunto de tags para representar cada robô
/// </summary>
/// <description>
/// P.S.: Aqui eu uso a flag 'isOdd' para representar quando um robô tem as duas bolas laterais.
/// </description>
std::array<Vision::RecognizedTag, 3> Vision::pick_a_tag() {
	std::array<RecognizedTag, 3> found_tags{};

	// OUR ROBOTS
	for (int i = 0; i < tags.at(Color::Main).size() && i < 3; i++) {
		std::vector<Tag> secondary_tags;

		Tag main_tag = tags.at(Color::Main).at(i);

		// Posição do robô
		cv::Point position = main_tag.position;

		// Cálculo da orientação de acordo com os pontos rear e front
		double orientation = atan2((main_tag.frontPoint.y - position.y) * field::field_height / height,
								  (main_tag.frontPoint.x - position.x) * field::field_width / width);

		// Para cada tag principal, verifica quais são as secundárias correspondentes
		for (Tag &secondary_tag : tags.at(Color::Green)) {

			// Altera a orientação caso esteja errada
			int tag_side = in_sphere(secondary_tag.position, &main_tag, &secondary_tags, &orientation);
			if (tag_side != 0) {
				secondary_tag.left = tag_side > 0;
				// calculos feitos, joga tag no vetor
				secondary_tags.push_back(secondary_tag);
			}
		}

		if (secondary_tags.size() > 1) {
			// tag 3 tem duas tags secundárias
			found_tags[2] = {position, orientation,
							 main_tag.frontPoint, main_tag.rearPoint};
		} else if (!secondary_tags.empty() && !secondary_tags[0].left) {
			found_tags[1] = {position, orientation,
							 main_tag.frontPoint, main_tag.rearPoint};
		} else if (!secondary_tags.empty()){
			found_tags[0] = {position, orientation,
							 main_tag.frontPoint, main_tag.rearPoint};
		}
	} // OUR ROBOTS

	// ADV ROBOTS
	for (unsigned long i = 0; i < MAX_ADV; i++) {
		if (i < tags.at(Color::Adv).size())
			advRobots[i] = tags.at(Color::Adv).at(i).position;
		else
			advRobots[i] = cv::Point(-1, -1);
	}

	// BALL POSITION
	if (!tags[Color::Ball].empty()) {
		ball = tags.at(Color::Ball).at(0).position;
	}

	return found_tags;
}

//Vision::RecognizedTag Vision::pick_a_tag() {
//	int dist, tmpSide;
//
//	// OUR ROBOTS
//	for (int i = 0; i < tags.at(Color::Main).size() && i < 3; i++) {
//		// cria um robô temporário para armazenar nossas descobertas
//		Robot robot;
//		std::vector<Tag> tempTags;
//
//		// Posição do robô
//		auto position = tags.at(Color::Main).at(i).position;
//
//		// Cálculo da orientação de acordo com os pontos rear e front
//		robot.orientation = atan2((tags.at(Color::Main).at(i).frontPoint.y - robot.position.y) * field::field_height / height,
//								  (tags.at(Color::Main).at(i).frontPoint.x - robot.position.x) * field::field_width / width);
//
//		// Armazena a tag
//		tempTags.push_back(tags.at(Color::Main).at(i));
//
//		// Para cada tag principal, verifica quais são as secundárias correspondentes
//		for (int j = 0; j < tags.at(Color::Green).size(); j++) {
//			// já faz a atribuição verificando se o valor retornado é 0 (falso); além disso, altera a orientação caso esteja errada
//			if (tmpSide = in_sphere(tags.at(Color::Green).at(j).position, Tag(cv::Point_(), 0), &tempTags, &robot)) {
//				// identifica se já tem mais de uma tag
//				if (tempTags.size() > 1) {
//					robot.isOdd = true;
//				}
//				tags.at(Color::Green).at(j).left = (tmpSide > 0) ? true : false;
//				// calculos feitos, joga tag no vetor
//				tempTags.push_back(tags.at(Color::Green).at(j));
//			}
//		}
//
//
//		// Dá nome aos bois (robôs)
//		if (robot.isOdd) { // isOdd representa que este tem as duas bolas
//			robot_list.at(2).position = robot.position; // colocar em um vetor
//			robot_list.at(2).secundary = tempTags.at(0).frontPoint; // colocar em um vetor
//			robot_list.at(2).orientation = robot.orientation;
//			robot_list.at(2).rearPoint = tempTags.at(0).rearPoint;
//		} else if (tempTags.size() > 1 && tempTags.at(1).left) {
//			robot_list.at(0).position = robot.position; // colocar em um vetor
//			robot_list.at(0).secundary = tempTags.at(0).frontPoint; // colocar em um vetor
//			robot_list.at(0).orientation = robot.orientation;
//			robot_list.at(0).rearPoint = tempTags.at(0).rearPoint;
//		} else {
//			robot_list.at(1).position = robot.position; // colocar em um vetor
//			robot_list.at(1).secundary = tempTags.at(0).frontPoint; // colocar em um vetor
//			robot_list.at(1).orientation = robot.orientation;
//			robot_list.at(1).rearPoint = tempTags.at(0).rearPoint;
//		}
//	} // OUR ROBOTS
//
//	// ADV ROBOTS
//	for (int i = 0; i < MAX_ADV; i++) {
//		if (i < tags.at(ADV).size())
//			advRobots[i] = tags.at(ADV).at(i).position;
//		else
//			advRobots[i] = cv::Point(-1, -1);
//	}
//
//	// BALL POSITION
//	if (!tags[BALL].empty()) {
//		ball = tags.at(BALL).at(0).position;
//	}
//}

/// <summary>
/// Verifica se uma tag secundária pertence a esta pick-a e calcula seu delta.
/// </summary>
/// <param name="position">Posição central do robô</param>
/// <param name="secondary">O suposto ponto que marca uma bola da tag</param>
/// <param name="orientation">A orientação do robô</param>
/// <returns>
/// 0, se esta não é uma tag secundária;
/// -1, caso a secundária esteja à esquerda;
/// 1, caso a secundária esteja à direita
/// </returns>
int Vision::in_sphere(cv::Point secondary, Tag *main_tag, std::vector<Tag> *secondary_tags, double *orientation) {
	// se esta secundária faz parte do robô
	if (calcDistance(main_tag->position, secondary) <= ROBOT_RADIUS) {
		if (calcDistance(main_tag->frontPoint, secondary) < calcDistance(main_tag->rearPoint, secondary)) {
			main_tag->switchPoints();
			// recalcula a orientação com os novos pontos (isso só é feito uma vez em cada robô, se necessário)
			*orientation = atan2((main_tag->frontPoint.y - main_tag->position.y) * field::field_height / height,
								(main_tag->frontPoint.x - main_tag->position.x) * field::field_width / width);
		}

		float secSide = atan2((secondary.y - main_tag->position.y) * field::field_height / height,
							  (secondary.x - main_tag->position.x) * field::field_width / width);

		// Cálculo do ângulo de orientação para diferenciar robôs de mesma cor
		return (atan2(sin(secSide - *orientation + 3.1415), cos(secSide - *orientation + 3.1415))) > 0 ? 1 : -1;
	}
	return 0;
}

//int Vision::in_sphere(Robot *robot, std::vector<Tag> *tempTags, cv::Point secondary) {
//	// se esta secundária faz parte do robô
//	if (calcDistance(robot->position, secondary) <= ROBOT_RADIUS) {
//		if (calcDistance(tempTags->at(0).frontPoint, secondary) < calcDistance(tempTags->at(0).rearPoint, secondary)) {
//			tempTags->at(0).switchPoints();
//			// recalcula a orientação com os novos pontos (isso só é feito uma vez em cada robô, se necessário)
//			robot->orientation = atan2((tempTags->at(0).frontPoint.y - robot->position.y) * field::field_height / height,
//									   (tempTags->at(0).frontPoint.x - robot->position.x) * field::field_width / width);
//		}
//
//		float secSide = atan2((secondary.y - robot->position.y) * field::field_height / height,
//							  (secondary.x - robot->position.x) * field::field_width / width);
//
//		// Cálculo do ângulo de orientação para diferenciar robôs de mesma cor
//		return (atan2(sin(secSide - robot->orientation + 3.1415), cos(secSide - robot->orientation + 3.1415))) > 0 ? 1
//																												   : -1;
//	}
//	return 0;
//}

double Vision::calcDistance(const cv::Point p1, const cv::Point p2) const {
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void Vision::switchMainWithAdv() {
	int tmp;

	for (int i = Limit::Min; i <= Limit::Max; i++) {
		tmp = cieL[Color::Main][i];
		cieL[Color::Main][i] = cieL[Color::Adv][i];
		cieL[Color::Adv][i] = tmp;

		tmp = cieA[Color::Main][i];
		cieA[Color::Main][i] = cieA[Color::Adv][i];
		cieA[Color::Adv][i] = tmp;

		tmp = cieB[Color::Main][i];
		cieB[Color::Main][i] = cieB[Color::Adv][i];
		cieB[Color::Adv][i] = tmp;
	}

	tmp = areaMin[Color::Main];
	areaMin[Color::Main] = areaMin[Color::Adv];
	areaMin[Color::Adv] = tmp;

	tmp = erode[Color::Main];
	erode[Color::Main] = erode[Color::Adv];
	erode[Color::Adv] = tmp;

	tmp = dilate[Color::Main];
	dilate[Color::Main] = dilate[Color::Adv];
	dilate[Color::Adv] = tmp;

	tmp = blur[Color::Main];
	blur[Color::Main] = blur[Color::Adv];
	blur[Color::Adv] = tmp;
}

cv::Mat Vision::getSplitFrame() {
	cv::Mat horizontal[2], vertical[2];

	for (unsigned long index = 0; index < 3; index++) {
		cv::cvtColor(threshold_frame.at(index), threshold_frame.at(index), cv::COLOR_GRAY2RGB);
	}

	cv::pyrDown(in_frame, vertical[0]);
	cv::pyrDown(threshold_frame.at(0), vertical[1]);

	cv::hconcat(vertical, 2, horizontal[0]);

	cv::pyrDown(threshold_frame.at(1), vertical[0]);
	cv::pyrDown(threshold_frame.at(2), vertical[1]);

	cv::hconcat(vertical, 2, horizontal[1]);

	cv::vconcat(horizontal, 2, splitFrame);

	return splitFrame;
}

void Vision::saveCamCalibFrame() {
//    cv::Mat temp = rawFrameCamcalib.clone();
	cv::Mat temp = in_frame.clone();
	savedCamCalibFrames.push_back(temp);
	std::string text = "CamCalib_" + std::to_string(getCamCalibFrames().size());
	saveCameraCalibPicture(text, "media/pictures/camCalib/");
	std::cout << "Saving picture " << std::endl;
}

void Vision::collectImagesForCalibration() {
	std::cout << "Collecting pictures " << std::endl;
	cv::String path("media/pictures/camCalib/*.png"); //select only png
	std::vector<cv::String> fn;
	std::vector<cv::Mat> data;
    try{
        cv::glob(path, fn, true); // recurse
        for (auto &index : fn) {
            cv::Mat im = cv::imread(index);
            if (im.empty()) continue; //only proceed if sucsessful
            // you probably want to do some preprocessing
            savedCamCalibFrames.push_back(im);
        }
        std::cout << "Pictures collected: " << savedCamCalibFrames.size() << std::endl;
        cameraCalibration();

    }catch (...){
        std::cout << "An exception occurred. No images for calibration. \n";
    }

}

void Vision::cameraCalibration() {

	std::vector<std::vector<cv::Point2f>> checkerBoardImageSpacePoints;
    std::vector<std::vector<cv::Point3f>> worldSpaceCornersPoints;

	getChessBoardCorners(savedCamCalibFrames, worldSpaceCornersPoints, checkerBoardImageSpacePoints);
	std::cout << "Image Space Points " << checkerBoardImageSpacePoints.size() << std::endl;

	std::cout << "world SpaceCorners Points " << worldSpaceCornersPoints.size() << std::endl;
	std::vector<cv::Mat> rVectors, tVectors;
	distanceCoeficents = cv::Mat::zeros(8, 1, CV_64F);

    int flag = 0;
    flag |= CV_CALIB_FIX_K4;
    flag |= CV_CALIB_FIX_K5;

	//root mean square (RMS) reprojection error and should be between 0.1 and 1.0 pixels in a good calibration.
    double rms = cv::calibrateCamera(worldSpaceCornersPoints, checkerBoardImageSpacePoints, in_frame.size(),
                                     cameraMatrix, distanceCoeficents, rVectors, tVectors, flag);

	savedCamCalibFrames.clear();

	flag_cam_calibrated = true;

	std::cout << "Camera parameters matrix." << std::endl;
	std::cout << cameraMatrix << std::endl;
	std::cout << "Camera distortion coefficients" << std::endl;
	std::cout << distanceCoeficents << std::endl;
	std::cout << "RMS" << std::endl;
	std::cout << rms << std::endl;
	std::cout << "End of calibration" << std::endl;
}


void Vision::getChessBoardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point3f>>& pts3d,std::vector<std::vector<cv::Point2f>>& pts2d) const {
	cv::TermCriteria termCriteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
	cv::Mat grayFrame;
	//std::vector<std::vector<cv::Point2f>> allFoundCorners;
	for (auto &image : images) {
		std::vector<cv::Point2f> pointBuf;
        std::vector<cv::Point3f> corners;

		bool found = cv::findChessboardCorners(image, CHESSBOARD_DIMENSION, pointBuf,
											   CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        for (int i = 0; i < CHESSBOARD_DIMENSION.height; i++) {
            for (int j = 0; j < CHESSBOARD_DIMENSION.width; ++j) {
                corners.emplace_back(j * CALIBRATION_SQUARE_DIMENSION, i * CALIBRATION_SQUARE_DIMENSION, 0.0f);
            }
        }

		if (found) {
			cv::cvtColor(image, grayFrame, cv::COLOR_RGB2GRAY);
			cv::cornerSubPix(grayFrame, pointBuf, cv::Size(5, 5), cv::Size(-1, -1), termCriteria);
			pts2d.push_back(pointBuf);
            pts3d.push_back(corners);
		}
	}
}

bool Vision::foundChessBoardCorners() const {
	std::vector<cv::Vec2f> foundPoints;
	cv::Mat temp;
	temp = in_frame.clone();

	return cv::findChessboardCorners(temp, CHESSBOARD_DIMENSION, foundPoints,
									 CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
}

void Vision::saveCameraCalibPicture(const std::string in_name, const std::string directory) {
	cv::Mat frame = in_frame.clone();
	std::string picName = directory + in_name + ".png";

	cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
	cv::imwrite(picName, frame);
}

void Vision::savePicture(const std::string in_name) {
	cv::Mat frame = in_frame.clone();
	std::string picName = "media/pictures/" + in_name + ".png";

	cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
	cv::imwrite(picName, frame);
}

void Vision::startNewVideo(const std::string in_name) {
	std::string videoName = "media/videos/" + in_name + ".avi";

	video.open(videoName, CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(width, height));
	std::cout << "Started a new video recording." << std::endl;
	bOnAir = true;
}

bool Vision::recordToVideo() {
	if (!video.isOpened()) return false;
	cv::Mat frame = in_frame.clone();
	cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

	video.write(frame);
	return true;
}

bool Vision::finishVideo() {
	if (!video.isOpened()) return false;

	std::cout << "Finished video recording." << std::endl;
	video.release();
	bOnAir = false;
	return true;
}

cv::Point Vision::getAdvRobot(const int index) const {
	if (index < 0 || index >= MAX_ADV) {
		std::cout << "Vision::getAdvRobot: index argument is invalid." << std::endl;
		return cv::Point(-1, -1);
	} else {
		return advRobots[index];
	}
}

cv::Mat Vision::getThreshold(const unsigned long index) {
	cv::cvtColor(threshold_frame.at(index), threshold_frame.at(index), cv::COLOR_GRAY2RGB);
	return threshold_frame.at(index);
}

void Vision::setFlagCamCalibrated(const bool value) {
	this->flag_cam_calibrated = value;
}

void Vision::popCamCalibFrames() {
	savedCamCalibFrames.pop_back();
}

void Vision::setCIE_L(const unsigned long index0, const int index1, const int inValue) {
	if (index0 < MAX_COLORS && (index1 == Limit::Min || index1 == Limit::Max)) cieL[index0][index1] = inValue;
	else std::cout << "Vision:setCIE_L: could not set (invalid index)" << std::endl;
}

void Vision::setCIE_A(const unsigned long index0, const int index1, const int inValue) {
	if (index0 < MAX_COLORS && (index1 == Limit::Min || index1 == Limit::Max)) cieA[index0][index1] = inValue;
	else std::cout << "Vision:setCIE_A: could not set (invalid index)" << std::endl;
}

void Vision::setCIE_B(const unsigned long index0, const int index1, const int inValue) {
	if (index0 < MAX_COLORS && (index1 == Limit::Min || index1 == Limit::Max)) cieB[index0][index1] = inValue;
	else std::cout << "Vision:setCIE_B: could not set (invalid index)" << std::endl;
}

void Vision::setErode(const unsigned long index, const int inValue) {
	if (index < MAX_COLORS)
		erode[index] = inValue;
	else std::cout << "Vision:setErode: could not set (invalid index or convert type)" << std::endl;
}

void Vision::setDilate(const unsigned long index, const int inValue) {
	if (index < MAX_COLORS)
		dilate[index] = inValue;
	else std::cout << "Vision:setDilate: could not set (invalid index or convert type)" << std::endl;
}

void Vision::setBlur(const unsigned long index, int inValue) {
	if (index < MAX_COLORS) {
		if (inValue == 0 || inValue % 2 == 1)
			blur[index] = inValue;
		else
			blur[index] = inValue+1;
	} else {
		std::cout << "Vision:setBlur: could not set (invalid index or convert type)" << std::endl;
	}
}

void Vision::setAmin(const unsigned long index, const int inValue) {
	if (index < MAX_COLORS)
		areaMin[index] = inValue;
	else std::cout << "Vision:setAmin: could not set (invalid index or convert type)" << std::endl;
}

void Vision::setFrameSize(const int inWidth, const int inHeight) {
	if (inWidth >= 0) width = inWidth;
	if (inHeight >= 0) height = inHeight;
}

Vision::Vision(int w, int h) : width(w), height(h), cieL{{0, 255}, {0, 255}, {0, 255}, {0, 255}},
							   cieA{{0, 255}, {0, 255}, {0, 255}, {0, 255}}, video_rec_enable(true),
							   cieB{{0, 255}, {0, 255}, {0, 255}, {0, 255}}, bOnAir(false),
							   threshold_frame(MAX_COLORS), tags(MAX_COLORS), areaMin{0, 0, 0, 0},
							   dilate{0, 0, 0, 0}, erode{0, 0, 0, 0}, blur{3, 3, 3, 3} {
}

Vision::~Vision() = default;




