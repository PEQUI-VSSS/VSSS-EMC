#ifndef TAG_HPP_
#define TAG_HPP_

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>

#define ROBOT_RADIUS 15 // valor encontrado empiricamente (quanto menor, mais próximos os robôs podem ficar sem dar errado)

class Tag {
	public:
		cv::Point position;
		cv::Point frontPoint;
		cv::Point rearPoint;

		double area;

		bool left;

		/// <summary>
		/// Constroi a tag
		/// </summary>
		/// <param name="pos">Posição da tag</param>
		/// <param name="myarea">Corresponde ao tagArea</param>
		Tag(cv::Point pos, double myarea);

		/// <summary>
		/// Seta os pontos frontPoint e rearPoint aleatoriamente
		/// </summary>
		/// <param name="myLine">Linha encontrada pelo fitline</param>
		void setLine(cv::Vec4f myLine);

		/// <summary>
		/// Inverte os pontos frontPoint e rearPoint depois que a visão determinar a ordem correta
		/// </summary>
		void switchPoints();
};

#endif /* TAG_HPP_ */
