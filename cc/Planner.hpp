#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include <vector>
#include "opencv2/opencv.hpp"
#include "../pack-capture-gui/capture-gui/Robot.hpp"
#include "Constants.hpp"

#define PI 3.14159265453
#define HIST_SIZE 3
#define DELTA_X 3
#define ROBOT_RADIUS 17
#define DEVIATION_DELTA 4

#define POSITION 0
#define SPEED 1
#define ORIENTATION 2
#define VECTOR 3
#define VECTORRAW 4

using namespace CONST;

class Planner {
private:
    // tipos de dados
    typedef struct _obstacle {
        cv::Point position;
        double distance;
    } Obstacle;
    typedef struct _state {
        std::vector<cv::Point> objects;
    } State;
    typedef struct _velocityVector {
        double angle;
        float velocity;
    } VelocityVector;

    // histórico de estados: armazena os HIST_SIZE últimos frames
    std::vector<State> hist;
    // planner habilitado?
    bool use_this = true;

    // gera salva o estado atual no histórico
    void update_hist(State current_state);

    // pega os dados e gera um estado
    State gen_state(std::vector<Robot> robots, cv::Point * advRobots, cv::Point ball);

    // encontra os obstáculos entre o ponto de início e de fim
    std::vector<Obstacle> find_obstacles(State predicted_state, cv::Point startPos, cv::Point target, int start_index);

    double distance(cv::Point p1, cv::Point p2);

    double distance_to_line(cv::Point start, cv::Point end, cv::Point point);

    cv::Point * find_deviation(cv::Point start, cv::Point end, Obstacle obstacle);

    bool validate_target(cv::Point target);

    bool validate_shot_target(cv::Point target);

    cv::Point crop_target(cv::Point target);

    cv::Point * find_kick_deviation(cv::Point start, cv::Point end, Planner::Obstacle obstacle);

public:
    // criar função de planejamento de trajetória
    // controle por curva

    // recebe um ponteiro para os robôs pois após tudo ser calculado os alvos devem ser atualizados
    void plan(int robot_index, std::vector<Robot> * pRobots);

    void update_planner(std::vector<Robot> robots, cv::Point * advRobots, cv::Point ball, bool use_this);

    State predict_positions(double timeAhead);

    cv::Point best_shot_target(int robot_index);
};

#endif /* TAG_HPP_ */