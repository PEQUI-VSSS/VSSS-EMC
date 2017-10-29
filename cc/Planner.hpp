#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include <vector>
#include "opencv2/opencv.hpp"
#include "../pack-capture-gui/capture-gui/Robot.hpp"

#define HIST_SIZE 3
#define DELTA_X 3
#define ROBOT_RADIUS 17

#define POSITION 0
#define SPEED 1
#define ORIENTATION 2
#define VECTOR 3
#define VECTORRAW 4

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

    // constantes
    int COORD_MID_FIELD_X, MEIO_GOL_Y, MAX_Y;
    // histórico de estados: armazena os HIST_SIZE últimos frames
    std::vector<State> hist;


    // recebe três pontos e retorna o vetor que deve ser executado para a curva nesse passo
    VelocityVector curve_control(cv::Point start, cv::Point mid, cv::Point end, double vdefault);

    // gera salva o estado atual no histórico
    void update_hist(State current_state);

    // pega os dados e gera um estado
    State gen_state(std::vector<Robot> robots, cv::Point * advRobots, cv::Point ball);

    // encontra os obstáculos entre o ponto de início e de fim
    std::vector<Obstacle> find_obstacles(State predicted_state, int start_index, cv::Point target);

    double distance_to_line(cv::Point start, cv::Point end, cv::Point mid);

    double distance(cv::Point p1, cv::Point p2);

    cv::Point * find_deviation(cv::Point start, cv::Point end, Obstacle obstacle);

public:
    // criar função de planejamento de trajetória
    // controle por curva

    // recebe um ponteiro para os robôs pois após tudo ser calculado os alvos devem ser atualizados
    void plan(std::vector<Robot> * pRobots, cv::Point * advRobots, cv::Point ball);

    void set_constants(int midfieldX, int midgoalY, int maxY);

    State predict_positions(double timeAhead);
};

#endif /* TAG_HPP_ */
