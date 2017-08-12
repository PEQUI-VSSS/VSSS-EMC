#ifndef VISION_HPP_
#define VISION_HPP_

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include "../pack-capture-gui/capture-gui/Robot.hpp"
#include <iostream>     // std::cout
#include "Kalman_Filter.hpp"



class Vision
{
public:
  int const HWS = 50; // Half Window Size
  cv::Point ball_p1;
  cv::Point ball_p2;
  cv::Point robot_p1[6];
  cv::Point robot_p2[6];
  boost::thread_group threshold_threads;
  boost::thread_group threshold_threadsWindow;
  cv::Point  Ballorigin;
  cv::Point robotOrigin[6];

  vector< KalmanFilter > KF_RobotBall;
  // Team_Main[INDEX] - Vector de cv::Point
  //   GUARDA A POSIÇÃO DAS TAGS PRIMÁRIAS DO TIME(.x e .y acessam a posição)
  vector< cv::Point > Team_Main;
  vector<vector< cv::Point >> TeamMainWindow;
  // Team_Sec[COLOR_INDEX][INDEX] - Vector de Vector de cv::Point
  //	 CADA POSIÇÃO GUARDA UM VECTOR DE cv::Point PARA CADA COR SECUNDÁRIA DO TIME
  vector<vector< cv::Point >> Team_Sec;
  vector<vector< double >> Team_Sec_area;
  vector<vector< vector <cv::Point> >> TeamSecWindow;
  vector<vector< vector <double> >> TeamSecWindowArea;

  // ADV_Main[INDEX] - Vector de cv::Point
  //   GUARDA A POSIÇÃO DAS TAGS PRIMÁRIAS DO ADVERSÁRIO(.x e .y acessam a posição)
  vector<vector< cv::Point >> Adv_Main;
  // Ball - cv::Point
  //   GUARDA A POSIÇÃO DA BOLA
  cv::Point Ball;
  cv::Point KF_Ball_point;
  std::vector<cv::Point> KF_Robot_point;
  std::vector<Robot> robot_list;
  bool Ball_lost;
  bool robot_lost[6];
  int hue[5][2];
  int saturation[5][2];
  int value[5][2];
  int delation[5];
  int erosion[5];
  int blur[5];
  int areaMin[5];

  int width;
  int height;

  unsigned char **threshold = NULL;

  void resetKF()
  {
    KalmanFilter kf;
    KF_RobotBall.clear();
    KF_RobotBall.push_back(kf);
    KF_RobotBall.push_back(kf);
    KF_RobotBall.push_back(kf);
    KF_RobotBall.push_back(kf);
    KF_RobotBall.push_back(kf);
    KF_RobotBall.push_back(kf);
    KF_RobotBall.push_back(kf);
  }

  void set_ROI(cv::Point kf_ball_point, vector <cv::Point> kf_robot_point){
    KF_Robot_point.clear();
    KF_Ball_point = kf_ball_point;
    for (int i = 0; i < kf_robot_point.size(); i++)
    {
      KF_Robot_point.push_back(kf_robot_point[i]);
    }
  }

  bool isAnyRobotLost()
  {
    //std::cout << Ball_lost << ", "  << robot_lost[0] << ", " << robot_lost[1] << ", " << robot_lost[2] << std::endl;
    return (robot_lost[0] || robot_lost[1] || robot_lost[2] || robot_lost[3] || robot_lost[4] || robot_lost[5]);
  }

  bool isBallLost(){

    return Ball_lost;
  }
  int get_robot_list_size()
  {
    return robot_list.size();
  }

  void setWindowSize(cv::Point KF, bool isRobot, int index)
  {

    cv::Point aux;

    //Determinar o x do ponto 1
    if (KF.x-HWS<=0)
      aux.x = 0;
    else if (KF.x-HWS>=width)
      aux.x = width;
    else
      aux.x = KF.x-HWS;

    //Determinar o y do ponto 1
    if (KF.y-HWS<=0)
      aux.y = 0;
    else if (KF.y-HWS>=height)
      aux.y = height;
    else
      aux.y = KF.y-HWS;

    // Definir o ponto 1
    if (isRobot)
      robot_p1[index] = cv::Point(aux.x, aux.y);
    else //isBall
      ball_p1 = cv::Point(aux.x, aux.y);

    //Determinar o x do ponto 2
    if (KF.x+HWS<=0)
      aux.x = 0;
    else if (KF.x+HWS>=width)
      aux.x = width;
    else
      aux.x = KF.x+HWS;

    //Determinar o y do ponto 2
    if (KF.y+HWS<=0)
      aux.y = 0;
    else if (KF.y+HWS>=height)
      aux.y = height;
    else
      aux.y = KF.y+HWS;

    //Definir Ponto 2
    if (isRobot)
      robot_p2[index] = cv::Point(aux.x, aux.y);
    else //isBall
      ball_p2 = cv::Point(aux.x, aux.y);
  }

  // Verifica se a janela é realmente uma janela e não uma linha
  bool checkWindowSize(cv::Point p1, cv::Point p2)
  {
    if (abs(p1.x - p2.x) > 0 && abs(p1.y - p2.y) > 0)
      return true;
    else
      return false;
  }

  Robot get_robot_from_list(int index)
  {
    Robot r;
    if (index < robot_list.size() && index >= 0)
    return robot_list[index];
    else
    {
      std::cout << "COULD NOT GET ROBOT "<<index<<" FROM LIST (VISION). QUER ME FUDER??!" << std::endl;
      return r;
    }
  }

  cv::Point get_ball_position()
  {
    return Ball;
  }

  void setCalibParams(int H[5][2], int S[5][2], int V[5][2], int Amin[5], int E[5], int D[5], int B[5])
  {
    for (int i = 0; i < 5; i++)
    {
      areaMin[i] = Amin[i];
      erosion[i] = E[i];
      delation[i] = D[i];
      blur[i] = B[i];
      for (int j = 0; j < 2; j++)
      {
        hue[i][j] = H[i][j];
        saturation[i][j] = S[i][j];
        value[i][j] = V[i][j];
      }
    }
  }

  void windowed_parallel_tracking(cv::Mat im) {

    // Janelas
    cv::Mat dummy[10];
    cv::Mat crop[10];


    cv::Mat image_copy = im.clone();

    // robos do nosso time
    for (int i = 0; i < 3; i++)
    {
      setWindowSize(KF_Robot_point[i], true, i);
      if (!checkWindowSize(robot_p1[i], robot_p2[i]))
      {
        setRobotLost(i);
        return;
      }

      robotOrigin[i] = robot_p1[i];
      cv::Rect  rect(robot_p1[i],robot_p2[i]);

      dummy[2*i] = image_copy(rect);
      crop[2*i] = dummy[2*i].clone();
      dummy[1+2*i] = image_copy(rect);
      crop[1+2*i] = dummy[1+2*i].clone();

    }

    // robos adversarios
    for (int i = 3; i < 6; i++)
    {
      setWindowSize(KF_Robot_point[i], true, i);
      if (!checkWindowSize(robot_p1[i], robot_p2[i]))
      {
        robot_lost[i] = true;
        return;
      }

      robotOrigin[i] = robot_p1[i];
      cv::Rect  rect(robot_p1[i],robot_p2[i]);

      dummy[i+3] = image_copy(rect);
      crop[i+3] = dummy[i+3].clone();
    }

    // bola
    setWindowSize(KF_Ball_point, false, 0);

    if (!checkWindowSize(ball_p1, ball_p2))
    {
      Ball_lost = true;
      return;
    }

    Ballorigin = ball_p1;

    cv::Rect  rect(ball_p1,ball_p2);
    dummy[9] = image_copy(rect);
    crop[9] = dummy[9].clone();


    // thresholds
    // Robo 1 - amarelo e verde
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[0]), 0, 0, robot_p1[0], robot_p2[0]));
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[1]), 1, 0, robot_p1[0], robot_p2[0]));

    // Robo 2 - amarelo e verde
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[2]), 0, 1, robot_p1[1], robot_p2[1]));
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[3]), 1, 1, robot_p1[1], robot_p2[1]));

    // Robo 3 - amarelo e rosa
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[4]), 0, 2, robot_p1[2], robot_p2[2]));
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[5]), 2, 2, robot_p1[2], robot_p2[2]));

    // Adversario 1 - azul
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[6]), 4, 3, robot_p1[3], robot_p2[3]));

    // Adversario 2 - azul
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[7]), 4, 4, robot_p1[4], robot_p2[4]));

    // Adversario 3 - azul
    threshold_threadsWindow.add_thread(new boost::thread(&Vision::windowed_img_tracking,this, boost::ref(crop[8]), 4, 5, robot_p1[5], robot_p2[5]));

    windowed_img_tracking(crop[9], 3, 6, ball_p1, ball_p2);

    //Tracking Adversário
    //threshold_threadsWindow.add_thread(new boost::thread(&Vision::img_tracking,this, boost::ref(image_copy), 5));

    threshold_threadsWindow.join_all();

    image_copy.release();
    for (int i = 0; i < 10; i++)
    {
      crop[i].release();
      dummy[i].release();
    }

  }

  void parallel_tracking(cv::Mat im) {
    cv::Mat image_copy = im.clone();
    cv::cvtColor(image_copy,image_copy,cv::COLOR_RGB2HSV);
    Ballorigin = cv::Point(0,0);

    for(int i =0; i<5; i++) {

      threshold_threads.add_thread(new boost::thread(&Vision::img_tracking,this, boost::ref(image_copy), (i)));
    }
    threshold_threads.join_all();
    image_copy.release();
  }

  void setRobotLost(int index)
  {
    TeamMainWindow[index].clear();
    TeamSecWindow[index][0].clear();
    TeamSecWindow[index][1].clear();
    TeamSecWindowArea[index][0].clear();
    TeamSecWindowArea[index][1].clear();
    robot_lost[index] = true;
  }



  void windowed_img_tracking(cv::Mat image,int color_id, int window_id, cv::Point p1,cv::Point p2) {

    int ec,e3c,H,S,V;
    vector< vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;

    cv::cvtColor(image,image,cv::COLOR_RGB2HSV);

    cv::Mat dummy;
    cv::Mat crop;

    for(int i =0; i<image.cols; i++) { //Threshold calculations
      for(int j =0; j<image.rows; j++) {
        ec = image.cols*i + j;
        e3c = ec*3;

        H = image.data[e3c];
        S = image.data[e3c + 1];
        V = image.data[e3c + 2];

        ec = width*(i+p1.y) + (j+p1.x);
        e3c = ec*3;
        if((H>=hue[color_id][0]&&H<=hue[color_id][1])&&
        (S>=saturation[color_id][0]&&S<=saturation[color_id][1])&&
        (V>=value[color_id][0]&&V<=value[color_id][1])) {

          threshold[color_id][e3c] = 255;
          threshold[color_id][e3c+1] = 255;
          threshold[color_id][e3c+2] = 255;
        } else {
          threshold[color_id][e3c] = 0;
          threshold[color_id][e3c+1] = 0;
          threshold[color_id][e3c+2] = 0;
        }
      }
    }
  //  cout << window_id << " - 1.6.3" << endl;
    cv::Rect  rect(p1,p2);

    //kernel
    cv::Mat erodeElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(3,3));
    cv::Mat dilateElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(3,3));

    cv::Mat temp(height,width,CV_8UC3,threshold[color_id]);
    dummy = temp(rect);
    crop = dummy.clone();
    cv::erode(crop,crop,erodeElement,cv::Point(-1,-1),erosion[color_id]);
    cv::dilate(crop,crop,dilateElement,cv::Point(-1,-1),delation[color_id]);
    //std::cout<<"====Windoz vision Blur: "<<blur[color_id]<<" id color: "<<color_id<<std::endl;
    cv::medianBlur(crop, crop, blur[color_id]);
    //cv::medianBlur(crop, crop, 5);
    cv::cvtColor(crop,crop,cv::COLOR_RGB2GRAY);
    cv::findContours(crop,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE);

    switch(color_id) {

      case 0:// TEAM MAIN COLOR

      if (hierarchy.size() > 0) {
        TeamMainWindow[window_id].clear();
        int index = 0;
        while(index >= 0) {

          cv::Moments moment = moments((cv::Mat)contours[index]);
          double area = contourArea(contours[index]);
          //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
          if(area >= areaMin[color_id]/100) {
            TeamMainWindow[window_id].push_back(cv::Point(moment.m10/area,moment.m01/area));

          }
          else
          {
            setRobotLost(window_id);
          }
          index = hierarchy[index][0];
        }
      }

      break;


      case 1:// TEAM FIRST SECUNDARY COLOR
      if (hierarchy.size() > 0) {
        TeamSecWindowArea[window_id][0].clear();
        TeamSecWindow[window_id][0].clear();
        int index = 0;
        while(index >= 0) {
          cv::Moments moment = moments((cv::Mat)contours[index]);
          double area = contourArea(contours[index]);
          //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
          if(area >= areaMin[color_id]/100) {
            TeamSecWindow[window_id][0].push_back(cv::Point(moment.m10/area,moment.m01/area));
            //cout << "Janela [" << window_id << "] " << "VERDE: " <<  TeamSecWindow[window_id][0].size() << endl;
            TeamSecWindowArea[window_id][0].push_back(area);
          }
          index = hierarchy[index][0];
        }
      }


      break;

      case 2:// TEAM SECOND SECUNDARY COLOR
      if (hierarchy.size() > 0) {
        TeamSecWindowArea[window_id][1].clear();
        TeamSecWindow[window_id][1].clear();
        int index = 0;
        while(index >= 0) {
          cv::Moments moment = moments((cv::Mat)contours[index]);
          double area = contourArea(contours[index]);
          //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
          if(area >= areaMin[color_id]/100) {
            TeamSecWindow[window_id][1].push_back(cv::Point(moment.m10/area,moment.m01/area));
            //cout << "Janela [" << window_id << "] " << "ROSA: " <<  TeamSecWindow[window_id][1].size() << endl;
            TeamSecWindowArea[window_id][1].push_back(area);

          }
          index = hierarchy[index][0];
        }
      }

      break;

      case 3:// BALL
      if (hierarchy.size() > 0) {
        cv::Moments moment = moments((cv::Mat)contours[0]);
        double area = contourArea(contours[0]);
        //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
        //cout<<"windowed"<<endl;

        if(area >= areaMin[color_id]/100) {
          Ball = cv::Point(moment.m10/area,moment.m01/area)+Ballorigin;
          Ball_lost = false;

        }else{
          Ball_lost = true;
        }
      }else{
        Ball_lost = true;
        //std::cout<<"BALL LOST"<<std::endl;

      }
      break;

      case 4:// ADVERSARY MAIN COLOR

      if (hierarchy.size() > 0) {
      Adv_Main[window_id-3].clear();
      int index = 0;
      while(index >= 0) {
      cv::Moments moment = moments((cv::Mat)contours[index]);
      double area = contourArea(contours[index]);
      //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
      if(area >= areaMin[color_id]/100) {
      Adv_Main[window_id-3].push_back(cv::Point(moment.m10/area,moment.m01/area));

      }
    index = hierarchy[index][0];
  }
  robot_list[window_id].position = getCorrectAdversary(window_id)+robotOrigin[window_id];
  //std::cout << "Window Adv " << window_id-3 << ": " << robot_list[window_id-3].position.x << ", " << robot_list[window_id-3].position.y << std::endl;



} else {
  robot_lost[3] = true;
  robot_lost[4] = true;
  robot_lost[5] = true;
}

break;

}
//cout << window_id << " - 1.6.6" << endl;
}

void img_tracking(cv::Mat image,int color_id) {
  int ec,e3c,H,S,V;
  vector< vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;


  for(int i =0; i<image.rows; i++) { //Threshold calculations
    for(int j =0; j<image.cols; j++) {
      ec = image.cols*i + j;
      e3c = ec*3;

      H = image.data[e3c];
      S = image.data[e3c + 1];
      V = image.data[e3c + 2];

      if((H>=hue[color_id][0]&&H<=hue[color_id][1])&&
      (S>=saturation[color_id][0]&&S<=saturation[color_id][1])&&
      (V>=value[color_id][0]&&V<=value[color_id][1])) {

        threshold[color_id][e3c] = 255;
        threshold[color_id][e3c+1] = 255;
        threshold[color_id][e3c+2] = 255;
      } else {
        threshold[color_id][e3c] = 0;
        threshold[color_id][e3c+1] = 0;
        threshold[color_id][e3c+2] = 0;
      }
    }
  }

  //kernel
  cv::Mat erodeElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(3,3));
  cv::Mat dilateElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(3,3));

  cv::Mat temp(height,width,CV_8UC3,threshold[color_id]);
  cv::erode(temp,temp,erodeElement,cv::Point(-1,-1),erosion[color_id]);
  cv::dilate(temp,temp,dilateElement,cv::Point(-1,-1),delation[color_id]);
  //std::cout<<"====Vision Blur: "<<blur[color_id]<<" id color: "<<color_id<<std::endl;
  cv::medianBlur(temp, temp, blur[color_id]);
  cv::cvtColor(temp,temp,cv::COLOR_RGB2GRAY);
  cv::findContours(temp,contours,hierarchy,cv::RETR_CCOMP,cv::CHAIN_APPROX_SIMPLE);


  switch(color_id) {

    case 0:// TEAM MAIN COLOR

    if (hierarchy.size() > 0) {
      Team_Main.clear();
      int index = 0;
      while(index >= 0) {
        cv::Moments moment = moments((cv::Mat)contours[index]);
        double area = contourArea(contours[index]);
        //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
        if(area >= areaMin[color_id]/100) {
          Team_Main.push_back(cv::Point(moment.m10/area,moment.m01/area));
        }
        index = hierarchy[index][0];
      }

    }

    break;


    case 1:// TEAM FIRST SECUNDARY COLOR
    if (hierarchy.size() > 0) {
      Team_Sec_area[0].clear();
      Team_Sec[0].clear();
      int index = 0;
      while(index >= 0) {
        cv::Moments moment = moments((cv::Mat)contours[index]);
        double area = contourArea(contours[index]);
        //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
        if(area >= areaMin[color_id]/100) {
          Team_Sec[0].push_back(cv::Point(moment.m10/area,moment.m01/area));
          Team_Sec_area[0].push_back(area);
        }
        index = hierarchy[index][0];
      }
    }


    break;

    case 2:// TEAM SECOND SECUNDARY COLOR
    if (hierarchy.size() > 0) {
      Team_Sec_area[1].clear();
      Team_Sec[1].clear();
      int index = 0;
      while(index >= 0) {
        cv::Moments moment = moments((cv::Mat)contours[index]);
        double area = contourArea(contours[index]);
        //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
        if(area >= areaMin[color_id]/100) {
          Team_Sec[1].push_back(cv::Point(moment.m10/area,moment.m01/area));
          Team_Sec_area[1].push_back(area);

        }
        index = hierarchy[index][0];
      }
    }

    break;


    case 4:// ADVERSARY MAIN COLOR

    if (hierarchy.size() > 0) {
      Adv_Main[0].clear();
      Adv_Main[1].clear();
      Adv_Main[2].clear();
      int advCounter = 0;
      int index = 0;
      while(index >= 0 && advCounter < 3) {
        cv::Moments moment = moments((cv::Mat)contours[index]);
        double area = contourArea(contours[index]);
        //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
        if(area >= areaMin[color_id]/100) {
          Adv_Main[advCounter].push_back(cv::Point(moment.m10/area,moment.m01/area));
          robot_list[advCounter+3].position = cv::Point(moment.m10/area,moment.m01/area);
          robot_lost[advCounter+3] = false;
          //std::cout << "Old Adv " <<  advCounter << ": " << robot_list[advCounter+3].position.x << ", " << robot_list[advCounter+3].position.y << std::endl;
          advCounter++;
        }
        index = hierarchy[index][0];
      }

    } else {
      robot_lost[3] = true;
      robot_lost[4] = true;
      robot_lost[5] = true;
    }

    break;

    case 3:// BALL
    if (hierarchy.size() > 0) {
      cv::Moments moment = moments((cv::Mat)contours[0]);
      double area = contourArea(contours[0]);
      //Se a área do objeto for muito pequena então provavelmente deve ser apenas ruído.
      //  cout<<"img_tracking"<<endl;

      if(area >= areaMin[color_id]/100) {
        Ball = cv::Point(moment.m10/area,moment.m01/area);
        Ball_lost = false;

      }
      else {
        Ball_lost = true;
      }
    }else{
      Ball_lost = true;
      //std::cout<<"BALL LOST"<<std::endl;

    }
    break;

  }
}

void windowed_robot_creation_uni_duni_tag(int window_id) {
  Robot robot;

  double omax = -99999; // angulo maximo
  double omin = 99999; // angulo minimo
  cv::Point secundary;
  int index1[2] = {0,0}; // index do robo com img_tracking
  int index2[2] = {0,0}; // index do robo com img_tracking (usado para trocar o index)
  float sx,sy,px,py,tx,ty; // posilções x e y das tags primária, secundária e ternária
  float distanceRef1 = 999999999.0;
  float distanceRef2 = 999999999.0;
  float distance = 0;
  int main_tag;
  double o = 0;

  // Verificar quantas tags amarelas tem dentro da janela
  // Verifica se a tag principal encontrada pertence a janela certa
  if (TeamMainWindow[window_id].size() <= 0)
  {
    setRobotLost(window_id);
    return;
  }
  else if (TeamMainWindow[window_id].size() == 1)
  main_tag = 0;
  else
  main_tag = getCorrectMainTag(window_id);

  //std::cout << "5.1.2" << std::endl;
  distanceRef1 = 999999999.0;
  distanceRef2 = 999999999.0;

  //Se não tiver tag secundária dentro da janela -> abortar missão
  if (TeamSecWindow[window_id][0].size() == 0 && TeamSecWindow[window_id][1].size() == 0)
  {
    setRobotLost(window_id);
    return;
  }

  // Verifica quais tags secundárias dentro da janela estão mais próximas da tag principal correta
  for(int i = 0; i < 2; i++) {
    for(int k = 0; k < TeamSecWindow[window_id][i].size(); k++) {

      distance = calcDistance(TeamMainWindow[window_id][main_tag],TeamSecWindow[window_id][i][k]);

      if(distance <= distanceRef1) {
        distanceRef2 = distanceRef1;
        index2[0] = index1[0];
        index2[1] = index1[1];

        distanceRef1 = distance;
        index1[0] = i;
        index1[1] = k;
        if(i==1)
        robot.pink=true;

      } else if(distance < distanceRef2) {
        distanceRef2 = distance;
        index2[0] = i;
        index2[1] = k;
      }
    }

  }

  // Determina a posição do robô na imagem (inteira)
  robot.position = TeamMainWindow[window_id][main_tag]+robotOrigin[window_id];


  // Verifica, pelo tamanho das cores secunárias, qual é a tag secunária (frente o robô) e qual é a ternária (diferenciar robôs de mesma cor)
  if (TeamSecWindowArea[window_id][index1[0]].size() > index1[1] && TeamSecWindowArea[window_id][index2[0]].size() > index2[1])
  {
    if(TeamSecWindowArea[window_id][index1[0]][index1[1]]>TeamSecWindowArea[window_id][index2[0]][index2[1]]) {
      robot.secundary = TeamSecWindow[window_id][index1[0]][index1[1]]+robotOrigin[window_id];
      robot.ternary =  TeamSecWindow[window_id][index2[0]][index2[1]]+robotOrigin[window_id];

    } else {
      robot.secundary = TeamSecWindow[window_id][index2[0]][index2[1]]+robotOrigin[window_id];
      robot.ternary = TeamSecWindow[window_id][index1[0]][index1[1]]+robotOrigin[window_id];
    }
  }
  else
  {
    // ainda não foi calibrado, não precisa achar os robôs.
    setRobotLost(window_id);
    return;
  }



  sx = robot.secundary.x;
  sy =  robot.secundary.y;

  px = robot.position.x;
  py = robot.position.y;

  tx = robot.ternary.x;
  ty =  robot.ternary.y;

  // Cálculo da orientação das tags secundária (orientation) e ternária (orientation2)
  robot.orientation = atan2((sy-py)*1.3/480,(sx-px)*1.5/640);
  robot.orientation2 = atan2((ty-py)*1.3/480,(tx-px)*1.5/640);

  o = atan2(sin(robot.orientation2-robot.orientation+3.1415),
    cos(robot.orientation2-robot.orientation+3.1415));


  // Dita o ID dos robôs
  if(robot.pink){
    if (window_id != 2)
    { // A janela identificou o robô errado
      setRobotLost(window_id);
    }
    else
    {
      robot_list[2].position = robot.position; // colocar em um vetor
      robot_list[2].secundary = robot.secundary; // colocar em um vetor
      robot_list[2].orientation =  robot.orientation;
      robot_lost[window_id] = false;
    }
    return;

  }else  if(o>0) {
    if (window_id != 0)
    { // A janela identificou o robô errado
      setRobotLost(window_id);
    }
    else
    {
      robot_list[0].position = robot.position; // colocar em um vetor
      robot_list[0].secundary = robot.secundary; // colocar em um vetor
      robot_list[0].orientation =  robot.orientation;
      robot_lost[window_id] = false;
    }
    return;

  }else {
    if (window_id != 1)
    { // A janela identificou o robô errado
      setRobotLost(window_id);
    }
    else
    {
      robot_list[1].position = robot.position; // colocar em um vetor
      robot_list[1].secundary = robot.secundary; // colocar em um vetor
      robot_list[1].orientation =  robot.orientation;
      robot_lost[window_id] = false;
    }
    return;

  }

  // Se chegou até aqui, nenhum robô foi identificado dentro da janela
  setRobotLost(window_id);

}

// Verifica qual tag principal pertence a janela
// Pela menor distância da tag certa em relação ao frame anterior
int getCorrectMainTag(int window_id)
{
  float minDistance = 99999.0;
  float distance;
  int correctTag;

  for (int i = 0; i < TeamMainWindow[window_id].size(); i++)
  {
    // Compara a distancia entre a tag do frame atual com a tag do frame anterior
    distance = calcDistance(TeamMainWindow[window_id][i]+robotOrigin[window_id],robot_list[window_id].position);
    if (distance <= minDistance)
    {
      minDistance = distance;
      correctTag = i;
    }
  }
  return correctTag;
}

// Verifica qual adversario pertence a janela
// Pela menor distância da tag certa em relação ao frame anterior
cv::Point getCorrectAdversary(int window_id)
{
  float minDistance = 99999.0;
  float distance;
  int correctTag;

  for (int i = 0; i < Adv_Main[window_id-3].size(); i++)
  {
    // Compara a distancia entre a tag do frame atual com a tag do frame anterior
    distance = calcDistance(Adv_Main[window_id-3][i]+robotOrigin[window_id],robot_list[window_id].position);
    if (distance <= minDistance)
    {
      minDistance = distance;
      correctTag = i;
    }
  }
  return Adv_Main[window_id-3][correctTag];
}

void robot_creation_uni_duni_tag() {
  vector <Robot> robot;
  Robot r;
  double omax = -99999; // angulo maximo
  double omin = 99999; // angulo minimo
  cv::Point secundary;
  int index1[2] = {0,0}; // index do robo com img_tracking
  int index2[2] = {0,0}; // index do robo com img_tracking (usado para trocar o index)
  float sx,sy,px,py,tx,ty; // posilções x e y das tags primária, secundária e ternária
  int l=0; // índice do robo
  float distanceRef1 = 999999999.0;
  float distanceRef2 = 999999999.0;
  float distance = 0;
  double o = 0;

  for(int j = 0; j < Team_Main.size()&&j<=3; j++) {
    robot.push_back(r);
    distanceRef1 = 999999999.0;
    distanceRef2 = 999999999.0;

    // Para cada tag principal, verifica quais são as secundárias correspondentes
    for(int i = 0; i < 2; i++) {
      for(int k = 0; k < Team_Sec[i].size(); k++) {

        distance = calcDistance(Team_Main[j],Team_Sec[i][k]);

        if(distance <= distanceRef1) {
          distanceRef2 = distanceRef1;
          index2[0] = index1[0];
          index2[1] = index1[1];

          distanceRef1 = distance;
          index1[0] = i;
          index1[1] = k;
          if(i==1)
          robot[l].pink=true;

        } else if(distance < distanceRef2) {
          distanceRef2 = distance;
          index2[0] = i;
          index2[1] = k;
        }
      }

    }

    // Determina a posição do robô
    robot[l].position = Team_Main[j];

  // Verifica qual das tags é a secundária (frente do robô) e qual é a ternária (diferencia os robôs de mesma cor)
    if (Team_Sec_area[index1[0]].size() > index1[1] && Team_Sec_area[index2[0]].size() > index2[1])
    {
      if(Team_Sec_area[index1[0]][index1[1]]>Team_Sec_area[index2[0]][index2[1]]) {
        robot[l].secundary = Team_Sec[index1[0]][index1[1]];
        robot[l].ternary =  Team_Sec[index2[0]][index2[1]];
      } else {
        robot[l].secundary = Team_Sec[index2[0]][index2[1]];
        robot[l].ternary = Team_Sec[index1[0]][index1[1]];
      }
    }
    else
    {
      // ainda não foi calibrado, não precisa achar os robôs.
      return;
    }



    sx = robot[l].secundary.x;
    sy =  robot[l].secundary.y;

    px = robot[l].position.x;
    py = robot[l].position.y;

    tx = robot[l].ternary.x;
    ty =  robot[l].ternary.y;

    // Cálculo da orientação das tags secundária e ternária em relação a tag principal
    robot[l].orientation = atan2((sy-py)*1.3/480,(sx-px)*1.5/640);
    robot[l].orientation2 = atan2((ty-py)*1.3/480,(tx-px)*1.5/640);

    // Cálculo do ângulo de orientação para diferenciar robôs de mesma cor
    o = atan2(sin(robot[l].orientation2-robot[l].orientation+3.1415),
      cos(robot[l].orientation2-robot[l].orientation+3.1415));

    // Dá nome aos bois (robôs)
    if(robot[l].pink){
      robot_list[2].position = robot[l].position; // colocar em um vetor
      robot_list[2].secundary = robot[l].secundary; // colocar em um vetor
      robot_list[2].orientation =  robot[l].orientation;
      robot_lost[2] = false;

    }else  if(o>0) {

      robot_list[0].position = robot[l].position; // colocar em um vetor
      robot_list[0].secundary = robot[l].secundary; // colocar em um vetor
      robot_list[0].orientation =  robot[l].orientation;
      robot_lost[0] = false;

    }else {

      robot_list[1].position = robot[l].position; // colocar em um vetor
      robot_list[1].secundary = robot[l].secundary; // colocar em um vetor
      robot_list[1].orientation =  robot[l].orientation;
      robot_lost[1] = false;

    }
    // repetir para cada tag principal (que são 3)
    l==3? : l++;

  }
}

void calcOrientation(int tag_id) { //Define a orientação da tag em analise;
  float sx,sy,px,py;

  sx =  robot_list[tag_id].secundary.x;
  sy =  robot_list[tag_id].secundary.y;

  px = robot_list[tag_id].position.x;
  py = robot_list[tag_id].position.y;

  robot_list[tag_id].orientation = atan2((sy-py)*1.3/480,(sx-px)*1.5/640);
  robot_list[tag_id].position.x = robot_list[tag_id].position.x;
  robot_list[tag_id].position.y = robot_list[tag_id].position.y;
}

float calcDistance(cv::Point position, cv::Point secundary) {
  return sqrt(pow(position.x-secundary.x,2) + pow(position.y-secundary.y,2));
}


Vision(int w, int h)
{
  vector< double > a;

  Ball_lost = true;
  robot_lost[0] = true;
  robot_lost[1] = true;
  robot_lost[2] = true;
  robot_lost[3] = true;
  robot_lost[4] = true;
  robot_lost[5] = true;

  Ballorigin = cv::Point(0,0);
  Team_Sec_area.push_back(a);
  Team_Sec_area.push_back(a);
  Team_Sec_area.push_back(a);

  width = w;
  height = h;

  vector< cv::Point > p;
  vector<vector < cv::Point >> q;
  vector<vector < double >> d;
  vector< double > f;
  Robot r;
  p.push_back(cv::Point(0,0));
  robot_list.push_back(r);
  robot_list.push_back(r);
  robot_list.push_back(r);
  robot_list.push_back(r);
  robot_list.push_back(r);
  robot_list.push_back(r);

  Adv_Main.push_back(p);
  Adv_Main.push_back(p);
  Adv_Main.push_back(p);

  Team_Sec.push_back(p);
  Team_Sec.push_back(p);
  Team_Sec.push_back(p);

  Team_Main.push_back(cv::Point(0,0));
  Team_Main.push_back(cv::Point(0,0));
  Team_Main.push_back(cv::Point(0,0));

  TeamMainWindow.push_back(p);
  TeamMainWindow.push_back(p);
  TeamMainWindow.push_back(p);

  TeamSecWindow.push_back(q);
  TeamSecWindow.push_back(q);
  TeamSecWindow.push_back(q);

  TeamSecWindow[0].push_back(p);
  TeamSecWindow[1].push_back(p);
  TeamSecWindow[2].push_back(p);
  TeamSecWindow[0].push_back(p);
  TeamSecWindow[1].push_back(p);
  TeamSecWindow[2].push_back(p);

  TeamSecWindowArea.push_back(d);
  TeamSecWindowArea.push_back(d);
  TeamSecWindowArea.push_back(d);

  TeamSecWindowArea[0].push_back(f);
  TeamSecWindowArea[1].push_back(f);
  TeamSecWindowArea[2].push_back(f);
  TeamSecWindowArea[0].push_back(f);
  TeamSecWindowArea[1].push_back(f);
  TeamSecWindowArea[2].push_back(f);


  KalmanFilter kf;
  KF_RobotBall.push_back(kf);
  KF_RobotBall.push_back(kf);
  KF_RobotBall.push_back(kf);
  KF_RobotBall.push_back(kf);
  KF_RobotBall.push_back(kf);
  KF_RobotBall.push_back(kf);
  KF_RobotBall.push_back(kf);

  threshold = (unsigned char**) malloc(5 * sizeof(unsigned char *));
  for(int i = 0; i < 5; i++)
  {
    threshold[i] =  (unsigned char*) malloc((3*(width*height + width) +3) * sizeof(unsigned char));
    if(threshold[i] == NULL)
    {
      cout<<"out of memory\n"<<endl;
    }
  }
}


~Vision()
{
  if (threshold != NULL)
  {
    for(int i = 0; i < 5; i++)
    free(threshold[i]);
    free(threshold);
  }
}



};
#endif /* VISION_HPP_ */
