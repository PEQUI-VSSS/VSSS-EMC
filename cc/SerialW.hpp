#ifndef SERIALW_HPP_
#define SERIALW_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include "../pack-capture-gui/capture-gui/Robot.hpp"

#define POSITION 0
#define SPEED 1
#define ORIENTATION 2
#define PI 3.14159265453

class SerialW
{
public:
	bool Serial_Enabled;
	int	USB;
	std::string port;
SerialW()
	{
		Serial_Enabled =false;
	}

int start(std::string serial){
	struct termios tty;
	struct termios tty_old;
	memset (&tty, 0, sizeof tty);

	USB = open(serial.c_str(), O_WRONLY| O_NOCTTY);
    if(USB != -1)
    {
        Serial_Enabled=true;
    }else{

		Serial_Enabled=false;
		return USB;
	}



/* Error Handling */
if ( tcgetattr ( USB, &tty ) != 0 ) {
   std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
}

/* Save old tty parameters */
tty_old = tty;

/* Set Baud Rate */
cfsetospeed (&tty, (speed_t)B115200);


tty.c_oflag = 0;
tty.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
tty.c_cflag &= ~(CSIZE | PARENB);
tty.c_cflag |= CS8;

/* Setting other Port Stuff */
//tty.c_cflag     &=  ~PARENB;            // Make 8n1
//tty.c_cflag     &=  ~CSTOPB;
//tty.c_cflag     &=  ~CSIZE;
//tty.c_cflag     |=  CS8;

//tty.c_cflag     &=  ~CRTSCTS;           // no flow control
//tty.c_cc[VMIN]   =  1;                  // read doesn't block
//tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
//tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

/* Make raw */
cfmakeraw(&tty);

/* Flush Port, then applies attributes */
tcflush( USB, TCIFLUSH );

if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
}

return USB;
	}


void sendToRobot(Robot r){
	stringstream cmd;
	double temp0= floor(r.Vr*100)/100;
	double temp1= floor(r.Vl*100)/100;
	cmd<<r.ID<< temp0<<";"<<temp1<<"#";
	sendSerial(cmd.str());
	//std::cout<<cmd.str()<<std::endl;
	}

void sendCmdToRobots(std::vector<Robot> robot_list){
	stringstream cmd;
	double temp0, temp1, temp2;
	for (int i = 0; i < 3; i++){
		switch (robot_list[i].cmdType){
			case POSITION:
			temp0= round(double(robot_list[i].transTarget.x)*(150.0/640.0)*100)/100;
			temp1= round(double(robot_list[i].transTarget.y)*(130.0/480.0)*100)/100;
			temp2= round(double(robot_list[i].vmax)*100)/100;
			cmd<<robot_list[i].ID<<"P"<< temp0<<";"<<temp1<<";"<<temp2<<"#"<< endl;
			break;
			case SPEED:
			temp0= round(robot_list[i].Vr*100)/100;
			temp1= round(robot_list[i].Vl*100)/100;
			cmd<<robot_list[i].ID<< temp0<<";"<<temp1<<"#" << endl;
			break;
			case ORIENTATION:
			temp0= robot_list[i].transOrientation*180/PI;
			temp1= round(double(robot_list[i].vmax)*100)/100;
			cmd << robot_list[i].ID << "O" << temp0 << ";" << temp1 << "#" << endl;
			cout << temp0 << endl;
			break;
			default:
			temp0= round(double(robot_list[i].transTarget.x)*(150.0/640.0)*100)/100;
			temp1= round(double(robot_list[i].transTarget.y)*(130.0/480.0)*100)/100;
			temp2= round(double(robot_list[i].vmax)*100)/100;
			cmd<<robot_list[i].ID<<"P"<< temp0<<";"<<temp1<<";"<<temp2<<"#" << endl;
		}
	}
	sendSerial(cmd.str());
	//std::cout<<cmd.str()<<std::endl;
}
void sendSerial(std::string cmd){

int n_written = write( USB, cmd.c_str(),cmd.size());

	}

std::string readSerial(){

	int n = 0,
    spot = 0;
	char buf = '\0';

	/* Whole response*/
	char response[1024];
	memset(response, '\0', sizeof response);

do {
    n = read( USB, &buf, 1 );
    sprintf( &response[spot], "%c", buf );
    spot += n;
} while( buf != '\r' && n > 0);

if (n < 0) {
    std::cout << "Error reading: " << strerror(errno) << std::endl;
}
else if (n == 0) {
    std::cout << "Read nothing!" << std::endl;
}
else {
    std::cout << "Response: " << response << std::endl;
}

	return std::string(response);

	}
};
#endif /* CONTROLGUI_HPP_ */
