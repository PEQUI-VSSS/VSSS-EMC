#include <gtkmm.h>
#include "CamCap.hpp"
#include "ros/ros.h"
#include "vsss_msgs/Control.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "vsss");

	#ifdef CUDA_FOUND
		std::cout << "CUDA is enabled." << std::endl;
	#else
		std::cout << "CUDA not found." << std::endl;
	#endif

	Gtk::Main kit(argc, argv);
	Gtk::Window window;

	window.set_title("Pequi Mecânico - VSSS EMC");

	auto screen_width = Gdk::screen_width();
	bool isLowRes = screen_width <= 1600;

	CamCap camcap(isLowRes);
	window.add(camcap);

	window.show_all();

	camcap.interface.visionGUI.hideGMM();
	camcap.interface.fr_camCalib_offline.hide();
	camcap.interface.controlGUI.update_ack_interface();

	window.maximize();

	Gtk::Main::run(window);

	return EXIT_SUCCESS;
}
