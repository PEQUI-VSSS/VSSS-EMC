#include <gtkmm.h>
#include <thread>
#include "CamCap.hpp"

int main(int argc, char **argv) {

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

	camcap.interface.fr_camCalib_offline.hide();
	camcap.interface.controlGUI.update_ack_interface();

	window.maximize();


	auto handler = std::thread([&]{
		auto& xbee = camcap.interface.controlGUI.messenger.xbee;
		while (true) {
			if (xbee != nullptr) xbee->save_logs();
		}
	});

	Gtk::Main::run(window);

	return EXIT_SUCCESS;
}
