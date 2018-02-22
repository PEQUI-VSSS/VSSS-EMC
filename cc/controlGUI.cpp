#include "controlGUI.hpp"

bool ControlGUI::get_PID_test_flag() {
    return PID_test_flag;
}

void ControlGUI::set_PID_test_flag(bool input) {
    PID_test_flag = input;
}

ControlGUI::ControlGUI() {
    // Adicionar o frame do Serial e sua VBOX
    pack_start(Top_hbox, false, true, 5);
    Top_hbox.pack_start(Serial_fm, false, true, 5);
    Serial_fm.set_label("Serial");
    Serial_fm.add(Serial_vbox);

//    Top_hbox.pack_start(Test_fm, false, true, 5);

//    Test_fm.set_label("Test");
//    Test_fm.add(Test_vbox);
    button_PID_Test.set_label("PID Test on Click");
//    Test_vbox.pack_start(button_PID_Test, false, true, 5);

    Serial_hbox[0].pack_start(cb_serial, false, true, 5);
    Serial_hbox[0].pack_start(bt_Serial_Start, false, true, 5);
    Serial_hbox[0].pack_start(bt_Serial_Refresh, false, true, 5);
    Serial_hbox[0].pack_end(button_PID_Test, false, true, 5);
    Serial_vbox.pack_start(Serial_hbox[0], false, true, 5);

    Serial_hbox[1].pack_start(cb_test, false, true, 5);
    Serial_hbox[1].pack_start(Tbox_V1, false, true, 5);
    Serial_hbox[1].pack_start(Tbox_V2, false, true, 5);
    Serial_hbox[1].pack_start(bt_Serial_test, false, true, 5);
    Serial_vbox.pack_start(Serial_hbox[1], false, true, 5);

    Serial_hbox[2].pack_start(send_cmd_box, false, true, 5);
    Serial_hbox[2].pack_start(bt_send_cmd, false, true, 5);
    send_cmd_box.set_width_chars(25);
    bt_send_cmd.set_label("Send Command");
    Serial_vbox.pack_start(Serial_hbox[2], false, true, 5);

    Tbox_V1.set_max_length(6);
    Tbox_V2.set_max_length(6);
    Tbox_V1.set_width_chars(6);
    Tbox_V2.set_width_chars(6);
    cb_test.append("Robot A");
    cb_test.append("Robot B");
    cb_test.append("Robot C");
    cb_test.append("Robot D");
    cb_test.append("Robot E");
    cb_test.append("Robot F");
    cb_test.append("All");

    cb_test.set_active(6); // ALL
    Tbox_V1.set_text("0.8");
    Tbox_V2.set_text("0.8");


    bt_Serial_Start.set_state(Gtk::STATE_NORMAL);
    cb_serial.set_state(Gtk::STATE_NORMAL);

    Tbox_V1.set_state(Gtk::STATE_INSENSITIVE);
    Tbox_V2.set_state(Gtk::STATE_INSENSITIVE);
    bt_Serial_Refresh.set_state(Gtk::STATE_INSENSITIVE);
    bt_Serial_test.set_state(Gtk::STATE_INSENSITIVE);
    cb_test.set_state(Gtk::STATE_INSENSITIVE);
    bt_Robot_Status.set_state(Gtk::STATE_INSENSITIVE);
    bt_reset_ack.set_state(Gtk::STATE_INSENSITIVE);

    bt_Serial_Start.set_label("Start");
    bt_Serial_Refresh.set_label("Refresh");

    bt_Serial_test.set_label("Send");

    _create_status_frame();

    pack_start(testFrame, false, true, 5);
    configureTestFrame();

    _update_cb_serial();
    // Conectar os sinais para o acontecimento dos eventos
    button_PID_Test.signal_pressed().connect(sigc::mem_fun(*this, &ControlGUI::_PID_Test));
    bt_Serial_test.signal_clicked().connect(sigc::mem_fun(*this, &ControlGUI::_send_test));
    bt_Serial_Refresh.signal_clicked().connect(sigc::mem_fun(*this, &ControlGUI::_update_cb_serial));
    bt_Serial_Start.signal_clicked().connect(sigc::mem_fun(*this, &ControlGUI::_start_serial));
    bt_Robot_Status.signal_clicked().connect(sigc::mem_fun(*this, &ControlGUI::_robot_status));
//	bt_reset_ack.signal_clicked().connect(sigc::mem_fun(*this, &ControlGUI::reset_lost_acks));
    bt_send_cmd.signal_clicked().connect(sigc::mem_fun(*this, &ControlGUI::_send_command));
}

void ControlGUI::configureTestFrame() {
    std::string labels[5] = {"Name 1", "Name 2", "Name 3", "Name 4", "Name 5"};
    double min[5] = {0, 0, 0, 0, 0};
    double max[5] = {100, 100, 100, 100, 100};
    double currentValue[5] = {0, 10, 20, 30, 40};
    double digits[5] = {0, 0, 0, 0, 0};
    double steps[5] = {1, 1, 1, 1, 1};

    for (int i = 0; i < 5; i++) {
        testFrame.setLabel(i, labels[i]);
        testFrame.configureHScale(i, currentValue[i],  min[i], max[i], digits[i], steps[i]);
    }
}

void ControlGUI::_send_command(){
	std::string cmd = send_cmd_box.get_text(); 
    messenger.send_old_format(cmd);
}

void ControlGUI::_PID_Test(){
    PID_test_flag = !PID_test_flag;
}

// Gets battery % and robot id to update a single robot's battery status
void ControlGUI::updateInterfaceStatus(double battery, int id) {
    if (battery > 20) {
        status_img[id].set("img/online.png");
        battery_bar[id].set_fraction(battery/100);
        status_lb[id].set_text(std::to_string(battery).substr(0,5)+"%");
    } else if (battery > 0) {
        status_img[id].set("img/critical.png");
        battery_bar[id].set_fraction(battery/100);
        status_lb[id].set_text(std::to_string(battery).substr(0,5)+"%");
    } else {
        status_img[id].set("img/zombie.png");
        battery_bar[id].set_fraction(0.0);
        battery_bar[id].set_text("0%");
        status_lb[id].set_text("DEAD");
    }
}

// update the battery status of all robots
void ControlGUI::_robot_status(){
    std::string dateString;
    time_t tt;

    // define last update time
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    tt = std::chrono::system_clock::to_time_t(now);
    dateString.append("Last Update: ");
    dateString.append(std::string(ctime(&tt)).substr(0,24));
    lastUpdate_lb.set_text(dateString);

    // update robot status
	for (int i = 0; i < TOTAL_ROBOTS; ++i) {
		char id = get_robot_id(i);
		double battery = messenger.get_battery(id);
		if(battery != -1) {
			updateInterfaceStatus(battery, i);
		} else { 
			status_img[i].set("img/offline.png");
			battery_bar[i].set_fraction(0.0);
            battery_bar[i].set_text("0%");
            status_lb[i].set_text("Offline");
		}
	}
}

void ControlGUI::_start_serial(){
    Glib::ustring serial = cb_serial.get_active_text();
	if (serial.empty()) return;
	
	messenger.start_xbee(serial);

    bt_Serial_Start.set_state(Gtk::STATE_INSENSITIVE);
    cb_serial.set_state(Gtk::STATE_INSENSITIVE);

    Tbox_V1.set_state(Gtk::STATE_NORMAL);
    Tbox_V2.set_state(Gtk::STATE_NORMAL);
    pid_edit_bt.set_state(Gtk::STATE_NORMAL);
    bt_Serial_Refresh.set_state(Gtk::STATE_NORMAL);
    bt_Serial_test.set_state(Gtk::STATE_NORMAL);
    cb_test.set_state(Gtk::STATE_NORMAL);
    pid_edit_bt.set_state(Gtk::STATE_NORMAL);
    bt_Robot_Status.set_state(Gtk::STATE_NORMAL);
}

bool ControlGUI::isFloat(std::string value){
    int counter = 0, i = 0;

    if (value.size() < 1 || value.front() == '.' || value.back() == '.')
        return false;

    if(!isdigit(value[0])) {
        if(value[0] != '-')
            return false;
        else
            i = 1;
    }

    for (; i < value.size(); i++) {
        if (value[i] == '.') counter++;
        else if (!isdigit(value[i])) return false;
    }
    // só pode ter um ponto
    if (counter > 1) return false;

    return true;
}

void ControlGUI::_send_test(){
    // verifica se os valores inseridos nos campos são válidos (são números entre -1.4 e 1.4)
    if(!isFloat(Tbox_V1.get_text()))
        Tbox_V1.set_text("0");
    if(!isFloat(Tbox_V2.get_text()))
        Tbox_V2.set_text("0");

    float v1 = std::stof(Tbox_V1.get_text());
    float v2 = std::stof(Tbox_V2.get_text());

    if(abs(v1) > 1.4) {
        if(v1 < 0)
            Tbox_V1.set_text("-1.4");
        else
            Tbox_V1.set_text("1.4");
    }
    
    if(abs(v2) > 1.4) {
        if(v2 < 0)
            Tbox_V2.set_text("-1.4");
        else
            Tbox_V2.set_text("1.4");
    }

	int pos = cb_test.get_active_row_number();
	std::string cmd = Tbox_V1.get_text()+";"+Tbox_V2.get_text();
	
	if(pos == -1) {
		return;
	} else if(pos != 6) {
		char id = get_robot_id(pos);
		messenger.send_msg(id,cmd);
	} else {
		for (int i = 0; i < 6; ++i) {
			char id = get_robot_id(i);
			messenger.send_msg(id,cmd);
		}
	}
}

int ControlGUI::get_robot_pos(char id) {
	return uint8_t(id)-65;
} 
 
char ControlGUI::get_robot_id(int pos) {
	return char(65+pos);
}

void ControlGUI::_update_cb_serial(){
	messenger.stop_xbee();

    cb_serial.remove_all();
	
    for(int i = 0; i < 256; ++i) {
        std::string port = "/dev/ttyUSB";
		port.append(std::to_string(i));

        int fd = open(port.c_str(), O_RDWR);
        if(fd != -1) {
            std::cout<<port<<std::endl;
            cb_serial.append(port);
			close(fd);
        }
    }
    
    bt_Serial_Start.set_state(Gtk::STATE_NORMAL);
    cb_serial.set_state(Gtk::STATE_NORMAL);
    bt_Serial_Refresh.set_state(Gtk::STATE_NORMAL);

    pid_edit_bt.set_state(Gtk::STATE_INSENSITIVE);
    Tbox_V1.set_state(Gtk::STATE_INSENSITIVE);
    Tbox_V2.set_state(Gtk::STATE_INSENSITIVE);
    bt_Serial_test.set_state(Gtk::STATE_INSENSITIVE);
    cb_test.set_state(Gtk::STATE_INSENSITIVE);
    bt_Robot_Status.set_state(Gtk::STATE_INSENSITIVE);
}

void ControlGUI::_create_status_frame(){

    pack_start(status_fm, false, true, 5);
    status_fm.set_label("Robot Status");
    status_fm.add(status_grid);

    status_grid.set_border_width(10);
    status_grid.set_column_spacing(10);
    status_grid.set_halign(Gtk::ALIGN_CENTER);

    bt_Robot_Status.set_label("Update");
    bt_Robot_Status.set_state(Gtk::STATE_NORMAL);
    status_grid.attach(bt_Robot_Status, 0, 0, 1, 1);
    lastUpdate_lb.set_text("Last Update: -");
    lastUpdate_lb.set_valign(Gtk::ALIGN_BASELINE);
    status_grid.attach(lastUpdate_lb, 1, 0, 3, 1);
    bt_reset_ack.set_label("Reset Lost ACKs");
    status_grid.attach(bt_reset_ack, 4, 0, 1, 1);

    std::vector<std::string> name;
    name.push_back("Robot A");
    name.push_back("Robot B");
    name.push_back("Robot C");
    name.push_back("Robot D");
    name.push_back("Robot E");
    name.push_back("Robot F");

    for (int i = 0; i < TOTAL_ROBOTS; i++) {
        status_img[i].set("img/offline.png");
        status_grid.attach(status_img[i], 0, i+1, 1, 1);
        robots_lb[i].set_text(name.at(i));
        status_grid.attach(robots_lb[i], 1, i+1, 1, 1);
        battery_bar[i].set_valign(Gtk::ALIGN_CENTER);
        status_grid.attach(battery_bar[i], 2, i+1, 1, 1);
        status_lb[i].set_text("Offline");
        status_grid.attach(status_lb[i], 3, i+1, 1, 1);
        dropped_frames_text[i].set_label("Lost ACKs: ");
        status_grid.attach(dropped_frames_text[i], 4, i+1, 1, 1);
        dropped_frames[i].set_label("- ");
		status_grid.attach(dropped_frames[i], 5, i+1, 1, 1);
    }
}

void ControlGUI::update_dropped_frames(std::vector<message> acks) {
    for(message ack : acks) {
        int pos = get_robot_pos(ack.id);
        if(ack.data != "0") dropped_frames_num[pos]++;
        total_frames_num[pos]++;
        double rate = double(dropped_frames_num[pos])/double(total_frames_num[pos])*100;
        std::stringstream stream;
        stream << std::fixed << std::setprecision(3) << rate << "%";
        std::string dropped_f = stream.str();
        dropped_frames[pos].set_label(dropped_f);
    }
}

void ControlGUI::reset_lost_acks() {
    for (int i = 0; i < TOTAL_ROBOTS; ++i) {
		dropped_frames_num[i] = 0;
        total_frames_num[i] = 0;
        dropped_frames[i].set_label("- ");
    }
}

// Função para verificar se os valores digitados nos campos
// de PID são válidos: apenas números e um único ponto
bool ControlGUI::checkPIDvalues(){
    std::string value;
    int counter;
    for (int i = 0; i < 3; i++) {
        counter = 0;
        value.clear();
        value.append(pid_box[i].get_text());
        std::cout << i << ": " << value << std::endl;

        if (value.front() == '.' || value.back() == '.')
            return false;
        for (int j = 0; j < value.size(); j++) {
            if (value[j] == '.') counter++;
            if (!isdigit(value[j]) && value[j] != '.') return false;
        }
        if (counter > 1) return false;
    }
    return true;
}
