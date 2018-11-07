/*
 * controlGUI.hpp
 *
 *  Created on: May 8th, 2016
 *      Author: Daniel
 *
 * Este código cria a interface da aba 'Control' do código do VS.
 * Para liberar os widgets, o botão PID deve estar pressionado.
 * Nas caixas de texto, pode-se usar '.' ou ',' para tratar números
 * do tipo double.
 *
 * Conta com uma Hscale e uma Entry (o TextBox do Gtkmm) para que possa
 * colocar o valor do PID de cada jogador.
 */

#ifndef CONTROLGUI_HPP_
#define CONTROLGUI_HPP_

#include <gtkmm.h>
#include <string>
#include "Messenger.h"
#include <unistd.h>
#include <fcntl.h>
// system_clock::now
#include <iostream>
#include <ctime>
#include <chrono>

class ControlGUI : public Gtk::VBox {
	private:
		int xbee_connections = 0;

	public:
		Messenger messenger;

		const static int TOTAL_ROBOTS = 6;

		// Flag para saber se o botão PID está pressionado ou não.
		bool PID_test_flag = false;
		// Containers para o conteúdo da interface gráfica
		Gtk::Frame Serial_fm;
		Gtk::HBox Top_hbox;
		Gtk::VBox Serial_vbox;
		Gtk::HBox Serial_hbox[5];
		Gtk::Label *label;
		Gtk::Button bt_send_cmd;
		Gtk::Entry send_cmd_box;

		Gtk::Entry entry_set_frameskip;
		Gtk::Button bt_set_frameskip;
		Gtk::Label time_msgs;
		double acc_time;
		int time_count;

		// Botões e combo box Rádio
		Gtk::Button bt_Serial_Start;
		Gtk::Button bt_Robot_Status;
		Gtk::Button bt_Serial_Refresh;
		Gtk::ComboBoxText cb_serial;
		Gtk::ToggleButton button_PID_Test;
		Gtk::Button bt_Serial_test;
		Gtk::ComboBoxText cb_test;
		Gtk::Entry Tbox_V1;
		Gtk::Entry Tbox_V2;

		Gtk::CheckButton ack_enable_button;
		Gtk::Label ack_enable_label;

		bool ekf_always_send = false;
		Gtk::CheckButton ekf_always_send_button;
		Gtk::Label ekf_always_send_label;

		Gtk::Grid status_grid;
		Gtk::Frame status_fm;
		Gtk::Image status_img[TOTAL_ROBOTS];
		Gtk::Label robots_lb[TOTAL_ROBOTS];
		Gtk::Label status_lb[TOTAL_ROBOTS];
		Gtk::Label lastUpdate_lb;
		Gtk::ProgressBar battery_bar[TOTAL_ROBOTS];
		Gtk::Label dropped_frames[TOTAL_ROBOTS];
		Gtk::Button bt_reset_ack;

		Gtk::Button pid_edit_bt;

		ControlGUI();

		void _send_command();

		void _PID_Test();

		// Gets battery % and robot id to update a single robot's battery status
		void updateInterfaceStatus(double battery, int id);

		// update the battery status of all robots
		void _robot_status();

		void _start_serial();

		bool isFloat(std::string value);

		void _send_test();

		void _update_cb_serial();

		void _create_status_frame();

		char get_robot_id(int pos);
		void update_dropped_frames();
		void reset_lost_acks();
		void update_ack_interface();
		void set_frameskipper();
		void update_msg_time();
		void ekf_always_send_enable();

		// essa função não deve ser chamada imediatamente após desconectar o xbee
		// o xbee demora alguns segundos para realmente desconectar
		// conecta o xbee automaticamente caso tenha apenas um conectado
		void auto_start_serial();
};

#endif /* CONTROLGUI_HPP_ */
