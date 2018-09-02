/*
* v4lgui.hpp
*
*  Created on: Feb 1, 2014
*      Author: gustavo
*/

#ifndef V4LINTERFACE_HPP_
#define V4LINTERFACE_HPP_

#include <Goalkeeper.hpp>
#include <Defender.hpp>
#include <Attacker.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <gtkmm.h>
#include <gtkmm/messagedialog.h>
#include <linux/videodev2.h>
#include "v4lcap.hpp"
#include "ImageView.hpp"
#include "Messenger.h"
#include "../../cc/vision/visionGUI.hpp"
#include <ctime>
#include <chrono>

#define PI 3.14159265453

namespace capture {
	class V4LInterface;
	struct __ctrl_holder;
}

typedef struct capture::__ctrl_holder {
	struct v4l2_queryctrl qctrl;
	Gtk::Widget *widget;
	sigc::connection con;
} ControlHolder;

class capture::V4LInterface : public Gtk::VBox {

	public:
		const std::array<Robot2*, 3>& robots;

		bool isLowRes;

		VisionGUI visionGUI;

		bool warped = false;

		ImageView imageView;
		Messenger* messenger;

		std::list<ControlHolder> ctrl_list_default;

		std::string camera_card;

		Gtk::Image red_button_released;
		Gtk::Image red_button_pressed;

		Gtk::Label robot_pos_lb_list[3];
		Gtk::Label ball_pos_lb;

		Gtk::Frame robots_id_fm;
		Gtk::HBox info_hbox;
		Gtk::VBox robots_pos_vbox;
		Gtk::HBox robots_pos_hbox[7];
		Gtk::HBox start_game_hbox;
		Gtk::VBox buttons_vbox;
		Gtk::Button start_game_bt;

		Gtk::Frame robots_pos_fm;
		Gtk::Frame robots_buttons_fm;
		Gtk::VBox robots_pos_buttons_vbox;
		Gtk::HBox fps_hbox;
		Gtk::Label *fps_label;

		Gtk::CheckButton record_video_checkbox;

		bool reset_warp_flag;

		char robotsIDs[6] = {'A', 'B', 'C', 'D', 'E', 'F'};
		Gtk::VBox robots_id_vbox;
		Gtk::HBox robots_id_hbox[4];
		Gtk::Button robots_id_edit_bt;
		Gtk::Button robots_auto_bt;
		Gtk::Button robots_id_done_bt;
		Gtk::ComboBoxText robots_id_box[3];
		int robots_id_tmp[3];
		bool robots_id_edit_flag = false;

		Gtk::Frame robots_speed_fm;
		Gtk::VBox robots_speed_vbox[4];
		Gtk::HScale robots_speed_hscale[3];
		double robots_speed_tmp[3];
		Gtk::HBox robots_speed_hbox[4];
		Gtk::ProgressBar robots_speed_progressBar[3];
		Gtk::Button robots_speed_edit_bt;
		Gtk::Button robots_speed_done_bt;
		bool robots_speed_edit_flag = false;

		Gtk::Label robot_label[3];
		Gtk::Frame robots_function_fm;
		Gtk::VBox robots_function_vbox;
		Gtk::HBox robots_function_hbox[4];
		Gtk::ComboBoxText cb_robot_function[3];
		int robots_function_tmp[3];
		Gtk::Button robots_function_edit_bt;
		Gtk::Button robots_function_done_bt;
		bool robots_function_edit_flag = false;

		capture::v4lcap vcap;

		V4LInterface(Messenger *messenger_ptr, const std::array<Robot2*, 3>& robots_ref);
		explicit V4LInterface(bool isLow, Messenger *messenger_ptr, const std::array<Robot2*, 3>& robots_ref);
		void initInterface();

		int offsetL;
		int offsetR;
		Gtk::Scale HScale_offsetL;
		Gtk::Scale HScale_offsetR;
		bool adjust_event_flag;
		bool invert_image_flag = false;

		Gtk::ToggleButton bt_warp;
		Gtk::ToggleButton bt_adjust;
		Gtk::CheckButton bt_invert_image;

		// Signals
		sigc::connection cb_input_signal;
		sigc::connection cb_standard_signal;
		sigc::connection cb_format_desc_signal;
		sigc::connection cb_frame_size_signal;
		sigc::connection cb_frame_interval_signal;

		// Camera calibration frames
		Gtk::Frame fr_camCalib_offline;
		Gtk::Frame fr_camCalib_online;
		Gtk::Button btn_camCalib_collect, btn_camCalib_reset, btn_camCalib_start, btn_camCalib_pop, btn_camCalib_offline_start;
		Gtk::ToggleButton btn_camCalib;

		bool CamCalib_flag_event;

		void __event_bt_quick_save_clicked();
		void __event_bt_quick_load_clicked();
		void __event_bt_save_clicked();
		void __event_bt_load_clicked();
		void __event_bt_start_clicked();
		void __event_bt_warp_clicked();
		void __event_bt_adjust_pressed();
		void __event_bt_reset_warp_clicked();
		void HScale_offsetR_value_changed();
		void HScale_offsetL_value_changed();
		void __event_bt_invert_image_signal_clicked();
		void __event_cb_device_changed();
		void __event_cb_input_changed();
		void __event_cb_standard_changed();
		void __event_cb_format_desc_changed();
		void __event_cb_frame_size_changed();
		void __event_camCalib_mode_clicked();
		void __event_camCalib_online_collect_clicked();
		void __event_camCalib_online_pop_clicked();
		void __event_camCalib_online_reset_clicked();
		void __event_camCalib_online_start_clicked();
		void __event_camCalib_offline_start_clicked();
		void __event_camCalib_pressed();
		void __event_cb_frame_interval_changed();
		void createIDsFrame();
		void createSpeedsFrame();
		void update_speed_progressBars();
		void update_robot_functions();
		void createPositionsAndButtonsFrame();
		void createFunctionsFrame();
		void event_robots_function_done_bt_signal_clicked();
		void event_robots_function_edit_bt_signal_clicked();
		void event_start_game_bt_signal_clicked();
		void event_disable_video_record();
		void event_robots_speed_done_bt_signal_clicked();
		void event_robots_speed_edit_bt_signal_pressed();
		void event_robots_id_done_bt_signal_clicked();
		void event_robots_id_edit_bt_signal_pressed();
		void event_robots_auto_bt_signal_pressed();
		void updateRobotLabels();
		void update_ball_position(Geometry::Point ball);
		void updateFPS(int fps);
		bool get_start_game_flag();
		void update_interface_robots();
		void update_interface_camera();

		/* Signals */
		typedef sigc::signal<bool, bool> SignalStart;

		SignalStart signal_start() {
			return m_signal_start;
		}

	protected:
		SignalStart m_signal_start;

		bool on_button_press_event(GdkEventButton *event) override;

	private:
		bool start_game_flag = false;

		void __init_combo_boxes();
		void __create_frm_device_info();
		void __create_frm_device_properties();
		void __create_frm_cam_calib();
		void __create_frm_CamCalibMode_Offline();
		void __create_frm_CamCalibMode_Online();
		void createQuickActionsFrame();
		void __create_frm_warp();

		// Combo properties updates
		void __update_cb_device();
		void __update_cb_input();
		void __update_cb_standard();
		void __update_cb_format_desc();
		void __update_cb_frame_size();
		void __update_cb_frame_interval();
		void __update_all();

		Gtk::Frame frm_device_info;
		Gtk::ComboBoxText cb_device;
		Gtk::Button bt_start;
		Gtk::Button bt_reset_warp;

		Gtk::Frame frm_quick_actions;
		Gtk::Button bt_quick_save;
		Gtk::Button bt_quick_load;
		Gtk::Button bt_save;
		Gtk::Button bt_load;

		Gtk::Label lb_device_name;
		Gtk::Label lb_device_card;
		Gtk::Label lb_device_driver;
		Gtk::Label lb_device_bus;

		Gtk::Frame frm_device_prop;
		Gtk::Frame frm_warp;
		Gtk::SpinButton sp_width;
		Gtk::SpinButton sp_height;
		Gtk::Frame frm_cam_calib;

		Gtk::ComboBox cb_input;
		Gtk::ComboBox cb_standard;
		Gtk::ComboBox cb_format_desc;
		Gtk::ComboBox cb_frame_size;
		Gtk::ComboBox cb_frame_interval;

		Glib::RefPtr<Gtk::ListStore> ls_input;
		Glib::RefPtr<Gtk::ListStore> ls_standard;
		Glib::RefPtr<Gtk::ListStore> ls_format_desc;
		Glib::RefPtr<Gtk::ListStore> ls_frame_size;
		Glib::RefPtr<Gtk::ListStore> ls_frame_interval;

		// Camera Calibration Mode
		Gtk::RadioButton calib_offline, calib_online;

		template<class T>
		class ModelColumn : public Gtk::TreeModel::ColumnRecord {
			public:

				ModelColumn() {
					add(m_col_name);
					add(m_col_data);
				}

				Gtk::TreeModelColumn<Glib::ustring> m_col_name;
				Gtk::TreeModelColumn<T> m_col_data;
		};

		typedef ModelColumn<struct v4l2_input> ModelInput;
		typedef ModelColumn<struct v4l2_standard> ModelStandard;
		typedef ModelColumn<struct v4l2_fmtdesc> ModelFormatDesc;
		typedef ModelColumn<struct v4l2_frmsizeenum> ModelFrameSize;
		typedef ModelColumn<struct v4l2_querymenu> ModelControlMenu;
		typedef ModelColumn<struct v4l2_frmivalenum> ModelFrameInterval;

		ModelInput model_input;
		ModelStandard model_standard;
		ModelFormatDesc model_format_desc;
		ModelFrameSize model_frame_size;
		ModelFrameInterval model_frame_interval;

		ModelControlMenu model_control_menu;

		Gtk::Notebook notebook;

		Gtk::ScrolledWindow scrolledWindow;
		Gtk::VBox capture_vbox;

		void __make_control_list_default();
		void __make_control_table(std::list<ControlHolder> &list, const char *title);
		void __update_control_widgets(std::list<ControlHolder> &list);
		void __block_control_signals(std::list<ControlHolder> &list, bool block);
		bool __set_control_hscale(int type, double val, std::list<ControlHolder> *list, Gtk::Widget *wctrl);
		void __set_control(std::list<ControlHolder> *list, Gtk::Widget *wctrl);
		void discover_robot_ids();
};

#endif /* V4LINTERFACE_HPP_ */
