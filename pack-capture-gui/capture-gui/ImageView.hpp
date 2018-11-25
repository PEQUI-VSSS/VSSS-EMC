/*
 * ImageView.hpp
 *
 *  Created on: Feb 3, 2014
 *      Author: gustavo
 */

#ifndef IMAGEVIEW_HPP_
#define IMAGEVIEW_HPP_

#include <gtkmm.h>
#include <cairomm/context.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <ImageWarp.hpp>

#pragma once

namespace capture {
	class ImageView;
}

class capture::ImageView : public Gtk::DrawingArea {

		Glib::RefPtr<Gdk::Pixbuf> pb;
		bool on_button_press_event(GdkEventButton *event) override;

	public:
		warp::ImageWarp imageWarp;

		unsigned char *data;
		int width, height, stride;

		int gmm_clicks[2][2];

		double tar_pos[2];
		double robot_pos[2];
		double look_pos[2];

		int gmm_counter = 0;
		bool warp_event_flag = false;
		bool PID_test_flag = false;
		bool adjust_event_flag = false;
		bool gmm_sample_flag = false;
		bool gmm_ready_flag = false;
		bool split_flag = false;
		int sector = -1;

		ImageView();
		void set_data(unsigned char *data, int width, int height);
		void disable_image_show();
		void refresh();
		bool on_draw(const Cairo::RefPtr<Cairo::Context> &cr) override;
};

#endif /* IMAGEVIEW_HPP_ */
