#include "visionGUI.hpp"

using namespace vision;

void VisionGUI::__create_frm_drawing_options() {
	Gtk::Frame *frame;
	Gtk::HBox *hbox;

	hbox = new Gtk::HBox();
	frame = new Gtk::Frame();

	pack_start(*frame, false, true, 5);
	frame->add(*hbox);
	frame->set_label("Drawing Options");
	hbox->set_halign(Gtk::ALIGN_CENTER);
	hbox->pack_start(draw_info_checkbox, false, true, 5);
	draw_info_checkbox.set_label("Disable Drawing");
	draw_info_checkbox.set_can_focus(false);
	draw_info_checkbox.set_margin_bottom(5);

	draw_info_checkbox.signal_clicked().connect(
			sigc::mem_fun(*this, &VisionGUI::event_draw_info_checkbox_signal_clicked));
}

void VisionGUI::__create_frm_calib_mode() {
	Gtk::VBox *vbox;
	Gtk::Grid *grid;
	Gtk::Frame *frame;
	Gtk::Label *label;

	vbox = new Gtk::VBox();
	grid = new Gtk::Grid();
	frame = new Gtk::Frame();

	pack_start(*frame, false, false, 5);

	frame->add(*vbox);
	vbox->pack_start(*grid, false, true, 5);
	vbox->set_halign(Gtk::ALIGN_CENTER);
	vbox->set_valign(Gtk::ALIGN_CENTER);

	frame->set_label("Choose Your Destiny");

	grid->set_border_width(5);
	grid->set_column_spacing(10);
	grid->set_row_spacing(5);

	label = new Gtk::Label("Calibration Mode: ");
	grid->attach(*label, 0, 0, 1, 1);
	rb_mode_CIELAB.set_label("CIELAB");
	grid->attach(rb_mode_CIELAB, 1, 0, 1, 1);
	rb_mode_GMM.set_label("GMM");
	rb_mode_GMM.join_group(rb_mode_CIELAB);
	grid->attach(rb_mode_GMM, 2, 0, 1, 1);

	rb_mode_CIELAB.set_state(Gtk::STATE_INSENSITIVE);
	rb_mode_GMM.set_state(Gtk::STATE_INSENSITIVE);

	rb_mode_CIELAB.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_rb_mode_clicked));
	rb_mode_GMM.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_rb_mode_clicked));
}

void VisionGUI::__event_rb_mode_clicked() {
	if (rb_mode_GMM.get_active()) {
		isCIELAB = false;
		fr_CIELAB.hide();
		fr_GMM.show();
		fr_splitView.hide();
		disableSplitView = true;
	} else {
		fr_CIELAB.show();
		fr_GMM.hide();
		fr_splitView.show();
		isCIELAB = true;
		disableSplitView = false;
	}
}

void VisionGUI::__create_frm_gmm() {
	Gtk::VBox *vbox;
	Gtk::Grid *grid;
	Gtk::Frame *frame;
	Gtk::Label *label;
	Gtk::HBox *hbox;

	vbox = new Gtk::VBox();

	pack_start(fr_GMM, false, false, 5);

	fr_GMM.add(*vbox);
	vbox->set_halign(Gtk::ALIGN_CENTER);
	vbox->set_valign(Gtk::ALIGN_CENTER);

	fr_GMM.set_label("GMM Calibration");

	hbox = new Gtk::HBox();
	vbox->pack_start(*hbox, false, true, 5);
	hbox->set_halign(Gtk::ALIGN_CENTER);
	hbox->set_valign(Gtk::ALIGN_CENTER);

	frame = new Gtk::Frame();
	grid = new Gtk::Grid();
	grid->set_margin_left(10);
	grid->set_margin_right(10);
	grid->set_margin_top(5);
	grid->set_margin_bottom(5);
	grid->set_column_spacing(5);
	grid->set_column_homogeneous(true);
	frame->set_label("Samples");
	frame->add(*grid);
	hbox->pack_start(*frame, false, true, 5);

	bt_collectSamples.set_label("Collect");
	grid->attach(bt_collectSamples, 0, 0, 1, 1);
	bt_popSample.set_label("Pop (0)");
	grid->attach(bt_popSample, 1, 0, 1, 1);
	bt_clearSamples.set_label("Reset");
	grid->attach(bt_clearSamples, 2, 0, 1, 1);

	frame = new Gtk::Frame();
	grid = new Gtk::Grid();
	grid->set_margin_left(10);
	grid->set_margin_right(10);
	grid->set_margin_top(5);
	grid->set_margin_bottom(5);
	grid->set_column_spacing(5);
	grid->set_column_homogeneous(true);
	frame->set_label("Save/Load");
	frame->add(*grid);
	hbox->pack_start(*frame, false, true, 5);

	bt_GMM_save.set_label("Save");
	grid->attach(bt_GMM_save, 0, 0, 1, 1);
	bt_GMM_load.set_label("Load");
	grid->attach(bt_GMM_load, 3, 0, 1, 1);

	frame = new Gtk::Frame();
	grid = new Gtk::Grid();
	grid->set_border_width(10);
	grid->set_column_spacing(10);
	grid->set_row_spacing(5);
	grid->set_halign(Gtk::ALIGN_CENTER);
	grid->set_valign(Gtk::ALIGN_CENTER);
	frame->set_label("Training");
	frame->add(*grid);
	vbox->pack_start(*frame, false, true, 5);

	label = new Gtk::Label("Clusters: ");
	grid->attach(*label, 0, 0, 1, 1);
	HScale_clusters.set_digits(0);
	HScale_clusters.set_size_request(125);
	HScale_clusters.set_increments(1, 3);
	HScale_clusters.set_range(1, 15);
	HScale_clusters.set_value_pos(Gtk::POS_RIGHT);
	HScale_clusters.set_draw_value();
	grid->attach(HScale_clusters, 1, 0, 2, 1);
	bt_trainGMM.set_label("Train GMM");
	grid->attach(bt_trainGMM, 4, 0, 1, 1);

	frame = new Gtk::Frame();
	auto *innerVbox = new Gtk::VBox();
	frame->set_label("Frame");
	frame->add(*innerVbox);
	vbox->pack_start(*frame, false, true, 5);

	hbox = new Gtk::HBox();
	hbox->set_halign(Gtk::ALIGN_CENTER);
	hbox->set_valign(Gtk::ALIGN_CENTER);
	rb_GMM_original.set_label("Original");
	hbox->pack_start(rb_GMM_original, false, true, 5);
	rb_GMM_gaussians.set_label("Gaussians");
	rb_GMM_gaussians.join_group(rb_GMM_original);
	hbox->pack_start(rb_GMM_gaussians, false, true, 5);
	rb_GMM_final.set_label("Final");
	rb_GMM_final.join_group(rb_GMM_original);
	hbox->pack_start(rb_GMM_final, false, true, 5);
	rb_GMM_threshold.set_label("Threshold");
	rb_GMM_threshold.join_group(rb_GMM_original);
	hbox->pack_start(rb_GMM_threshold, false, true, 5);
	innerVbox->pack_start(*hbox, false, true, 5);

	hbox = new Gtk::HBox();
	hbox->set_halign(Gtk::ALIGN_CENTER);
	hbox->set_valign(Gtk::ALIGN_CENTER);
	label = new Gtk::Label("Threshold: ");
	hbox->pack_start(*label, false, true, 5);
	bt_GMM_left.set_label("<");
	hbox->pack_start(bt_GMM_left, false, true, 5);
	lb_threshold.set_text(realColors.at(0));
	hbox->pack_start(lb_threshold, false, true, 5);
	bt_GMM_right.set_label(">");
	hbox->pack_start(bt_GMM_right, false, true, 5);
	innerVbox->pack_start(*hbox, false, true, 5);

	hbox = new Gtk::HBox();
	hbox->set_halign(Gtk::ALIGN_CENTER);
	hbox->set_valign(Gtk::ALIGN_CENTER);
	cb_gaussianColor.append("Select Gaus.:");
	for (unsigned long i = 0; i < gmm->getClusters(); i++) {
		cb_gaussianColor.append(gaussianColors.at(i));
	}
	cb_gaussianColor.set_active(0);
	hbox->pack_start(cb_gaussianColor, false, true, 5);
	cb_realColor.append("Select Color:");
	cb_realColor.append("Background");
	cb_realColor.append("Main");
	cb_realColor.append("Green");
	cb_realColor.append("Ball");
	cb_realColor.append("Opponent");
	cb_realColor.set_active(0);
	hbox->pack_start(cb_realColor, false, true, 5);
	bt_GMM_match.set_label("Match!");
	hbox->pack_start(bt_GMM_match, false, true, 5);
	bt_GMM_done.set_label("Done");
	hbox->pack_start(bt_GMM_done, false, true, 5);
	innerVbox->pack_start(*hbox, false, true, 5);

	frame = new Gtk::Frame();
	grid = new Gtk::Grid();
	grid->set_border_width(10);
	grid->set_column_spacing(10);
	grid->set_row_spacing(5);
	grid->set_halign(Gtk::ALIGN_CENTER);
	grid->set_valign(Gtk::ALIGN_CENTER);
	frame->set_label("Pos-Processing");
	frame->add(*grid);
	vbox->pack_start(*frame, false, true, 5);
	vbox->set_halign(Gtk::ALIGN_CENTER);

	label = new Gtk::Label("Blur: ");
	grid->attach(*label, 4, 0, 1, 1);
	HScale_GMM_blur.set_digits(0);
	HScale_GMM_blur.set_size_request(50);
	HScale_GMM_blur.set_increments(1, 2);
	HScale_GMM_blur.set_range(0, 3);
	HScale_GMM_blur.set_value_pos(Gtk::POS_RIGHT);
	HScale_GMM_blur.set_draw_value();
	grid->attach(HScale_GMM_blur, 5, 0, 1, 1);
	label = new Gtk::Label("Erode: ");
	grid->attach(*label, 6, 0, 1, 1);
	HScale_GMM_erode.set_digits(0);
	HScale_GMM_erode.set_size_request(50);
	HScale_GMM_erode.set_increments(1, 2);
	HScale_GMM_erode.set_range(0, 5);
	HScale_GMM_erode.set_value_pos(Gtk::POS_RIGHT);
	HScale_GMM_erode.set_draw_value();
	grid->attach(HScale_GMM_erode, 7, 0, 1, 1);
	label = new Gtk::Label("Dilate: ");
	grid->attach(*label, 8, 0, 1, 1);
	HScale_GMM_dilate.set_digits(0);
	HScale_GMM_dilate.set_size_request(50);
	HScale_GMM_dilate.set_increments(1, 2);
	HScale_GMM_dilate.set_range(0, 5);
	HScale_GMM_dilate.set_value_pos(Gtk::POS_RIGHT);
	HScale_GMM_dilate.set_draw_value();
	grid->attach(HScale_GMM_dilate, 9, 0, 1, 1);

	bt_GMM_save.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_GMM_save_clicked));
	bt_GMM_load.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_GMM_load_clicked));
	bt_collectSamples.signal_pressed().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_collectSamples_pressed));
	bt_popSample.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_popSample_clicked));
	bt_clearSamples.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_clearSamples_clicked));
	bt_trainGMM.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_trainGMM_clicked));
	HScale_clusters.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_clusters_value_changed));
	bt_GMM_match.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_GMM_match_clicked));
	bt_GMM_done.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_GMM_done_clicked));
	rb_GMM_original.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_rb_GMM_frame_clicked));
	rb_GMM_gaussians.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_rb_GMM_frame_clicked));
	rb_GMM_final.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_rb_GMM_frame_clicked));
	rb_GMM_threshold.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_rb_GMM_frame_clicked));
	bt_GMM_left.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_GMM_left_clicked));
	bt_GMM_right.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_GMM_right_clicked));
	HScale_GMM_blur.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_GMM_blur_value_changed));
	HScale_GMM_erode.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_GMM_erode_value_changed));
	HScale_GMM_dilate.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_GMM_dilate_value_changed));
}

void VisionGUI::hideGMM() {
	fr_GMM.hide();
}

void VisionGUI::HScale_GMM_dilate_value_changed() {
	gmm->setDilate(colorIndex, static_cast<int>(HScale_GMM_dilate.get_value()));
}

void VisionGUI::HScale_GMM_blur_value_changed() {
	auto value = static_cast<int>(HScale_GMM_blur.get_value());
	if (value % 2 == 0 && value != 0) value++;
	gmm->setBlur(colorIndex, value);
}

void VisionGUI::HScale_GMM_erode_value_changed() {
	gmm->setErode(colorIndex, static_cast<int>(HScale_GMM_erode.get_value()));
}

bool VisionGUI::getDrawSamples() {
	return samplesEventFlag;
}

bool VisionGUI::getGaussiansFrameFlag() {
	return gaussiansFrame_flag;
}

bool VisionGUI::getFinalFrameFlag() {
	return finalFrame_flag;
}

bool VisionGUI::getThresholdFrameFlag() {
	return thresholdFrame_flag;
}

bool VisionGUI::getIsDrawing() {
	return !draw_info_flag;
}

void VisionGUI::__event_bt_GMM_save_clicked() {
	FileChooser saveWindow;

	if (saveWindow.result == Gtk::RESPONSE_OK) gmm->write(saveWindow.fileName);
}

void VisionGUI::__event_bt_GMM_load_clicked() {
	FileChooser loadWindow;

	if (loadWindow.result == Gtk::RESPONSE_OK) {
		if (gmm->read(loadWindow.fileName)) {
			rb_GMM_gaussians.set_active(true);
			rb_GMM_gaussians.clicked();
			HScale_clusters.set_value(gmm->getClusters());
		}
	}
}

void VisionGUI::quickLoadGMM() {
	gmm->read("autoGMM.json");
	HScale_clusters.set_value(gmm->getClusters());
	HScale_GMM_blur.set_value(gmm->getBlur(colorIndex));
	HScale_GMM_erode.set_value(gmm->getErode(colorIndex));
	HScale_GMM_dilate.set_value(gmm->getErode(colorIndex));
}

void VisionGUI::__event_bt_GMM_left_clicked() {
	if (colorIndex - 1 == 0) colorIndex = realColors.size() - 1;
	else colorIndex--;

	lb_threshold.set_text(realColors.at(colorIndex));
	HScale_GMM_blur.set_value(gmm->getBlur(colorIndex));
	HScale_GMM_dilate.set_value(gmm->getDilate(colorIndex));
	HScale_GMM_erode.set_value(gmm->getErode(colorIndex));
}

void VisionGUI::__event_bt_GMM_right_clicked() {
	if (colorIndex + 1 >= realColors.size()) colorIndex = 0;
	else colorIndex++;

	lb_threshold.set_text(realColors.at(colorIndex));
	HScale_GMM_blur.set_value(gmm->getBlur(colorIndex));
	HScale_GMM_dilate.set_value(gmm->getDilate(colorIndex));
	HScale_GMM_erode.set_value(gmm->getErode(colorIndex));
}

void VisionGUI::__event_rb_GMM_frame_clicked() {
	if (rb_GMM_original.get_active()) {
		gaussiansFrame_flag = false;
		finalFrame_flag = false;
		thresholdFrame_flag = false;
	} else if (rb_GMM_gaussians.get_active()) {
		gaussiansFrame_flag = true;
		finalFrame_flag = false;
		thresholdFrame_flag = false;
	} else if (rb_GMM_final.get_active()) {
		gaussiansFrame_flag = false;
		finalFrame_flag = true;
		thresholdFrame_flag = false;
	} else {
		gaussiansFrame_flag = false;
		finalFrame_flag = false;
		thresholdFrame_flag = true;
	}
}

void VisionGUI::__event_bt_GMM_match_clicked() {
	int gaussian = cb_gaussianColor.get_active_row_number();
	int color = cb_realColor.get_active_row_number();
	if (gaussian >= 0 && color >= 0) gmm->setMatchColor(static_cast<unsigned long>(gaussian - 1), color - 1);
}

void VisionGUI::__event_bt_GMM_done_clicked() {
	if (!gmm->getDoneFlag()) {
		gmm->setDone(true);
		bt_GMM_done.set_label("Reset");
		// rb_GMM_threshold.set_state(Gtk::STATE_NORMAL);
	} else {
		gmm->setDone(false);
		bt_GMM_done.set_label("Done");
		// rb_GMM_threshold.set_state(Gtk::STATE_INSENSITIVE);
	}
}

void VisionGUI::HScale_clusters_value_changed() {
	gmm->setClusters(static_cast<int>(HScale_clusters.get_value()));

	cb_gaussianColor.remove_all();
	cb_gaussianColor.append("Select Gaussian:");
	cb_gaussianColor.set_active(0);
	for (unsigned long i = 0; i < gmm->getClusters(); i++) {
		cb_gaussianColor.append(gaussianColors.at(i));
	}
}

void VisionGUI::__event_bt_trainGMM_clicked() {
	int res = gmm->train();
	if (res == 0) {
		rb_GMM_gaussians.set_active(true);
		rb_GMM_gaussians.clicked();
		bt_collectSamples.set_active(false);
		bt_clearSamples.set_state(Gtk::STATE_INSENSITIVE);
		bt_popSample.set_state(Gtk::STATE_INSENSITIVE);
		bt_GMM_save.set_state(Gtk::STATE_NORMAL);
		samplesEventFlag = false;
	} else HScale_clusters.set_state(Gtk::STATE_NORMAL);
}

void VisionGUI::__event_bt_collectSamples_pressed() {
	if (samplesEventFlag) {
		bt_popSample.set_state(Gtk::STATE_INSENSITIVE);
		bt_clearSamples.set_state(Gtk::STATE_INSENSITIVE);
	} else {
		bt_popSample.set_state(Gtk::STATE_NORMAL);
		bt_clearSamples.set_state(Gtk::STATE_NORMAL);
	}

	samplesEventFlag = !samplesEventFlag;
}

void VisionGUI::__event_bt_popSample_clicked() {
	gmm->popSample();
	decrementSamples();
	std::cout << "GMM sample popped. Total left: " << gmm->getSamplesSize() << std::endl;
}

void VisionGUI::__event_bt_clearSamples_clicked() {
	gmm->clearSamples();
	totalSamples = 0;
	bt_popSample.set_label("Pop (0)");
	std::cout << "GMM samples cleared." << std::endl;
}

bool VisionGUI::getSamplesEventFlag() {
	return samplesEventFlag;
}

void VisionGUI::__create_frm_split_view() {
	Gtk::VBox *vbox;
	Gtk::Grid *grid;
	Gtk::Label *label;

	vbox = new Gtk::VBox();
	grid = new Gtk::Grid();

	pack_start(fr_splitView, false, false, 5);

	fr_splitView.add(*vbox);
	vbox->pack_start(*grid, false, true, 5);
	vbox->set_halign(Gtk::ALIGN_CENTER);
	vbox->set_valign(Gtk::ALIGN_CENTER);

	fr_splitView.set_label("Split View");

	grid->set_border_width(5);
	grid->set_column_spacing(10);
	grid->set_row_spacing(5);

	label = new Gtk::Label("Select Mode: ");
	grid->attach(*label, 0, 0, 1, 1);
	rb_original_view.set_label("Original");
	grid->attach(rb_original_view, 1, 0, 1, 1);
	rb_split_view.set_label("Split");
	rb_split_view.join_group(rb_original_view);
	grid->attach(rb_split_view, 2, 0, 1, 1);

	rb_original_view.set_state(Gtk::STATE_INSENSITIVE);
	rb_split_view.set_state(Gtk::STATE_INSENSITIVE);

	rb_original_view.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_rb_split_mode_clicked));
	rb_split_view.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_rb_split_mode_clicked));
}

void VisionGUI::__event_rb_split_mode_clicked() {
	isSplitView = rb_split_view.get_active();
}

bool VisionGUI::getIsSplitView() {
	if (disableSplitView) return false;
	else return isSplitView;
}

void VisionGUI::__create_frm_capture() {
	Gtk::VBox *vbox;
	Gtk::Grid *grid;
	Gtk::Label *label;
	Gtk::Frame *frame;

	vbox = new Gtk::VBox();
	grid = new Gtk::Grid();
	frame = new Gtk::Frame();

	pack_start(*frame, false, false, 5);

	frame->add(*vbox);
	vbox->pack_start(*grid, false, true, 5);
	vbox->set_halign(Gtk::ALIGN_CENTER);
	vbox->set_valign(Gtk::ALIGN_CENTER);

	frame->set_label("Video/Image Capture");

	grid->set_border_width(5);
	grid->set_column_spacing(10);
	grid->set_row_spacing(5);

	label = new Gtk::Label("Picture Name:");
	label->set_alignment(1.0);
	grid->attach(*label, 0, 0, 1, 1);
	label = new Gtk::Label("Video Name:");
	label->set_alignment(1.0);
	grid->attach(*label, 0, 1, 1, 1);

	grid->attach(en_picture_name, 1, 0, 1, 1);
	grid->attach(en_video_name, 1, 1, 1, 1);

	bt_save_picture.set_label("Save");
	bt_record_video.set_label("REC");
	grid->attach(bt_save_picture, 2, 0, 1, 1);
	grid->attach(bt_record_video, 2, 1, 1, 1);

	en_video_name.set_state(Gtk::STATE_INSENSITIVE);
	en_picture_name.set_state(Gtk::STATE_INSENSITIVE);
	bt_record_video.set_state(Gtk::STATE_INSENSITIVE);
	bt_save_picture.set_state(Gtk::STATE_INSENSITIVE);

	bt_record_video.signal_pressed().connect(sigc::mem_fun(*this, &VisionGUI::bt_record_video_pressed));
	bt_save_picture.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::bt_save_picture_clicked));
}

void VisionGUI::bt_record_video_pressed() {
	if (vision->isRecording()) {
		vision->finishVideo();
		bt_record_video.set_label("REC");
		en_video_name.set_text("");
		en_video_name.set_state(Gtk::STATE_NORMAL);
	} else {
		std::string name = en_video_name.get_text();

		bt_record_video.set_label("Stop");
		en_video_name.set_state(Gtk::STATE_INSENSITIVE);
		if (name.empty()) {
			vision->startNewVideo(std::to_string(vidIndex++));
			en_video_name.set_text(std::to_string(vidIndex));
		} else {
			vision->startNewVideo(name);
		}
	}
}

void VisionGUI::bt_save_picture_clicked() {
	std::string name = en_picture_name.get_text();
	if (name.empty()) {
		vision->savePicture(std::to_string(picIndex++));
	} else {
		vision->savePicture(name);
	}
	en_picture_name.set_text("");
}

void VisionGUI::__create_frm_cielab() {
	Gtk::VBox *vbox;
	Gtk::Grid *grid;
	Gtk::Label *label;

	vbox = new Gtk::VBox();
	grid = new Gtk::Grid();

	pack_start(fr_CIELAB, false, false, 5);

	fr_CIELAB.add(*vbox);
	vbox->pack_start(*grid, false, true, 5);
	vbox->set_halign(Gtk::ALIGN_CENTER);
	vbox->set_valign(Gtk::ALIGN_CENTER);

	fr_CIELAB.set_label("CIELAB Calibration");

	grid->set_border_width(5);
	grid->set_column_spacing(15);
	grid->set_row_spacing(0);
	// grid->set_column_homogeneous(true);

	bt_LAB_calib.set_label("CIELAB Calib.");
	grid->attach(bt_LAB_calib, 0, 0, 1, 1);

	bt_CIELAB_left.set_label(" < ");
	grid->attach(bt_CIELAB_left, 2, 0, 1, 1);
	CIELAB_label.set_text("Main");
	grid->attach(CIELAB_label, 3, 0, 1, 1);
	bt_CIELAB_right.set_label(" > ");
	grid->attach(bt_CIELAB_right, 4, 0, 1, 1);

	bt_switchMainAdv.set_label("Main <-> Adv");
	grid->attach(bt_switchMainAdv, 6, 0, 1, 1);

	grid = new Gtk::Grid();
	grid->set_border_width(5);
	grid->set_column_spacing(15);
	grid->set_row_spacing(0);
	grid->set_column_homogeneous(true);
	vbox->pack_start(*grid, false, true, 5);

	lb_cieL_min.set_text("Black");
	lb_cieL_min.set_alignment(1.0, 1.0);
	grid->attach(lb_cieL_min, 0, 0, 1, 1);

	HScale_cieL_min.set_digits(0);
	HScale_cieL_min.set_increments(1, 1);
	HScale_cieL_min.set_range(0, 255);
	HScale_cieL_min.set_value_pos(Gtk::POS_TOP);
	HScale_cieL_min.set_draw_value();
	grid->attach(HScale_cieL_min, 1, 0, 2, 1);

	lb_cieL_max.set_text("White");
	lb_cieL_max.set_alignment(1.0, 1.0);
	grid->attach(lb_cieL_max, 3, 0, 1, 1);

	HScale_cieL_max.set_digits(0);
	HScale_cieL_max.set_increments(1, 1);
	HScale_cieL_max.set_range(0, 255);
	HScale_cieL_max.set_value_pos(Gtk::POS_TOP);
	HScale_cieL_max.set_draw_value();
	grid->attach(HScale_cieL_max, 4, 0, 2, 1);

	lb_cieA_min.set_text("Green");
	lb_cieA_min.set_alignment(1.0, 1.0);
	grid->attach(lb_cieA_min, 0, 1, 1, 1);

	HScale_cieA_min.set_digits(0);
	HScale_cieA_min.set_increments(1, 1);
	HScale_cieA_min.set_range(0, 255);
	HScale_cieA_min.set_value_pos(Gtk::POS_TOP);
	HScale_cieA_min.set_draw_value();
	grid->attach(HScale_cieA_min, 1, 1, 2, 1);

	lb_cieA_max.set_text("Red");
	lb_cieA_max.set_alignment(1.0, 1.0);
	grid->attach(lb_cieA_max, 3, 1, 1, 1);

	HScale_cieA_max.set_digits(0);
	HScale_cieA_max.set_increments(1, 1);
	HScale_cieA_max.set_range(0, 255);
	HScale_cieA_max.set_value_pos(Gtk::POS_TOP);
	HScale_cieA_max.set_draw_value();

	grid->attach(HScale_cieA_max, 4, 1, 2, 1);

	lb_cieB_min.set_text("Blue");
	lb_cieB_min.set_alignment(1.0, 1.0);
	grid->attach(lb_cieB_min, 0, 2, 1, 1);

	HScale_cieB_min.set_digits(0);
	HScale_cieB_min.set_increments(1, 1);
	HScale_cieB_min.set_range(0, 255);
	HScale_cieB_min.set_value_pos(Gtk::POS_TOP);
	HScale_cieB_min.set_draw_value();
	grid->attach(HScale_cieB_min, 1, 2, 2, 1);

	lb_cieB_max.set_text("Yellow");
	lb_cieB_max.set_alignment(1.0, 1.0);
	grid->attach(lb_cieB_max, 3, 2, 1, 1);

	HScale_cieB_max.set_digits(0);
	HScale_cieB_max.set_increments(1, 1);
	HScale_cieB_max.set_range(0, 255);
	HScale_cieB_max.set_value_pos(Gtk::POS_TOP);
	HScale_cieB_max.set_draw_value();

	grid->attach(HScale_cieB_max, 4, 2, 2, 1);

	label = new Gtk::Label("Erode:");
	label->set_alignment(1.0, 1.0);
	grid->attach(*label, 0, 3, 1, 1);

	HScale_Erode.set_digits(0);
	HScale_Erode.set_increments(1, 1);
	HScale_Erode.set_range(0, 50);
	HScale_Erode.set_value_pos(Gtk::POS_TOP);
	HScale_Erode.set_draw_value();
	grid->attach(HScale_Erode, 1, 3, 2, 1);

	label = new Gtk::Label("Dilate:");
	label->set_alignment(1.0, 1.0);
	grid->attach(*label, 3, 3, 1, 1);

	HScale_Dilate.set_digits(0);
	HScale_Dilate.set_increments(1, 1);
	HScale_Dilate.set_range(0, 50);
	HScale_Dilate.set_value_pos(Gtk::POS_TOP);
	HScale_Dilate.set_draw_value();
	grid->attach(HScale_Dilate, 4, 3, 2, 1);

	label = new Gtk::Label("Blur:");
	label->set_alignment(1.0, 1.0);
	grid->attach(*label, 0, 4, 1, 1);

	HScale_Blur.set_digits(0);
	HScale_Blur.set_increments(1, 1);
	HScale_Blur.set_range(0, 9);
	HScale_Blur.set_value_pos(Gtk::POS_TOP);
	HScale_Blur.set_draw_value();
	grid->attach(HScale_Blur, 1, 4, 2, 1);

	label = new Gtk::Label("Amin:");
	label->set_alignment(1.0, 1.0);
	grid->attach(*label, 3, 4, 1, 1);

	HScale_Amin.set_digits(0);
	HScale_Amin.set_increments(1, 1);
	HScale_Amin.set_range(0, 200);
	HScale_Amin.set_value_pos(Gtk::POS_TOP);
	HScale_Amin.set_draw_value();

	grid->attach(HScale_Amin, 4, 4, 2, 1);

	bt_LAB_calib.set_state(Gtk::STATE_INSENSITIVE);
	bt_switchMainAdv.set_state(Gtk::STATE_INSENSITIVE);
	HScale_cieL_min.set_state(Gtk::STATE_INSENSITIVE);
	HScale_cieA_min.set_state(Gtk::STATE_INSENSITIVE);
	HScale_cieB_min.set_state(Gtk::STATE_INSENSITIVE);
	HScale_cieL_max.set_state(Gtk::STATE_INSENSITIVE);
	HScale_cieA_max.set_state(Gtk::STATE_INSENSITIVE);
	HScale_cieB_max.set_state(Gtk::STATE_INSENSITIVE);
	HScale_Dilate.set_state(Gtk::STATE_INSENSITIVE);
	HScale_Erode.set_state(Gtk::STATE_INSENSITIVE);
	HScale_Blur.set_state(Gtk::STATE_INSENSITIVE);
	HScale_Amin.set_state(Gtk::STATE_INSENSITIVE);
	bt_CIELAB_left.set_state(Gtk::STATE_INSENSITIVE);
	bt_CIELAB_right.set_state(Gtk::STATE_INSENSITIVE);

	bt_LAB_calib.signal_pressed().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_CIELAB_calib_pressed));
	bt_CIELAB_right.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_right_CIELAB_clicked));
	bt_CIELAB_left.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_left_CIELAB_clicked));
	bt_switchMainAdv.signal_clicked().connect(sigc::mem_fun(*this, &VisionGUI::__event_bt_switchMainAdv_clicked));
	HScale_cieL_min.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_cieL_min_value_changed));
	HScale_cieL_max.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_cieL_max_value_changed));
	HScale_cieA_min.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_cieA_min_value_changed));
	HScale_cieA_max.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_cieA_max_value_changed));
	HScale_cieB_min.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_cieB_min_value_changed));
	HScale_cieB_max.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_cieB_max_value_changed));
	HScale_Erode.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_Erode_value_changed));
	HScale_Dilate.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_Dilate_value_changed));
	HScale_Blur.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_Blur_value_changed));
	HScale_Amin.signal_value_changed().connect(sigc::mem_fun(*this, &VisionGUI::HScale_Amin_value_changed));
}

void VisionGUI::__event_bt_switchMainAdv_clicked() {
	vision->switchMainWithAdv();
	HScale_cieL_min.set_value(vision->getCIE_L(Img_id, 0));
	HScale_cieL_max.set_value(vision->getCIE_L(Img_id, 1));
	HScale_cieA_min.set_value(vision->getCIE_A(Img_id, 0));
	HScale_cieA_max.set_value(vision->getCIE_A(Img_id, 1));
	HScale_cieB_min.set_value(vision->getCIE_B(Img_id, 0));
	HScale_cieB_max.set_value(vision->getCIE_B(Img_id, 1));
	HScale_Amin.set_value(vision->getAmin(Img_id));
	HScale_Blur.set_value(vision->getBlur(Img_id));
	HScale_Erode.set_value(vision->getErode(Img_id));
	HScale_Dilate.set_value(vision->getDilate(Img_id));
}

void VisionGUI::update_vision_hscale_values() {
	HScale_Erode.set_value(vision->getErode(Img_id));
	HScale_Dilate.set_value(vision->getDilate(Img_id));
	HScale_Blur.set_value(vision->getBlur(Img_id));
	HScale_Amin.set_value(vision->getAmin(Img_id));

	HScale_cieL_min.set_value(vision->getCIE_L(Img_id, 0));
	HScale_cieL_max.set_value(vision->getCIE_L(Img_id, 1));
	HScale_cieA_min.set_value(vision->getCIE_A(Img_id, 0));
	HScale_cieA_max.set_value(vision->getCIE_A(Img_id, 1));
	HScale_cieB_min.set_value(vision->getCIE_B(Img_id, 0));
	HScale_cieB_max.set_value(vision->getCIE_B(Img_id, 1));
}

void VisionGUI::HScale_cieL_min_value_changed() {
	vision->setCIE_L(Img_id, 0, static_cast<int>(HScale_cieL_min.get_value()));
}

void VisionGUI::HScale_cieA_min_value_changed() {
	vision->setCIE_A(Img_id, 0, static_cast<int>(HScale_cieA_min.get_value()));
}

void VisionGUI::HScale_cieB_min_value_changed() {
	vision->setCIE_B(Img_id, 0, static_cast<int>(HScale_cieB_min.get_value()));
}

void VisionGUI::HScale_cieL_max_value_changed() {
	vision->setCIE_L(Img_id, 1, static_cast<int>(HScale_cieL_max.get_value()));
}

void VisionGUI::HScale_cieA_max_value_changed() {
	vision->setCIE_A(Img_id, 1, static_cast<int>(HScale_cieA_max.get_value()));
}

void VisionGUI::HScale_cieB_max_value_changed() {
	vision->setCIE_B(Img_id, 1, static_cast<int>(HScale_cieB_max.get_value()));
}

void VisionGUI::HScale_Amin_value_changed() {
	vision->setAmin(Img_id, static_cast<int>(HScale_Amin.get_value()));
}

void VisionGUI::HScale_Dilate_value_changed() {

	if (HScale_Dilate.get_value() < 0) {
		vision->setDilate(Img_id, 0);
	} else {
		vision->setDilate(Img_id, static_cast<int>(HScale_Dilate.get_value()));
	}
}

void VisionGUI::HScale_Erode_value_changed() {

	if (HScale_Erode.get_value() < 0) {
		vision->setErode(Img_id, 0);
	} else {
		vision->setErode(Img_id, static_cast<int>(HScale_Erode.get_value()));
	}
}

void VisionGUI::HScale_Blur_value_changed() {
	vision->setBlur(Img_id, static_cast<int>(HScale_Blur.get_value()));
}

void VisionGUI::__event_bt_CIELAB_calib_pressed() {

	if (CIELAB_calib_event_flag) {
		CIELAB_calib_event_flag = false;
		// VisionGUI::__event_auto_save();
		HScale_cieL_min.set_state(Gtk::STATE_INSENSITIVE);
		HScale_cieA_min.set_state(Gtk::STATE_INSENSITIVE);
		HScale_cieB_min.set_state(Gtk::STATE_INSENSITIVE);
		HScale_cieL_max.set_state(Gtk::STATE_INSENSITIVE);
		HScale_cieA_max.set_state(Gtk::STATE_INSENSITIVE);
		HScale_cieB_max.set_state(Gtk::STATE_INSENSITIVE);
		HScale_Dilate.set_state(Gtk::STATE_INSENSITIVE);
		HScale_Erode.set_state(Gtk::STATE_INSENSITIVE);
		HScale_Blur.set_state(Gtk::STATE_INSENSITIVE);
		HScale_Amin.set_state(Gtk::STATE_INSENSITIVE);
		bt_CIELAB_right.set_state(Gtk::STATE_INSENSITIVE);
		bt_CIELAB_left.set_state(Gtk::STATE_INSENSITIVE);
		bt_switchMainAdv.set_state(Gtk::STATE_INSENSITIVE);
	} else {
		CIELAB_calib_event_flag = true;
		HScale_cieL_min.set_state(Gtk::STATE_NORMAL);
		HScale_cieA_min.set_state(Gtk::STATE_NORMAL);
		HScale_cieB_min.set_state(Gtk::STATE_NORMAL);
		HScale_cieL_max.set_state(Gtk::STATE_NORMAL);
		HScale_cieA_max.set_state(Gtk::STATE_NORMAL);
		HScale_cieB_max.set_state(Gtk::STATE_NORMAL);
		HScale_Dilate.set_state(Gtk::STATE_NORMAL);
		HScale_Erode.set_state(Gtk::STATE_NORMAL);
		HScale_Blur.set_state(Gtk::STATE_NORMAL);
		HScale_Amin.set_state(Gtk::STATE_NORMAL);
		bt_CIELAB_right.set_state(Gtk::STATE_NORMAL);
		bt_CIELAB_left.set_state(Gtk::STATE_NORMAL);
		bt_switchMainAdv.set_state(Gtk::STATE_NORMAL);
	}
}

void VisionGUI::selectFrame(int sector) {
	switch (sector) {
		case 0:
			rb_original_view.set_active(true);
			__event_rb_split_mode_clicked();
			break;

		case 1:
			Img_id = 3;
			rb_original_view.set_active(true);
			__event_rb_split_mode_clicked();
			__event_bt_right_CIELAB_clicked();
			if (!bt_LAB_calib.get_active()) {
				bt_LAB_calib.set_active(true);
				__event_bt_CIELAB_calib_pressed();
			}
			break;
		case 2:
			Img_id = 0;
			rb_original_view.set_active(true);
			__event_rb_split_mode_clicked();
			__event_bt_right_CIELAB_clicked();
			if (!bt_LAB_calib.get_active()) {
				bt_LAB_calib.set_active(true);
				__event_bt_CIELAB_calib_pressed();
			}
			break;
		case 3:
			Img_id = 1;
			rb_original_view.set_active(true);
			__event_rb_split_mode_clicked();
			__event_bt_right_CIELAB_clicked();
			if (!bt_LAB_calib.get_active()) {
				bt_LAB_calib.set_active(true);
				__event_bt_CIELAB_calib_pressed();
			}
			break;
		default:
			break;
	}
}

void VisionGUI::__event_bt_right_CIELAB_clicked() {

	if (Img_id == 3)
		Img_id = 0;
	else
		Img_id++;

	HScale_cieL_min.set_value(vision->getCIE_L(Img_id, 0));
	HScale_cieL_max.set_value(vision->getCIE_L(Img_id, 1));

	HScale_cieA_min.set_value(vision->getCIE_A(Img_id, 0));
	HScale_cieA_max.set_value(vision->getCIE_A(Img_id, 1));

	HScale_cieB_min.set_value(vision->getCIE_B(Img_id, 0));
	HScale_cieB_max.set_value(vision->getCIE_B(Img_id, 1));


	HScale_Dilate.set_value(vision->getDilate(Img_id));
	HScale_Erode.set_value(vision->getErode(Img_id));

	HScale_Blur.set_value(vision->getBlur(Img_id));
	HScale_Amin.set_value(vision->getAmin(Img_id));

	switch (Img_id) {
		case 0:
			CIELAB_label.set_text("Main");
			break;
		case 1:
			CIELAB_label.set_text("Green");
			break;
		case 2:
			CIELAB_label.set_text("Ball");
			break;
		case 3:
			CIELAB_label.set_text("Opp.");
			break;
		default:break;
	}
}

void VisionGUI::__event_bt_left_CIELAB_clicked() {

	if (Img_id == 0)
		Img_id = 3;
	else
		Img_id--;

	HScale_cieL_min.set_value(vision->getCIE_L(Img_id, 0));
	HScale_cieL_max.set_value(vision->getCIE_L(Img_id, 1));

	HScale_cieA_min.set_value(vision->getCIE_A(Img_id, 0));
	HScale_cieA_max.set_value(vision->getCIE_A(Img_id, 1));

	HScale_cieB_min.set_value(vision->getCIE_B(Img_id, 0));
	HScale_cieB_max.set_value(vision->getCIE_B(Img_id, 1));


	HScale_Dilate.set_value(vision->getDilate(Img_id));
	HScale_Erode.set_value(vision->getErode(Img_id));

	HScale_Blur.set_value(vision->getBlur(Img_id));
	HScale_Amin.set_value(vision->getAmin(Img_id));

	switch (Img_id) {
		case 0:
			CIELAB_label.set_text("Main");
			break;
		case 1:
			CIELAB_label.set_text("Green");
			break;
		case 2:
			CIELAB_label.set_text("Ball");
			break;
		case 3:
			CIELAB_label.set_text("Opp.");
			break;
		default:break;
	}
}

void VisionGUI::event_draw_info_checkbox_signal_clicked() {
	draw_info_flag = !draw_info_flag;
}

void VisionGUI::incrementSamples() {
	totalSamples++;
	std::string text = "Pop (" + std::to_string(totalSamples) + ")";
	bt_popSample.set_label(text);
}

void VisionGUI::decrementSamples() {
	if (totalSamples - 1 < 0) totalSamples = 0;
	else totalSamples--;
	std::string text = "Pop (" + std::to_string(totalSamples) + ")";
	bt_popSample.set_label(text);
}

void VisionGUI::setFrameSize(int inWidth, int inHeight) {
	vision->setFrameSize(inWidth, inHeight);
}

unsigned long VisionGUI::getGMMColorIndex() {
	return colorIndex;
}

bool VisionGUI::getIsCIELAB() {
	return isCIELAB;
}

VisionGUI::VisionGUI() :
		CIELAB_calib_event_flag(false), Img_id(0),
		vidIndex(0), picIndex(0), samplesEventFlag(false),
		totalSamples(0), gaussiansFrame_flag(false),
		finalFrame_flag(false), thresholdFrame_flag(false),
		colorIndex(0), isCIELAB(true), isSplitView(false),
		disableSplitView(false), draw_info_flag(false) {

	vision = new Vision(640, 480);
	gmm = new GMM(640, 480);

	//__create_frm_calib_mode();
	__create_frm_capture();
	__create_frm_drawing_options();
	__create_frm_split_view();
	__create_frm_cielab();
	//__create_frm_gmm();
}

VisionGUI::~VisionGUI() {
	if (vision->isRecording()) vision->finishVideo();
}
