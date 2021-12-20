#pragma once
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <thread>

class hand_human_detector
{
public:
	hand_human_detector(std::string cfg_file, std::string weights_file);
	void run(cv::Mat& frame, std::vector<cv::Rect>& hands, cv::Rect& human);
private:
	void getRects(cv::Mat& frame, const std::vector<cv::Mat>& outs, std::vector<cv::Rect>& hands, cv::Rect& human);
	// get output layers
	std::vector<std::string> getOutputsNames(const cv::dnn::Net& net);
	std::string cfg_file = "hand.cfg";
	std::string weights_file = "hand_last.weights";
	int blob_width = 416;
	int blob_height = 416;
	float conf_threshold = 0.3;
	float nms_thresh = 0.4;
};
// remove unnecessary bounding boxes

