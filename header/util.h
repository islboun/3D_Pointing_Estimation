#pragma once
#include <vector>
#include <pcl/common/common.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>



//Camera parameters
#define PAN_RANGE	70.6//57.5  //Camera Horizontal FOV
#define TILT_RANGE  60.0//43.5  //Camera Vertical FOV

#define RGB_FOCAL_X     525.0
#define RGB_FOCAL_Y     525.0
#define RGB_C_X         319.5
#define RGB_C_Y         239.5

#define DEPTH_FOCAL_X     575.8157348632812
#define DEPTH_FOCAL_Y     575.8157348632812
#define DEPTH_C_X         314.5
#define DEPTH_C_Y         235.5
#define min_range  0.5		//min distance to camera
#define max_range  5.5		//max distance to camera

// Depth segmentation parameters
#define  depth_pan_eps		1.5
#define  depth_tilt_eps		1.5
#define  depth_dist_eps		0.05
#define  depth_min_neigh	10
#define  depth_min_points	300

//Ground segmentation parameters
#define ground_num_iter			10		//number of iteration to estimate plane
#define ground_num_lpr				2000	//number of seed points
#define ground_th_seeds			0.2		// The threshold for seeds
#define ground_th_dist				0.1
#define ground_initial_height		0.45	// Initial sensor height from the ground plane


pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat depthMat);
cv::Mat CloudtoMat(pcl::PointCloud<pcl::PointXYZRGB> cloud, int height, int width);
cv::Mat GroundCloudToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud, int width, int height);
cv::Point get2Dcoord(pcl::PointXYZRGB cloud_point);
pcl::PointCloud<pcl::PointXYZ> pointXYZRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB> rgbcloud);
