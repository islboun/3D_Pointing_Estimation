#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
//#include <point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>


#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/highgui.hpp>



using namespace std;
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
using namespace std;
using namespace pcl;





class ground_seg {


public:
	ground_seg(int in_num_iter, int in_num_lpr, double in_th_seeds, double in_th_dist);
	void segment(pcl::PointCloud<pcl::PointXYZ>  in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& not_ground_cloud);





private:


	int num_iter; //number of iteration to estimate plane
	int num_lpr; //number of seed points
	double th_seeds; // The initial height (the height of the sensor)
	double th_dist; // The confidence threshold for the test ground plane
	float d; // distance with the threshold
	Eigen::MatrixXf normal;


	float estimate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_points);
	void extract_initial_seeds(pcl::PointCloud<pcl::PointXYZ> p_sorted, pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_seeds);


};


