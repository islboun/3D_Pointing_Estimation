#ifndef LIDAR_SEG_H
#define LIDAR_SEG_H


#include <pcl/point_types.h>
//#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <vector>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <chrono> 
#include <ctime>
#include <vector>
#include "util.h"
#include "RGB_HSV.h"



using namespace std;
using namespace pcl;


//Kinect Translation and Rotation matrix
const float RDepthAxisToRgbAxis[9] =
{

0.99970039, -0.01202495, -0.02131971,
       0.0127593 ,  0.99931807,  0.03464969,
       0.02088851, -0.03491134,  0.99917209,
};



const float TDepthAxisToRgbAxis[3] =
{
       -0.01193532, 0.00546306, -0.00044406,
};


class lidar_seg{

public:
std::map<vector<int>, int> cloudImageMapping;
int cluster = 0;

lidar_seg(double pan_res, double tilt_res); //constructors
void get_cons(double pan_res, double tilt_res);
void set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  in_cloud );
void set_cloud( cv::Mat depth);
void set_params(double pan_eps, double tilt_eps, double dist_eps, int min_points, int cluster_size);
void segment();
void segment(cv::Mat depth, cv::Mat rgb);
void segment(cv::Mat depth, cv::Mat rgb, vector<cv::Rect> hand_loc);
void segment(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl, cv::Mat rgb, vector<cv::Rect> hand_loc);
void segment(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl, cv::Mat rgb, vector<Eigen::MatrixXf> normal,float dist, vector<vector<pcl::PointXYZRGB>> hand_center, cv::Rect human);
void segment(cv::Mat depth, cv::Mat rgb, vector<Eigen::MatrixXf> normal,float dist, vector<vector<pcl::PointXYZRGB>> hand_center, cv::Rect human);
void to_sphere(cv::Mat depth, cv::Mat rgb);
void take_colored_cloud(pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud);
vector<int> get_clusters();
int get_cluster_no();
pcl::PointCloud<pcl::PointXYZRGB> get_cloud();
int get_total_cluster();
private:


void get_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  in_cloud);

void get_params(double pan_eps, double tilt_eps, double dist_eps, int min_points, int cluster_size);
void to_sphere() ;
void to_sphere(cv::Mat depth, cv::Mat rgb, vector<cv::Rect> hand_loc);
void to_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl, cv::Mat rgb);
void to_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl, cv::Mat rgb, vector<cv::Rect> hand_loc);
void dist(int index, int core_index, std::vector<int> &result_neigh);
void boundingbox(); 
vector<int> region_query(int iterator,bool core);
int get_cluster_no(const vector<int>& leftTop, const vector<int>& rightBottom);


vector<vector<int> > index_vec;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

pcl::PointCloud<pcl::PointXYZRGB> sphere_cloud ;


double pan_resolution, tilt_resolution, pan_epsilon, tilt_epsilon, dist_epsilon;
int seg_min ;
int min_cluster_size;
vector<bool> visited;
vector<int> clusters;
int pan_direction, tilt_direction;
int min_cluster_var;
int total_pan;
int total_tilt;

int total_cluster;

Eigen::MatrixXd range_image;
Eigen::Matrix4f DepthtoRGBMat;

vector<hsv> colors;
double angle_range = M_PI / 6;

int min_hand_point = 20;
float min_hand_dist = 0.55;

//distance parameters
float elipsoid_thresh1 = 1.4;
float elipsoid_thresh2 = 1.5;
float elipsoid_thresh3 = 1.7;
float color_thresh1 = 0.4;
float color_thresh2 = 0.5;

//float elipsoid_thresh1 = 0.01;
//float elipsoid_thresh2 = 0.3;
//float elipsoid_thresh3 = 0.5;
//float color_thresh1 = 0.1;
//float color_thresh2 = 0.5;
};





#endif
