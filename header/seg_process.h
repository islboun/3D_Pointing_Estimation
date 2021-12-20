#ifndef SEG_PROCESS_H
#define SEG_PROCESS_H_H


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





class seg_process
{
	public:
	seg_process(int nu_planes1,int neigh_dist1, int nu_planes2,int neigh_dist2);
	Eigen::MatrixXf get_hand_orientation( vector<int> leftTop, vector<int> rightBottom,float &dist, pcl::PointXYZRGB &hand_center1, pcl::PointXYZRGB& hand_center2);
	void draw_dir(Eigen::MatrixXf normal, pcl::PointXYZRGB hand_center, pcl::PointCloud<pcl::PointXYZRGB>& target_cloud);
	void set_data(vector<int> clust, std::map<vector<int>, int> map, pcl::PointCloud<pcl::PointXYZRGB> cloud,int cluster);
	void draw_truth(Eigen::Matrix3f normal, pcl::PointXYZRGB hand_center, pcl::PointCloud<pcl::PointXYZRGB>& target_cloud);
	private:
	int min_hand_point = 20;
	//number of planes and dist parameters in orientation algorithm
	int nu_planes1 ;
    int neigh_dist1 ;
    int nu_planes2 ;
    int neigh_dist2 ;
	int cluster;
	vector<int> clusters;
	std::map<vector<int>, int> cloudImageMapping;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	
	int get_pointing_end(Eigen::MatrixXf normal, pcl::PointXYZRGB hand_center, int human_cluster);
	int get_cluster_no(const vector<int> &leftTop, const vector<int> &rightBottom);
	pcl::PointXYZRGB get_hand_center(int centerx, int centery);
	void create_hand_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, const vector<int>& leftTop, const vector<int>& rightBottom, int hand_cluster);
	pcl::PointCloud<pcl::PointXYZRGB> get_crowded_plane(Eigen::MatrixXf normal, pcl::PointCloud<pcl::PointXYZRGB> hand_cloud,int division,int threshold_divider, pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_hand_cloud);
	int get_pointing_dir(Eigen::MatrixXf &result,pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, pcl::PointXYZRGB &direction_center1, pcl::PointXYZRGB& direction_center2, float &dist,int nu_planes, int neigh_dist);
};

#endif