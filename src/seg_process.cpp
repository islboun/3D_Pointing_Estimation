/*

Author: Mirhan Ürkmez
Date:17/12/2020
Description: Estimation of 3D pointing direction codes from hand point cloud data.  

*/


#include "lidar_seg.h"
#include "RGB_HSV.h"
#include "seg_process.h"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

using namespace std;
using namespace cv;
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

seg_process::seg_process(int nu_planes1,int neigh_dist1, int nu_planes2,int neigh_dist2)
{
	this->nu_planes1  = nu_planes1;
	this->neigh_dist1 = neigh_dist1;
	this->nu_planes2  = nu_planes2;
	this->neigh_dist2 = neigh_dist2;
}
void seg_process::set_data(vector<int> clust, std::map<vector<int>, int> map, pcl::PointCloud<pcl::PointXYZRGB> cloud,int cluster)
{
	this->clusters=clust;
	this->cloudImageMapping=map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    *temp = cloud;
    this->cloud = temp;
    this->cluster=cluster;
}

int seg_process::get_pointing_end(Eigen::MatrixXf normal, pcl::PointXYZRGB hand_center, int human_cluster)
{
    pcl::PointXYZRGB human_center;
    int human_seg_count = 0;
	
	//find human center coordinates
    for (int i = 0; i < clusters.size(); i++)
    {
        if (clusters[i] == human_cluster)
        {
            human_center.x += cloud->points[i].x;
            human_center.y += cloud->points[i].y;
            human_center.z += cloud->points[i].z;
            human_seg_count++;
        }
    }
    human_center.x /= human_seg_count;
    human_center.y /= human_seg_count;
    human_center.z /= human_seg_count;
    float human_x_vec = hand_center.x - human_center.x;
    float human_y_vec = hand_center.y - human_center.y;
    float human_z_vec = hand_center.z - human_center.z;
    if ((human_x_vec * normal(0) + human_y_vec * normal(1) + human_z_vec * normal(2)) > 0)
        return 1;
    else
        return 0;
    
    
}

Eigen::MatrixXf seg_process::get_hand_orientation( vector<int> leftTop, vector<int> rightBottom,float &dist, pcl::PointXYZRGB &hand_center1, pcl::PointXYZRGB& hand_center2)
{
    //hand cluster
    int hand_cluster_no = get_cluster_no(leftTop,rightBottom);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand3D(new pcl::PointCloud<PointXYZRGB>);
    create_hand_cloud(hand3D,leftTop, rightBottom, hand_cluster_no);

    Eigen::MatrixXf result(3,1);
    int counter = 0;
    while (!get_pointing_dir(result, hand3D, hand_center1,hand_center2, dist, nu_planes1, neigh_dist1))
    {
        //if direction is not found reduce the number of planes or distance threshold
        if (counter % 2 == 0)
            nu_planes1--;
        if (counter % 2 == 1)
            neigh_dist1--;
        if (nu_planes1 <= 1 )
        {
            break;
        }
           
        counter++;
    }
    
    return result;

}

int seg_process::get_cluster_no(const vector<int> &leftTop, const vector<int> &rightBottom)
{
    int centerx = (leftTop[0] + rightBottom[0]) / 2;
    int centery = (leftTop[1] + rightBottom[1]) / 2;
    int center_count = 0;
    vector<float> segment_count(cluster+2);
    vector<int> coord(2);
    for (int i = leftTop[0]; i < rightBottom[0]; i++)
    {
        for (int j = leftTop[1]; j < rightBottom[1]; j++)
        {
            coord[0] = i;
            coord[1] = j;
            auto it = cloudImageMapping.find(coord);
            if (it != cloudImageMapping.end())
            {
				// find the most dominant object in the bounding box
                if (clusters[it->second] > 0  && cloud->points[it->second].z> min_range && cloud->points[it->second].z < max_range)
                {
                    segment_count[clusters[it->second]] += 1/ cloud->points[it->second].z;

                }
                   
            }

        }
    }
    int hand_cluster = std::distance(segment_count.begin(), std::max_element(segment_count.begin(), segment_count.end()));
    return  hand_cluster;
}

pcl::PointXYZRGB seg_process::get_hand_center(int centerx, int centery)
{
    vector<int> coord{ 0,0 };
    pcl::PointXYZRGB hand_center;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            coord[0] = centerx + i;
            coord[1] = centery + j;
            auto it = cloudImageMapping.find(coord);
            if (it != cloudImageMapping.end())
            {
                hand_center = cloud->points[it->second];
                return hand_center;
            }

        }
    }
}

void seg_process::create_hand_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, const vector<int>& leftTop, const vector<int>& rightBottom, int hand_cluster)
{

    vector<int> coord{ 0,0 };
    for (int i = leftTop[0]; i < rightBottom[0]; i++)
    {
        for (int j = leftTop[1]; j < rightBottom[1]; j++)
        {
            coord[0] = i;
            coord[1] = j;
            auto it = cloudImageMapping.find(coord);

            if (it != cloudImageMapping.end())
            {
                if (clusters[it->second] < 0)
                    continue;
                if (clusters[it->second] == hand_cluster && cloud->points[it->second].z>min_range)
                {
                    hand_cloud->push_back(cloud->points[it->second]);
                }
                    
            }

        }
    }
    
 
}


pcl::PointCloud<pcl::PointXYZRGB> seg_process::get_crowded_plane(Eigen::MatrixXf normal, pcl::PointCloud<pcl::PointXYZRGB> hand_cloud,int division,int threshold_divider, pcl::PointCloud<pcl::PointXYZRGB>::Ptr big_hand_cloud)
{
    float min_dist = 500;
    float max_dist = -500;
    float dist = 0;
    pcl::PointXYZRGB plane_point;

    //find outermost distances in normal direction
    for (int i = 0; i < hand_cloud.size(); i++)
    {
        dist = normal(0) * hand_cloud.points[i].x + normal(1) * hand_cloud.points[i].y +normal(2) * hand_cloud.points[i].z;
        if (dist > max_dist)
        {
            max_dist = dist;
        }
            
        if (dist < min_dist)
        {
            min_dist = dist;
        }
            
    }
    float increment = (max_dist - min_dist) / (division);
    float threshold =  increment / threshold_divider;
    int max_count = 0;
    int max_plane;
    
    //find neighborhood sizes of planes
    for (int i = 0; i < division-1; i++)
    {
        int count = 0;

        for (int j = 0; j < hand_cloud.size(); j++)
        {
            dist = normal(0) * hand_cloud.points[j].x + normal(1) * hand_cloud.points[j].y + normal(2) * hand_cloud.points[j].z - (min_dist + increment * (i + 1));
            if (fabs(dist) < threshold)
                count++;// 1 / (fabs(dist) + 0.001);

        }
        if (count > max_count)
        {
            max_count = count;
            max_plane = i;
        }
    }

    //get plane with biggest neighborhood
    pcl::PointCloud<pcl::PointXYZRGB> hand_plane;
    int flag = 0;
    for (int j = 0; j < hand_cloud.size(); j++)
    {
        dist = normal(0) * hand_cloud.points[j].x + normal(1) * hand_cloud.points[j].y + normal(2) * hand_cloud.points[j].z - (min_dist + increment * (max_plane + 1));
        if (fabs(dist) < threshold)
        {
            hand_plane.push_back(hand_cloud.points[j]);
        }

    }
    return hand_plane;
}

void seg_process::draw_truth(Eigen::Matrix3f normal, pcl::PointXYZRGB hand_center, pcl::PointCloud<pcl::PointXYZRGB>& target_cloud)
{
    pcl::PointXYZRGB point;
    for (int i = -10000; i < 10000; i++)
    {
        point.x = hand_center.x + i * normal(0) * 0.01;
        point.y = hand_center.y + i * normal(1) * 0.01;
        point.z = hand_center.z + i * normal(2) * 0.01;
        point.r = 255;
        point.g = 0;
        point.b = 0;

        target_cloud.push_back(point);
    }
}


int seg_process::get_pointing_dir(Eigen::MatrixXf &result,pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, pcl::PointXYZRGB &direction_center1, pcl::PointXYZRGB& direction_center2, float &dist,int nu_planes, int neigh_dist)
{
    Eigen::MatrixXf normal;
    Eigen::MatrixXf normal2;
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*hand_cloud, cov, pc_mean); // computation of the covariance and mean
    JacobiSVD<MatrixXf> svd(cov, Eigen::ComputeFullU); //SVD for detecting surface normal

   //use the least singular vector as normal
    double singular = svd.singularValues()[0];
    int minInd = 0;
    //get smallest eigenvalue and corresponding eigenvector
    for (int i = 1; i < svd.singularValues().size(); i++)
    {
        if (svd.singularValues()[i] < singular)
        {
            singular = svd.singularValues()[i];
            minInd = i;
        }

    }

    //first plane and its neighborhood
    normal = svd.matrixU().col(minInd); //Normal vector
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_plane(new pcl::PointCloud<PointXYZRGB>);
    *hand_plane = get_crowded_plane(normal, *hand_cloud, nu_planes, neigh_dist, hand_cloud);

    //get second smallest eigenvalue and corresponding eigenvector
    singular = 100000;
    int midInd;
    for (int i = 0; i < svd.singularValues().size(); i++)
    {
        if (svd.singularValues()[i] < singular && i!=minInd)
        {
            singular = svd.singularValues()[i];
            midInd = i;
        }

    }
    
    //second plane and its neighborhood
    normal2 = svd.matrixU().col(midInd);
    pcl::PointCloud<pcl::PointXYZRGB> final_cloud = get_crowded_plane(normal2, *hand_plane, nu_planes2, neigh_dist2, hand_cloud);

    //check miminum point criteria
    if (final_cloud.size() < min_hand_point)
        return 0;

    //compute orientation
    pcl::computeMeanAndCovarianceMatrix(final_cloud, cov, pc_mean); // computation of the covariance and mean
    JacobiSVD<MatrixXf> svd2(cov, Eigen::ComputeFullU); //SVD for detecting surface normal

    //get biggest eigenvalue and correcponding direction
    singular = svd2.singularValues()[0];
    int maxInd = 0;
    for (int i = 1; i < svd2.singularValues().size(); i++)
    {
        if (svd2.singularValues()[i] > singular)
        {
            singular = svd2.singularValues()[i];
            maxInd = i;
        }

    }
    result = (svd2.matrixU().col(maxInd));

    //get two outermost hand points along the direction line
    float max_dist = -1000;
    float min_dist = 1000;
    int min_point_ind = 0;
    int max_point_ind = 0;
    for (int i = 0; i < final_cloud.size(); i++)
    {
        dist = result(0) * final_cloud.points[i].x + result(1) * final_cloud.points[i].y + result(2) * final_cloud.points[i].z;
        if (dist > max_dist)
        {
            max_dist = dist;
            max_point_ind = i;
        }

        if (dist < min_dist)
        {
            min_dist = dist;
            min_point_ind = i;
        }

    }
	//two outermost points int he pointing direction
    direction_center1 = final_cloud[min_point_ind];
    direction_center2 = final_cloud[max_point_ind];
    dist = -(result(0) * direction_center1.x + result(1) * direction_center1.y + result(2) * direction_center1.z);
    
    return 1;

}


void seg_process::draw_dir(Eigen::MatrixXf normal, pcl::PointXYZRGB hand_center, pcl::PointCloud<pcl::PointXYZRGB>& target_cloud)
{
    pcl::PointXYZRGB point;
    for (int i = -10000; i < 10000; i++)
    {
        point.x = hand_center.x + i * normal(0) * 0.001;
        point.y = hand_center.y + i * normal(1) * 0.001;
        point.z = hand_center.z + i * normal(2) * 0.001;
        point.r = 255;
        point.g = 255;
        point.b = 255;

        target_cloud.push_back(point);
    }
}
