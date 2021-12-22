/*

Authors: Meriç Durukan & Mirhan Ürkmez
Date:15/01/2021
Description: Segmentation of RGB-D data using pointcloud library is performed.

*/



#include "lidar_seg.h"
#include "RGB_HSV.h"
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

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

lidar_seg::lidar_seg(double pan_res, double tilt_res){
  get_cons(pan_res, tilt_res);
  //Camera rotation matrix
  for (int i = 0; i < 3; i++)
  {
      for (int j = 0; j < 3; j++)
      {
          this->DepthtoRGBMat(i,j)= RDepthAxisToRgbAxis[3*i+j];
      }
  }
  
  //Camera translation matrix
  for (int i = 0; i < 3; i++)
  {
      this->DepthtoRGBMat(i,3) = TDepthAxisToRgbAxis[i];
  }
}
void lidar_seg::get_cons( double pan_res, double tilt_res){
  this->pan_resolution = pan_res;
  this->tilt_resolution = tilt_res;
  index_vec.clear();
}

void lidar_seg::set_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  in_cloud ){
  get_cloud(in_cloud);
}

void lidar_seg::set_cloud( cv::Mat depth) {

    visited.clear();
    clusters.clear();
    visited.resize(depth.rows*depth.cols);
    clusters.resize(depth.rows * depth.cols);
    std::fill(visited.begin(), visited.end(), false); //visited indexes
    std::fill(clusters.begin(), clusters.end(), 0); //clusters


    total_pan = round(360 / this->pan_resolution) + 10; //10 is added for the safety of the code
    total_tilt = round(360 / this->tilt_resolution) + 10; // 10 is added for the safety of the code
    range_image = Eigen::MatrixXd::Zero(total_pan, total_tilt); //range iimage data collection Initializion
}

void lidar_seg::get_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr  in_cloud){
  copyPointCloud(*in_cloud, *cloud);
  //cloud = in_cloud; //take input cloud
  visited.clear();
  clusters.clear();

  visited.resize(cloud->size());
  clusters.resize(cloud->size());
  std::fill(visited.begin(), visited.end(), false); //visited indexes
  std::fill(clusters.begin(), clusters.end(), 0); //clusters
  //  cout<<"size:"<<cloud->size()<<endl;

  total_pan = round(360 /this->pan_resolution); //10 is added for the safety of the code
  total_tilt = round(180/this->tilt_resolution); // 10 is added for the safety of the code
  range_image = Eigen::MatrixXd::Zero(total_pan,total_tilt); //range iimage data collection Initializion
  //cout<<range_image<<endl;

}


void lidar_seg::set_params(double pan_eps, double tilt_eps, double dist_eps, int min_points, int cluster_size){

  get_params(pan_eps,tilt_eps,dist_eps, min_points, cluster_size);

}

void lidar_seg::get_params(double pan_eps, double tilt_eps, double dist_eps, int min_points, int cluster_size){

  //taking the parameters
  this->pan_epsilon = pan_eps;
  this->tilt_epsilon = tilt_eps;
  this->dist_epsilon = dist_eps;
  this->seg_min = min_points;
  this->min_cluster_size = cluster_size;

  pan_direction = round(this->pan_epsilon/ this->pan_resolution); //find the maximum direction
  tilt_direction = round(this->tilt_epsilon / this->tilt_resolution); //find the maximum direction

}


//Get Sphere coordinates for cloud data without color information
void lidar_seg::to_sphere() {
    sphere_cloud.points.resize(cloud->points.size());
    pcl::PointXYZ sphere_points;
    index_vec.resize(total_pan + 10, std::vector<int>(total_tilt + 10));
    std::fill(index_vec.begin(), index_vec.end(), vector<int>(total_tilt + 10, -1)); //index vector is filled with -1
    int pan_idx, tilt_idx;
    // To sphere
    for (int i = 0; i < cloud->size(); i++) {

        sphere_cloud.points[i].x = sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y + cloud->points[i].z * cloud->points[i].z); //distance
        sphere_cloud.points[i].y = asin(cloud->points[i].z / sphere_cloud.points[i].x); //tilt angle
        sphere_cloud.points[i].z = atan2(cloud->points[i].x, cloud->points[i].y); //pan angle

        if (sphere_cloud.points[i].z < 0.0)
        {
            sphere_cloud.points[i].z = sphere_cloud.points[i].z + 2 * M_PI; //0 and 2*M_PI
        }

        sphere_cloud.points[i].y = sphere_cloud.points[i].y + M_PI / 2; //0 and M_PI


        sphere_cloud.points[i].y = sphere_cloud.points[i].y * 180.0 / M_PI; //to degree
        sphere_cloud.points[i].z = sphere_cloud.points[i].z * 180.0 / M_PI; //to degree

        pan_idx = round(sphere_cloud.points[i].z / this->pan_resolution); //finding pan index
        tilt_idx = round(sphere_cloud.points[i].y / this->tilt_resolution); //finding tilt index

        //cout<<sphere_cloud.points[i].y<<endl;
        index_vec[pan_idx][tilt_idx] = i; //take an index vec;

    }

}

//Sphere coordinates from depth and rgb images

void lidar_seg::to_sphere(cv::Mat depth, cv::Mat rgb) {

   // sphere_cloud.points.resize(depth.rows * depth.cols);
    if (sphere_cloud.size() != 0)
    {
        cloud->clear();
        sphere_cloud.clear();
    }
    
    cloudImageMapping.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //tempCloud->points.resize(depth.rows * depth.cols);
    index_vec.resize(total_pan + 10, std::vector<int>(total_tilt + 10));
    std::fill(index_vec.begin(), index_vec.end(), vector<int>(total_tilt + 10, -1)); //index vector is filled with -1

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    source_cloud->resize(1);
    int pan_idx, tilt_idx;
    int index = 0;
    vector<int> coord{ 0, 0 };
    for (int i = 0; i < depth.rows; i++)
    {
        
        for (int j = 0; j < depth.cols; j++)
        {
            unsigned short p = depth.at<unsigned short>(i, j);
            float x_multiplier = (j - RGB_C_X) / RGB_FOCAL_X;
            float y_multiplier = (i- RGB_C_Y) / RGB_FOCAL_Y;

            // Get 3D coordinates
            float z_world = double(p) / 1000 / sqrt(x_multiplier * x_multiplier + y_multiplier * y_multiplier + 1);
            if (z_world == 0)
                continue;
            float x_world = z_world * x_multiplier;
            float y_world = z_world * y_multiplier;

			//Transform the cloud from depth camera coordinate system to RGB coordinate system
            source_cloud->points[0].x = x_world;
            source_cloud->points[0].y = y_world;
            source_cloud->points[0].z = z_world;

            pcl::transformPointCloud((*source_cloud), *transformed_cloud, this->DepthtoRGBMat);

            x_world = transformed_cloud->points[0].x;
            y_world = transformed_cloud->points[0].y;
            z_world = transformed_cloud->points[0].z;

            // Get corresponding RGB pixels
            float x_rgbcam = RGB_FOCAL_X * x_world / z_world + RGB_C_X;
            float y_rgbcam = RGB_FOCAL_Y * y_world / z_world + RGB_C_Y;

            // "Interpolate" pixel coordinates 
            int px_rgbcam = cvRound(x_rgbcam);
            int py_rgbcam = cvRound(y_rgbcam);

            //assign color if applicable
            if (px_rgbcam > 0 && px_rgbcam < depth.cols && py_rgbcam>0 && py_rgbcam < depth.rows)
            {
               
                pcl::PointXYZRGB sphere_point;
                pcl::PointXYZRGB cloud_point;
                coord[0] = px_rgbcam;
                coord[1] = py_rgbcam;
                if (cloudImageMapping.find(coord) != cloudImageMapping.end())
                    continue;
                cloudImageMapping.insert(std::make_pair(coord, index));
                int b = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[0];
                int g = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[1];
                int r = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[2];
				
				//set cloud data
                cloud_point.b = b;
                cloud_point.g = g;
                cloud_point.r = r;

                cloud_point.x = x_world;
                cloud_point.y = y_world;
                cloud_point.z = z_world;
                tempCloud->push_back(cloud_point);
                //convert to spherical coordinate system
                sphere_point.x = sqrt(x_world * x_world + y_world * y_world + z_world * z_world);
                if (sphere_point.x == 0)
                {
                    sphere_point.x = 10000;
                }
                sphere_point.y = asin(z_world / (sphere_point.x));
                sphere_point.z = atan2(y_world, x_world);
                if (sphere_point.z < 0.0)
                {
                    sphere_point.z = sphere_point.z + 2 * M_PI; //0 and 2*M_PI
                }
                sphere_point.y = sphere_point.y + M_PI / 2; //0 and M_PI
                sphere_point.y = sphere_point.y * 180.0 / M_PI; //to degree
                sphere_point.z = sphere_point.z * 180.0 / M_PI; //to degree

                pan_idx = round(sphere_point.z / this->pan_resolution); //finding pan index
                tilt_idx = round(sphere_point.y / this->tilt_resolution); //finding tilt index
                sphere_cloud.push_back(sphere_point);
                index_vec[pan_idx][tilt_idx] = index; //take an index vec;
                ++index;

                
            }
        }
    }
    cloud = tempCloud;
 
}



void lidar_seg::to_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl, cv::Mat rgb) {

   // sphere_cloud.points.resize(depth.rows * depth.cols);
    cloud->clear();
    sphere_cloud.clear();
    cloudImageMapping.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //tempCloud->points.resize(depth.rows * depth.cols);
    index_vec.resize(total_pan + 10, std::vector<int>(total_tilt + 10));
    std::fill(index_vec.begin(), index_vec.end(), vector<int>(total_tilt + 10, -1)); //index vector is filled with -1

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    source_cloud->resize(1);
    int pan_idx, tilt_idx;
    int index = 0;
    vector<int> coord{ 0, 0 };
    //unsigned char* data = (depth.data);
    for (int i = 0; i < depth_pcl->size(); i++)
    {

            float x_world,y_world,z_world;
            source_cloud->points[0].x = depth_pcl->points[i].x;
            source_cloud->points[0].y = depth_pcl->points[i].y;
            source_cloud->points[0].z = depth_pcl->points[i].z;

            pcl::transformPointCloud((*source_cloud), *transformed_cloud, this->DepthtoRGBMat);

            x_world = transformed_cloud->points[0].x;
            y_world = transformed_cloud->points[0].y;
            z_world = transformed_cloud->points[0].z;

            // Get corresponding RGB pixels
            float x_rgbcam = RGB_FOCAL_X * x_world / z_world + RGB_C_X;
            float y_rgbcam = RGB_FOCAL_Y * y_world / z_world + RGB_C_Y;

            // "Interpolate" pixel coordinates 
            int px_rgbcam = cvRound(x_rgbcam);
            int py_rgbcam = cvRound(y_rgbcam);
            //assign color if applicable
            if (px_rgbcam > 0 && px_rgbcam < rgb.cols && py_rgbcam>0 && py_rgbcam < rgb.rows)
            {
               
                pcl::PointXYZRGB sphere_point;
                pcl::PointXYZRGB cloud_point;
                coord[0] = px_rgbcam;
                coord[1] = py_rgbcam;
                if (cloudImageMapping.find(coord) != cloudImageMapping.end())
                    continue;
                cloudImageMapping.insert(std::make_pair(coord, index));
                cloud_point.b = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[0];
                cloud_point.g = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[1];
                cloud_point.r = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[2];
                cloud_point.x = x_world;
                cloud_point.y = y_world;
                cloud_point.z = z_world;
                tempCloud->push_back(cloud_point);
                //convert to spherical coordinate system
                sphere_point.x = sqrt(x_world * x_world + y_world * y_world + z_world * z_world);
                if (sphere_point.x == 0)
                {
                    sphere_point.x = 10000;
                }
                sphere_point.y = asin(z_world / (sphere_point.x));
                sphere_point.z = atan2(y_world, x_world);
                if (sphere_point.z < 0.0)
                {
                    sphere_point.z = sphere_point.z + 2 * M_PI; //0 and 2*M_PI
                }
                sphere_point.y = sphere_point.y + M_PI / 2; //0 and M_PI
                sphere_point.y = sphere_point.y * 180.0 / M_PI; //to degree
                sphere_point.z = sphere_point.z * 180.0 / M_PI; //to degree

                pan_idx = round(sphere_point.z / this->pan_resolution); //finding pan index
                tilt_idx = round(sphere_point.y / this->tilt_resolution); //finding tilt index
                sphere_cloud.push_back(sphere_point);
                index_vec[pan_idx][tilt_idx] = index; //take an index vec;
                ++index;

                
            }
        }
    
    cloud = tempCloud;

}


void lidar_seg::to_sphere(cv::Mat depth, cv::Mat rgb, vector<cv::Rect> hand_loc) {
    if (sphere_cloud.size() > 0)
    {
        sphere_cloud.clear();
        cloud->clear();
    }
   
    cloudImageMapping.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    index_vec.resize(total_pan + 10, std::vector<int>(total_tilt + 10));
    std::fill(index_vec.begin(), index_vec.end(), vector<int>(total_tilt + 10, -1)); //index vector is filled with -1

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    source_cloud->resize(1);

    int pan_idx, tilt_idx;
    int index = 0;
    vector<int> coord(2);

    for (int i = 0; i < depth.rows; i++)
    {
        for (int j = 0; j < depth.cols; j++)
        {
            //get depth reading in mm and convert it to 3D coordinates
            unsigned short p = depth.at<unsigned short>(i, j);
            float x_multiplier = (j - RGB_C_X) / RGB_FOCAL_X;
            float y_multiplier = (i- RGB_C_Y) / RGB_FOCAL_Y;
            
            // Get 3D coordinates
            float z_world = double(p)/1000 / sqrt(x_multiplier * x_multiplier + y_multiplier * y_multiplier + 1); 
            //if reading is 0 then its out of range of the depth cam
            if (z_world == 0)
                z_world = 1000;
            float x_world = z_world * x_multiplier;
            float y_world = z_world * y_multiplier;

            source_cloud->points[0].x = x_world;
            source_cloud->points[0].y = y_world;
            source_cloud->points[0].z = z_world;

            //transformation between depth and rgb camera 3d spaces
            pcl::transformPointCloud((*source_cloud), *transformed_cloud, this->DepthtoRGBMat);

            //3D coordinates in rgb cam space
            x_world = transformed_cloud->points[0].x;
            y_world = transformed_cloud->points[0].y;
            z_world = transformed_cloud->points[0].z;

            // Get corresponding RGB pixels
            float x_rgbcam = RGB_FOCAL_X * x_world /(z_world) + RGB_C_X;
            float y_rgbcam = RGB_FOCAL_Y * y_world / (z_world) + RGB_C_Y;

            // "Interpolate" pixel coordinates 
            int px_rgbcam = cvRound(x_rgbcam);
            int py_rgbcam = cvRound(y_rgbcam);

            //assign color if applicable
            if (px_rgbcam > 0 && px_rgbcam < depth.cols && py_rgbcam>0 && py_rgbcam < depth.rows )
            {
                for (int k = 0; k < hand_loc.size(); k++)
                {
                    //Check if 2D point falls to hand location
                    if (px_rgbcam <= hand_loc[k].x + hand_loc[k].width && px_rgbcam >= hand_loc[k].x &&
                        py_rgbcam <= hand_loc[k].y + hand_loc[k].height && py_rgbcam >= hand_loc[k].y && z_world> min_range)
                    {
                        pcl::PointXYZRGB sphere_point;
                        pcl::PointXYZRGB cloud_point;
                        // create mapping of 2D coordinates and point cloud data
                        coord[0] = px_rgbcam;
                        coord[1] = py_rgbcam;
                        if (cloudImageMapping.find(coord) != cloudImageMapping.end())
                            continue;
                        cloudImageMapping.insert(std::make_pair(coord, index));

                        //create point cloud data
                        cloud_point.b = rgb.at<cv::Vec3b>(i, j)[0];
                        cloud_point.g = rgb.at<cv::Vec3b>(i, j)[1];
                        cloud_point.r = rgb.at<cv::Vec3b>(i, j)[2];
                        cloud_point.x = x_world;
                        cloud_point.y = y_world;
                        cloud_point.z = z_world;
                        tempCloud->push_back(cloud_point);
                        //convert to spherical coordinate system
                        sphere_point.x = sqrt(x_world * x_world + y_world * y_world + z_world * z_world);
                        if (sphere_point.x == 0)
                        {
                            sphere_point.x = 10000;
                        }
                        sphere_point.y = asin(z_world / (sphere_point.x));
                        sphere_point.z = atan2(y_world, x_world);
                        if (sphere_point.z < 0.0)
                        {
                            sphere_point.z = sphere_point.z + 2 * M_PI; //0 and 2*M_PI
                        }
                        sphere_point.y = sphere_point.y + M_PI / 2; //0 and M_PI
                        sphere_point.y = sphere_point.y * 180.0 / M_PI; //to degree
                        sphere_point.z = sphere_point.z * 180.0 / M_PI; //to degree

                        pan_idx = round(sphere_point.z / this->pan_resolution); //finding pan index
                        tilt_idx = round(sphere_point.y / this->tilt_resolution); //finding tilt index
                        sphere_cloud.push_back(sphere_point);
                        index_vec[pan_idx][tilt_idx] = index; //take an index vec;
                        ++index;
                        break;
                    }
                }
                
            }
        }
    }
   
    cloud = tempCloud;

}


/*
Method: This method generates point cloud expressed in spherical coo. form an input 2-D depth Matrix expressed wrt pan and tilt angles.
*/

void lidar_seg::to_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl, cv::Mat rgb, vector<cv::Rect> hand_loc) {
    if (sphere_cloud.size() > 0)
    {
        sphere_cloud.clear();
        cloud->clear();
    }
   
    cloudImageMapping.clear();
    //sphere_cloud.points.resize(depth.rows*depth.cols);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    tempCloud->points.resize(depth_pcl->size());
    index_vec.resize(total_pan + 10, std::vector<int>(total_tilt + 10));
    std::fill(index_vec.begin(), index_vec.end(), vector<int>(total_tilt + 10, -1)); //index vector is filled with -1

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    source_cloud->resize(1);

    int pan_idx, tilt_idx;
    int index = 0;
    vector<int> coord{ 0, 0 };
    for (int i = 0; i < depth_pcl->size(); i++)
    {
            float x_world,y_world,z_world;

            source_cloud->points[0].x = depth_pcl->points[i].x;
            source_cloud->points[0].y = depth_pcl->points[i].y;
            source_cloud->points[0].z = depth_pcl->points[i].z;

            //transformation between depth and rgb camera 3d spaces
            pcl::transformPointCloud((*source_cloud), *transformed_cloud, this->DepthtoRGBMat);

            x_world = transformed_cloud->points[0].x;
            y_world = transformed_cloud->points[0].y;
            z_world = transformed_cloud->points[0].z;

            // Get corresponding RGB pixels
            float x_rgbcam = RGB_FOCAL_X * x_world /(z_world) + RGB_C_X;
            float y_rgbcam = RGB_FOCAL_Y * y_world / (z_world) + RGB_C_Y;

            // "Interpolate" pixel coordinates 
            int px_rgbcam = cvRound(x_rgbcam);
            int py_rgbcam = cvRound(y_rgbcam);
            //assign color if applicable
            if (px_rgbcam > 0 && px_rgbcam < rgb.cols && py_rgbcam>0 && py_rgbcam < rgb.rows )
            {
                for (int k = 0; k < hand_loc.size(); k++)
                {
                    //Check if 2D point falls to hand location
                    if (px_rgbcam <= hand_loc[k].x + hand_loc[k].width && px_rgbcam >= hand_loc[k].x &&
                        py_rgbcam <= hand_loc[k].y + hand_loc[k].height && py_rgbcam >= hand_loc[k].y && z_world> min_hand_dist)
                    {
                        pcl::PointXYZRGB sphere_point;

                        // create mapping of 2D coordinates and point cloud data
                        coord[0] = px_rgbcam;
                        coord[1] = py_rgbcam;
                        if (cloudImageMapping.find(coord) != cloudImageMapping.end())
                            continue;
                        cloudImageMapping.insert(std::make_pair(coord, index));

                        //create point cloud data
                        tempCloud->points[index].b = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[0];
                        tempCloud->points[index].g = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[1];
                        tempCloud->points[index].r = rgb.at<cv::Vec3b>(py_rgbcam, px_rgbcam)[2];
                        tempCloud->points[index].x = x_world;
                        tempCloud->points[index].y = y_world;
                        tempCloud->points[index].z = z_world;
                        //convert to spherical coordinate system
                        sphere_point.x = sqrt(x_world * x_world + y_world * y_world + z_world * z_world);
                        if (sphere_point.x == 0)
                        {
                            sphere_point.x = 10000;
                        }
                        sphere_point.y = asin(z_world / (sphere_point.x));
                        sphere_point.z = atan2(y_world, x_world);
                        if (sphere_point.z < 0.0)
                        {
                            sphere_point.z = sphere_point.z + 2 * M_PI; //0 and 2*M_PI
                        }
                        sphere_point.y = sphere_point.y + M_PI / 2; //0 and M_PI
                        sphere_point.y = sphere_point.y * 180.0 / M_PI; //to degree
                        sphere_point.z = sphere_point.z * 180.0 / M_PI; //to degree

                        pan_idx = round(sphere_point.z / this->pan_resolution); //finding pan index
                        tilt_idx = round(sphere_point.y / this->tilt_resolution); //finding tilt index
                        sphere_cloud.push_back(sphere_point);
                        index_vec[pan_idx][tilt_idx] = index; //take an index vec;
                        ++index;
                        break;
                    }
                }
                
            }
        }
    
   
    cloud = tempCloud;
   
}


void::lidar_seg::segment(){


  to_sphere(); //find spherical coordinates and range images
  //  cout<<"size:"<<sphere_cloud.size()<<endl;

  int k ; //while iterator to expand the cluster
  for(int i = 0; i< sphere_cloud.points.size(); i++){

    if(visited[i] == true){

      continue; //if the value visited the algorithm skip this point
    }
    else{

      std::vector<int> neighs =  region_query(i,true); //Find the core point

      if(neighs.size()<=  0)
      {

        //if this is not a core point continue to find a core point

        continue;

      }

      else{
        //if the core point is found, the cluster will expanded with other
        cluster = cluster +1;
        clusters[i] = cluster;
        k = 0;
        visited[i] = true;
        while(k< neighs.size())
        {

        //  clusters[neighs[k]] = cluster;
          std::vector<int> neighs_expand = region_query(neighs[k],false);
          if(neighs_expand.size()> 0)
          {
            neighs.insert( neighs.end(), neighs_expand.begin(), neighs_expand.end() );
          }

          k = k + 1 ;

        }


      }

      if(neighs.size() +1 < this->seg_min){
clusters[i]= -1;
for(auto a: neighs)
  clusters[a] = -1;
  cluster = cluster -1;

      }

      neighs.clear();
    }



  }
  total_cluster= cluster;
  cout<<"cluster: "<<cluster<<endl;

}

void lidar_seg::segment(cv::Mat depth, cv::Mat rgb) {

    to_sphere(depth, rgb); 
    int k; //while iterator to expand the cluster
    double sur = 0;
    clock_t begin = clock();
    colors.resize(cloud->points.size());
    clusters.resize(cloud->points.size());
    hsv empty{ 0,0,0 };
    std::fill(colors.begin(), colors.end(), empty);


    for (int i = 0; i < sphere_cloud.points.size(); i++) {
        if (cloud->points[i].z < min_range || cloud->points[i].z>max_range)
            continue;
        if (visited[i] == true) {

            continue; //if the value visited the algorithm skip this point
        }
        else {

            std::vector<int> neighs = region_query(i, true); //Find the core point


            if (neighs.size() <= 0)
            {

                //if this is not a core point continue to find a core point

                continue;
            }

            else {
                //if the core point is found, the cluster will expanded with other
                cluster = cluster + 1;
                clusters[i] = cluster;
                k = 0;
                visited[i] = true;

                while (k < neighs.size())
                {

                    //  clusters[neighs[k]] = cluster;
                    std::vector<int> neighs_expand = region_query(neighs[k], false);

                    if (neighs_expand.size() > 0)
                    {
                        neighs.insert(neighs.end(), neighs_expand.begin(), neighs_expand.end());
                    }
                    k = k + 1;

                }
            }

            if (neighs.size() + 1 < this->min_cluster_size) {
                clusters[i] = -1;
                for (auto a : neighs)
                    clusters[a] = -1;
                cluster = cluster - 1;

            }
            neighs.clear();
        }

    }
    total_cluster = cluster;
}
int lidar_seg::get_total_cluster()
{
    return this->total_cluster;
}

void::lidar_seg::segment(cv::Mat depth, cv::Mat rgb, vector<cv::Rect> hand_loc) {
    
    to_sphere(depth, rgb, hand_loc); //find spherical coordinates and range images

    colors.resize(cloud->points.size());
    clusters.resize(cloud->points.size());
    hsv empty{ 0,0,0 };
    std::fill(colors.begin(), colors.end(), empty);
    int k; //while iterator to expand the cluster
    for (int i = 0; i < sphere_cloud.points.size(); i++) {
        if (visited[i] == true) {
            continue; //if the value visited the algorithm skip this point
        }
        else {

            std::vector<int> neighs = region_query(i, true); //Find the core point     
            if (neighs.size() <= 0)
            {
                //if this is not a core point continue to find a core point
                continue;
            }
            else {
                //if the core point is found, the cluster will expanded with other
                cluster = cluster + 1;
                clusters[i] = cluster;
                k = 0;
                visited[i] = true;
                while (k < neighs.size())
                {
                    //  clusters[neighs[k]] = cluster;
                    std::vector<int> neighs_expand = region_query(neighs[k], false); 
                    if (neighs_expand.size() > 0)
                    {
                        neighs.insert(neighs.end(), neighs_expand.begin(), neighs_expand.end());
                    }
                    k = k + 1;

                }  
            }
            if (neighs.size() + 1 < this->seg_min) {
                clusters[i] = -1;
                for (auto a : neighs)
                    clusters[a] = -1;
                cluster = cluster - 1;
            }
            neighs.clear();
        }
    }
    total_cluster = cluster;
}


/*
Method: Depth data segmentation

*/
void::lidar_seg::segment(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl, cv::Mat rgb, vector<cv::Rect> hand_loc) {
    
    to_sphere(depth_pcl, rgb, hand_loc); //find spherical coordinates and range images
    colors.resize(sphere_cloud.points.size());
	clusters.resize(cloud->points.size());
    int k; //while iterator to expand the cluster
    for (int i = 0; i < sphere_cloud.points.size(); i++) {

        if (visited[i] == true) {

            continue; //if the value visited the algorithm skip this point
        }
        else {

            std::vector<int> neighs = region_query(i, true); //Find the core point

            
            if (neighs.size() <= 0)
            {

                //if this is not a core point continue to find a core point

                continue;

            }

            else {
                //if the core point is found, the cluster will expanded with other
                cluster = cluster + 1;
                clusters[i] = cluster;
                k = 0;
                visited[i] = true;
              
               
                while (k < neighs.size())
                {

                    //  clusters[neighs[k]] = cluster;
                    std::vector<int> neighs_expand = region_query(neighs[k], false);
                     
                    if (neighs_expand.size() > 0)
                    {
                        neighs.insert(neighs.end(), neighs_expand.begin(), neighs_expand.end());
                    }
                    k = k + 1;

                }

            }

            if (neighs.size() + 1 < this->seg_min) {
                clusters[i] = -1;
                for (auto a : neighs)
                    clusters[a] = -1;
                cluster = cluster - 1;

            }
            neighs.clear();
        }

    }
    total_cluster = cluster;
}

void lidar_seg::segment(cv::Mat depth, cv::Mat rgb, vector<Eigen::MatrixXf> normal,float dist, vector<vector<pcl::PointXYZRGB>> hand_center, cv::Rect human) {
   
    to_sphere(depth, rgb); //find spherical coordinates and range images
 
    int k; //while iterator to expand the cluster
    double sur = 0;
    clock_t begin = clock();
    colors.resize(cloud->points.size());
    clusters.resize(cloud->points.size());
    hsv empty{ 0,0,0 };
    std::fill(colors.begin(), colors.end(), empty);
   

    for (int i = 0; i < sphere_cloud.points.size(); i++) {
        int flag = 0;
        if (cloud->points[i].z < 0.55 || cloud->points[i].z>5)
            continue;

        //get only points in the angle range of pointnig direction
        for (int j = 0; j < normal.size(); j++)
        { 
            float x_vec = hand_center[j][0].x - cloud->points[i].x;
            float y_vec = hand_center[j][0].y - cloud->points[i].y;
            float z_vec = hand_center[j][0].z - cloud->points[i].z;
            float norm_vec = sqrt(x_vec * x_vec + y_vec * y_vec + z_vec * z_vec);
            float norm_dir = sqrt(normal[j](0) * normal[j](0) + normal[j](1) * normal[j](1) + normal[j](2) * normal[j](2));
            float angle = acos((x_vec * normal[j](0) + y_vec * normal[j](1) + z_vec * normal[j](2)) / (norm_vec * norm_dir));
            if ((M_PI - angle) < angle_range || angle < angle_range)
                flag = 1;

            
        }
        if (flag == 0)
            continue;
        if (visited[i] == true) {
            
            continue; //if the value visited the algorithm skip this point
        }
        else 
        {
            std::vector<int> neighs = region_query(i, true); //Find the core point
           
            if (neighs.size() <= 0)
            {
                //if this is not a core point continue to find a core point

                continue;

            }

            else {
                //if the core point is found, the cluster will expanded with other
                cluster = cluster + 1;
                clusters[i] = cluster;
                k = 0;
                visited[i] = true;

                while (k < neighs.size())
                {

                    //  clusters[neighs[k]] = cluster;
                    std::vector<int> neighs_expand = region_query(neighs[k], false);

                    if (neighs_expand.size() > 0)
                    {
                        neighs.insert(neighs.end(), neighs_expand.begin(), neighs_expand.end());
                    }
                    k = k + 1;

                }
            }
            
            if (neighs.size() + 1 < this->min_cluster_size) {
                clusters[i] = -1;
                for (auto a : neighs)
                    clusters[a] = -1;
                cluster = cluster - 1;
            }
            neighs.clear();
        }
    }
    clock_t end = clock();
    sur += double(end - begin) / CLOCKS_PER_SEC;
    total_cluster = cluster;

}


void lidar_seg::segment(pcl::PointCloud<pcl::PointXYZ>::Ptr depth_pcl, cv::Mat rgb, vector<Eigen::MatrixXf> normal,float dist, vector<vector<pcl::PointXYZRGB>> hand_center, cv::Rect human) {
   
    to_sphere(depth_pcl, rgb); //find spherical coordinates and range images
    int k; //while iterator to expand the cluster
    double sur = 0;
    clock_t begin = clock();
    colors.resize(cloud->points.size());
    clusters.resize(cloud->points.size());
    hsv empty{ 0,0,0 };
    std::fill(colors.begin(), colors.end(), empty);
   

    for (int i = 0; i < sphere_cloud.points.size(); i++) {
        int flag = 0;
        if (cloud->points[i].z < 0.55 || cloud->points[i].z>5)
            continue;

        //get only points in the angle range of pointnig direction
        for (int j = 0; j < normal.size(); j++)
        { 
            float x_vec = hand_center[j][0].x - cloud->points[i].x;
            float y_vec = hand_center[j][0].y - cloud->points[i].y;
            float z_vec = hand_center[j][0].z - cloud->points[i].z;
            float norm_vec = sqrt(x_vec * x_vec + y_vec * y_vec + z_vec * z_vec);
            float norm_dir = sqrt(normal[j](0) * normal[j](0) + normal[j](1) * normal[j](1) + normal[j](2) * normal[j](2));
            float angle = acos((x_vec * normal[j](0) + y_vec * normal[j](1) + z_vec * normal[j](2)) / (norm_vec * norm_dir));
            if ((M_PI - angle) < angle_range || angle < angle_range)
                flag = 1;

            
        }
        if (flag == 0)
            continue;
        if (visited[i] == true) {
            
            continue; //if the value visited the algorithm skip this point
        }
        else 
        {
            std::vector<int> neighs = region_query(i, true); //Find the core point
           
            if (neighs.size() <= 0)
            {
                //if this is not a core point continue to find a core point

                continue;

            }

            else {
                //if the core point is found, the cluster will expanded with other
                cluster = cluster + 1;
                clusters[i] = cluster;
                k = 0;
                visited[i] = true;

                
                while (k < neighs.size())
                {

                    //  clusters[neighs[k]] = cluster;
                    std::vector<int> neighs_expand = region_query(neighs[k], false);

                    if (neighs_expand.size() > 0)
                    {
                        neighs.insert(neighs.end(), neighs_expand.begin(), neighs_expand.end());
                    }
                    k = k + 1;

                }
                
               
                


            }
            
            if (neighs.size() + 1 < this->min_cluster_size) {
                clusters[i] = -1;
                for (auto a : neighs)
                    clusters[a] = -1;
                cluster = cluster - 1;


            }
            neighs.clear();
        }



    }
    clock_t end = clock();
    sur += double(end - begin) / CLOCKS_PER_SEC;
    total_cluster = cluster;
}

template <typename T>
bool contains(vector<T> vec, const T& elem)
{
    bool result = false;
    if (find(vec.begin(), vec.end(), elem) != vec.end())
    {
        result = true;
    }
    return result;
}

void lidar_seg::dist(int index, int core_index, std::vector<int> &result_neigh){
    
    
    clock_t begin = clock();
    if ((clusters[index] != 0 && clusters[index] != cluster) ) 
        return;
      float ellipsoid_dist; //ellipsoid distance

      hsv core_hsv;
      hsv ind_hsv;

      float pan_neigh = sphere_cloud.points[index].z ;
      float tilt_neigh = sphere_cloud.points[index].y;
      float dist_neigh = sphere_cloud.points[index].x;


      float pan_dist = 180-fabs(fabs(sphere_cloud.points[core_index].z - pan_neigh) -180); //calculate pan distance between two points
      float tilt_dist = sphere_cloud.points[core_index].y - tilt_neigh; // calculate tilt distance between two points
      float dist_dist = dist_neigh - sphere_cloud.points[core_index].x; // calculate distance

      //check if hsv dist is recorded before for both core_index and index
      if (colors[index].h == 0 && colors[index].s == 0 && colors[index].v == 0)
      {
          rgb ind_rgb = { cloud->points[index].r / 255.0 ,cloud->points[index].g / 255.0 ,cloud->points[index].b / 255.0 };
          ind_hsv = rgb2hsv(ind_rgb);
          colors[index] = ind_hsv;
      }
      else
          ind_hsv = colors[index];
      if (colors[core_index].h == 0 && colors[core_index].s == 0 && colors[core_index].v == 0)
      {
          rgb ind_rgb = { cloud->points[core_index].r / 255.0 ,cloud->points[core_index].g / 255.0 ,cloud->points[core_index].b / 255.0 };
          core_hsv = rgb2hsv(ind_rgb);
          colors[core_index] = core_hsv;
      }
      else
          core_hsv = colors[core_index];

      float color_dist =  hsvdist(core_hsv, ind_hsv);

      ellipsoid_dist = ((pan_dist * pan_dist) / (this->pan_epsilon * this->pan_epsilon)) + ((tilt_dist * tilt_dist)/(this->tilt_epsilon * this->tilt_epsilon)) + ((dist_dist * dist_dist)/(this->dist_epsilon * this->dist_epsilon)); //calculate ellipsoid distance
      if ((ellipsoid_dist <= elipsoid_thresh1) || (ellipsoid_dist <= elipsoid_thresh2 && color_dist < color_thresh2) || (ellipsoid_dist <= elipsoid_thresh3 && color_dist < color_thresh1))
      {

        if(clusters[index]==0 || clusters[index]==cluster){
      

          min_cluster_var = min_cluster_var + 1;
          if(visited[index] == false )
          { //only unique indexes can be added
            result_neigh.push_back(index);
            visited[index] = true;

          }

        }

      }
      
}

std::vector<int> lidar_seg::region_query(int iterator, bool core){

  std::vector<int> result_vec;
  

  min_cluster_var = 0;

  int pan_idx = int(sphere_cloud.points[iterator].z/this->pan_resolution);
  int tilt_idx = int(sphere_cloud.points[iterator].y / this->tilt_resolution);



  int dec_idx_pan=0;
  int inc_idx_pan= 0;
  int inc_idx_tilt =0;
  int dec_idx_tilt =0;

  int idx ;
  /*
  Find possible directions in the range image
  Tilt direction can be access the boundries, for this reason the directions are splitted into 3 parts. The first part is limited only the incremental direction in tilt,
  The second one is limited in the decreasing direction in tilt,
  There is no limitation for  the third one.


  */
  for(int i = 0; i< this->tilt_direction; i++){
    inc_idx_tilt = tilt_idx + i;
    dec_idx_tilt = tilt_idx - i;

    

    for(int k = 0; k< this->pan_direction; k++){



      if(k==0 && i==0) // core point continue
      {
        continue;
      }

      inc_idx_pan = (pan_idx +k) % total_pan ;
      dec_idx_pan =  pan_idx - k ;
      if(dec_idx_pan<0)
      {
        dec_idx_pan = dec_idx_pan + total_pan ; // function mod
      }


      if(inc_idx_tilt<= total_tilt && dec_idx_tilt <0 ){

        idx = index_vec[inc_idx_pan][inc_idx_tilt];
        if(idx!=-1 && visited[idx] == false)
            dist(idx,iterator, result_vec);

        idx = index_vec[dec_idx_pan][inc_idx_tilt];
        if(idx!=-1 && visited[idx]==false)
            dist(idx, iterator, result_vec);



      }
      else if(inc_idx_tilt> total_tilt && dec_idx_tilt > 0)
      {


        idx = index_vec[inc_idx_pan][dec_idx_tilt] ;
        if(idx!=-1 && visited[idx] == false)
            dist(idx, iterator,result_vec);

        idx = index_vec[dec_idx_pan][dec_idx_tilt];
        if(idx!=-1 && visited[idx] == false)
            dist(idx,iterator,result_vec);


        //  cout<<"hey"<<endl;


      }

      else if(inc_idx_tilt<=total_tilt && dec_idx_tilt>=0)
      {
        //  cout<<"hey"<<endl;



        idx = index_vec[inc_idx_pan][dec_idx_tilt] ;
        if(idx!=-1 && visited[idx] == false)
            dist(idx, iterator,result_vec);

        idx = index_vec[dec_idx_pan][dec_idx_tilt];
        if(idx!=-1 && visited[idx] == false)
            dist(idx,iterator,result_vec);



        idx = index_vec[inc_idx_pan][inc_idx_tilt];
        if(idx!=-1 && visited[idx] == false)
            dist(idx,iterator, result_vec);

        idx = index_vec[dec_idx_pan][inc_idx_tilt];
        if(idx!=-1 && visited[idx] == false)
            dist(idx, iterator, result_vec);

      }


    }
  }

  //Control the density
  if (core)
  {
      if (min_cluster_var < seg_min) {
          //if the point is not a core point or this is not a dense region, all points in the region free
          visited[iterator] = true;
          for (size_t not_core_it = 0; not_core_it < result_vec.size(); not_core_it++) {

              visited[result_vec[not_core_it]] = false;

          }
          result_vec.clear();
          min_cluster_var = 0;
          return result_vec;

      }
  }

    //if this is a core point, take the neighs

    for(size_t cluster_assign=0; cluster_assign< result_vec.size(); cluster_assign++){
      if(core == true){
        clusters[result_vec[cluster_assign]] = cluster +1; //assign the cluster

      }

      else{
        clusters[result_vec[cluster_assign]] = cluster; //assign the cluster

      }
    }
    min_cluster_var = 0;
    return result_vec;

}

void lidar_seg::take_colored_cloud(pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud){

    vector<size_t> color_vec(total_cluster);

    for(int i = 0;i<total_cluster;i++){
    // srand(time(NULL));
    size_t randNum = std::rand()%(256) ;

    color_vec[i] = randNum;
    }

    colored_cloud.resize(clusters.size());


    vector<vector<int> > color_vector;
      //color_vector.resize(cluster,vector<uint8_t>(3));
    vector<int> rgb;
    int c1,c2,c3;
    for(size_t color_cluster= 0 ; color_cluster<cluster; color_cluster++){

        c1 = rand() % 255 ;
        c2 = rand() % 255 ;
        c3 = rand() % 255 ;

        rgb.push_back(c1);
        rgb.push_back(c2);
        rgb.push_back(c3);

        color_vector.push_back(rgb);
        rgb.clear();
    }





    for(size_t color_it = 0; color_it< clusters.size(); color_it++){

      if(clusters[color_it] <= 0 ) //non-clustered points are skipped
      {



        continue;

      }

      colored_cloud.points[color_it].x = cloud->points[color_it].x;
      colored_cloud.points[color_it].y = cloud->points[color_it].y;
      colored_cloud.points[color_it].z = cloud->points[color_it].z;

      colored_cloud.points[color_it].r =  color_vector[clusters[color_it] - 1][0];//(255/cluster) * clusters[color_it] ;
      colored_cloud.points[color_it].g =  color_vector[clusters[color_it] - 1][1];//(255/cluster) * clusters[color_it] ;
      colored_cloud.points[color_it].b =  color_vector[clusters[color_it] - 1][2]; //(255/cluster) * clusters[color_it] ;

    }

  
   
}


vector<int> lidar_seg::get_clusters()
{
    return this->clusters;
}

int lidar_seg::get_cluster_no()
{
    return this->cluster;
}

pcl::PointCloud<pcl::PointXYZRGB> lidar_seg::get_cloud()
{
    return *this->cloud;
}

