// PCL specific includes
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <string>


#include <pcl/common/time.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
//#include  <chrono
#include <vector>
#include <ctime>


#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Geometry>

#include "lidar_seg.h"
#include "seg_process.h"
#include "ground_segmentation.h"
#include "util.h"
//#include "detection.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


#include <chrono> 
#include <string>
#include <string>
#include <iostream>
#include <filesystem>
#include <fstream>
#include "detection.h"

using namespace std::chrono;
using namespace std;
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
using namespace std;
using namespace pcl;
using namespace cv;

float cloud_process(Mat depth, Mat rgb, pcl::PointXYZRGB line_point,float x_dir,float y_dir, float z_dir, cv::Point p1, cv::Point p2, vector<Rect> hands)
{
    vector<Rect> hand_loc;
    hand_loc.push_back(cv::Rect(p1,p2));
    
    //Uncomment this to use hand location coming from the CNN detector.
    //hand_loc = hands;

    // Calculate camera resolution
    double pan_res = PAN_RANGE / depth.cols;
    double tilt_res = TILT_RANGE / depth.rows;

    //segment hand region
    lidar_seg seglid(pan_res, tilt_res); //set the range image resolutions (The first one is pan, the second one is tilt)
    seglid.set_cloud(depth); //Set the cloud
    seglid.set_params(depth_pan_eps, depth_tilt_eps, depth_dist_eps, depth_min_neigh, depth_min_points); //Set segmentation parameters
    seglid.segment(depth, rgb, hand_loc); //Start the segmentation

    //set the pointing direction finder class
    int nu_planes1 = 4;
    int neigh_dist1 = 3;
    int nu_planes2 = 3;
    int neigh_dist2 = 3;
    seg_process hand_obj_proc(nu_planes1, neigh_dist1, nu_planes2, neigh_dist2);
    pcl::PointCloud<pcl::PointXYZRGB> cloud = seglid.get_cloud();
    vector<int> clusters = seglid.get_clusters();
    hand_obj_proc.set_data(clusters, seglid.cloudImageMapping, cloud, seglid.cluster);

    pcl::PointXYZRGB center1;
    pcl::PointXYZRGB center2;
    float plane_dist;
    Eigen::MatrixXf normal;
    vector<int> hand_leftTop(2);
    vector<int> hand_rightBottom(2);
    hand_leftTop[0] = hand_loc[0].x;
    hand_rightBottom[0] = hand_loc[0].x + hand_loc[0].width;
    hand_leftTop[1] = hand_loc[0].y;
    hand_rightBottom[1] = hand_loc[0].y + hand_loc[0].height;
    //get pointing direction and two points on the pointing line
    normal=hand_obj_proc.get_hand_orientation(hand_leftTop, hand_rightBottom, plane_dist, center1, center2);
   
    //calculate error angle
    //Comment this part to not use the ground truth.
    float normal_length = sqrt(normal(0) * normal(0) + normal(1) * normal(1) + normal(2) * normal(2));
    float ground_length = sqrt(x_dir *  x_dir + y_dir * y_dir  + z_dir * z_dir);
    float angle = 180*acos(fabs(normal(0) * x_dir + normal(1) * y_dir + normal(2) * (z_dir))/(normal_length*ground_length))/M_PI;
    
    //get 3D cloud of the whole image
    seglid.to_sphere(depth, rgb);
    //draw ground truth line and detected line on an image and save it
    cloud = seglid.get_cloud();
    Eigen::Matrix3f normal2;
    normal2(0) = x_dir;
    normal2(1) = y_dir;
    normal2(2) = z_dir;
    hand_obj_proc.draw_truth(normal2, center1, cloud);
    hand_obj_proc.draw_dir(normal, center1, cloud);
    Mat img= CloudtoMat(cloud, depth.rows, depth.cols);
    imwrite("result.png",img);

    return angle;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr seg_ground(pcl::PointCloud<pcl::PointXYZ> laserCloudIn)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());


    ground_seg* ground_segmentation = new ground_seg(ground_num_iter, ground_num_lpr, ground_th_seeds, ground_th_dist);
    ground_segmentation->segment(laserCloudIn, ground_cloud, non_ground_cloud);

    return non_ground_cloud;
}

int main(int argc, char** argv)
{

    hand_human_detector detector("yolo_files/yolov4.cfg", "yolo_files/yolov4_last.weights");
    
    vector<Rect> hands;
    vector<Rect> poi_hands;
    Rect human;
    Mat depth_image;
    Mat rgb_image;
    string line;
    int img_no = 0;

    //Get image number
    cout << "Enter Image Number" << endl;
    cin >> img_no;

    int line_counter = 0;
    float x_dir, y_dir, z_dir;
    int left_x, right_x, top_y, bot_y;

    //read annotation file
    std::string name = "annot/img" + to_string(img_no) + ".txt";
    ifstream myfile(name);
    pcl::PointXYZRGB line_point;
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            //read hand point
            if (line_counter == 0)
            {
                line_point.x = stof(line.substr(0, line.find(" ")));
                std::string rest = line.substr(line.find(" ") + 1, -1);
                line_point.y = stof(rest.substr(0, rest.find(" ")));
                line_point.z = stof(rest.substr(rest.find(" ") + 1, -1));
            }
            //read 3D direction
            if (line_counter == 1)
            {
                x_dir = stof( line.substr(0, line.find(" ")));
                std::string rest = line.substr(line.find(" ")+1,-1 );
                y_dir = stof(rest.substr(0, rest.find(" ")));
                z_dir = stof(rest.substr(rest.find(" ") + 1, -1));
            }
            //read 2D hand location
            if (line_counter == 2)
            {
                left_x = stoi(line.substr(0, line.find(" ")));
                std::string rest = line.substr(line.find(" ") + 1, -1);
                top_y = stoi(rest.substr(0, rest.find(" ")));
                rest = rest.substr(rest.find(" ") + 1, -1);
                right_x = stoi(rest.substr(0, rest.find(" ")));
                bot_y = stoi(rest.substr(rest.find(" ") + 1, -1));;
            }
                
            line_counter++;
        }
        myfile.close();
    }
    else
    {
        cout << "Error: Annotation file not available." << endl;
        return 0;
    }
    depth_image = imread("depth_images/depth"+to_string(img_no)+".png", IMREAD_ANYDEPTH);
    rgb_image = imread("rgb_images/rgb" + to_string(img_no) + ".png", IMREAD_COLOR);
    detector.run(rgb_image, hands, human);


    //Uncomment below to activate gestue classifier

    /*auto net = cv::dnn::readNet("classifier_model/gesture.pb");
    Mat blob;
    Mat gray_hand;
    for (int i = 0; i < hands.size(); i++)
    {

        cvtColor(rgb_image(hands[i]), gray_hand, COLOR_BGR2GRAY);
        cv::dnn::blobFromImage(gray_hand*(1/255.0), blob, 1, cv::Size(64, 64), cv::Scalar(), true, false, CV_32F);
        net.setInput(blob);
        auto output = net.forward();
        if (output.at<float>(0, 1) > output.at<float>(0, 0))
            poi_hands.push_back(hands[i]);
    }*/
   
    if (rgb_image.cols != 640 || depth_image.cols!=640)
    {
        cout << "Error: Images are not appropriate." << endl;
        return 0;
    }

    cv::Point p1(left_x, top_y);
    cv::Point p2(right_x, bot_y);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = MatToPoinXYZ(depth_image);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr grounded = seg_ground(*cloud);

   // Mat dst = GroundCloudToMat(grounded, depth_image.cols, depth_image.rows);

    //change "depth_image" to "dst" for ground segmentation. Change "hands" to "poi_hands" for gesture classified hands.
    float angle_error = cloud_process(depth_image, rgb_image, line_point,x_dir,y_dir,z_dir,p1,p2,hands);
    cout << "Angular error is: " << angle_error << endl;
}
