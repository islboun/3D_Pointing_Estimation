#include "util.h"


using namespace cv;
using namespace std;
pcl::PointCloud<pcl::PointXYZ>::Ptr MatToPoinXYZ(cv::Mat depthMat)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud(new pcl::PointCloud<pcl::PointXYZ>);
    unsigned short* data = (unsigned short*)(depthMat.data);

    for (int i = 0; i < depthMat.rows; i++)
    {
        for (int j = 0; j < depthMat.cols; j++)
        {
            float p = float(depthMat.at<ushort>(i, j)) / 1000;
            if (p == 0)
                continue;
            pcl::PointXYZ point;
            float x_multiplier = (j - RGB_C_X) / RGB_FOCAL_X;
            float y_multiplier = (i-RGB_C_Y) / RGB_FOCAL_Y;
            point.z = ((float)(p)) / sqrt(x_multiplier * x_multiplier + y_multiplier * y_multiplier + 1);
            point.y = point.z * y_multiplier;
            point.x = point.z * x_multiplier;

            ptCloud->points.push_back(point);
            data++;
        }
    }
    ptCloud->width = (int)depthMat.cols;
    ptCloud->height = (int)depthMat.rows;
    return ptCloud;

}

Mat CloudtoMat(pcl::PointCloud<pcl::PointXYZRGB> cloud, int height, int width)
{
    Mat image = cv::Mat(height, width, CV_8UC3, Scalar(0));
#pragma omp parallel for
    for (int i = 0; i < cloud.size(); i++)
    {
        float x_world = cloud.points[i].x;
        float y_world = cloud.points[i].y;
        float z_world = cloud.points[i].z;
        float x_rgbcam = RGB_FOCAL_X * x_world / z_world + RGB_C_X;
        float y_rgbcam = RGB_FOCAL_Y * y_world / z_world + RGB_C_Y;

        // "Interpolate" pixel coordinates 
        int px_rgbcam = cvRound(x_rgbcam);
        int py_rgbcam = cvRound(y_rgbcam);
        if (px_rgbcam > 0 && px_rgbcam < width && py_rgbcam>0 && py_rgbcam < height)
        {
            
            image.at<Vec3b>(py_rgbcam, px_rgbcam)[0] = cloud.points[i].b;
            image.at<Vec3b>(py_rgbcam, px_rgbcam)[1] = cloud.points[i].g;
            image.at<Vec3b>(py_rgbcam, px_rgbcam)[2] = cloud.points[i].r;
        }
    }
    return image;
}

Mat GroundCloudToMat(pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud, int width, int height)
{
    Mat image(cv::Size(width, height), CV_16UC1, Scalar(0));

    for (int i = 0; i < ptCloud->size(); i++)
    {

        float x_multiplier = ptCloud->points[i].x / ptCloud->points[i].z;
        float y_multiplier = ptCloud->points[i].y / ptCloud->points[i].z;
        int x = round((x_multiplier)*RGB_FOCAL_X + RGB_C_X);
        int y = round((y_multiplier)*RGB_FOCAL_Y + RGB_C_Y);
        float z = ptCloud->points[i].z * sqrt(x_multiplier * x_multiplier + y_multiplier * y_multiplier + 1);
        if (0 <= y && y < height && 0 <= x && x < width)
        {
            image.at<unsigned short>(y, x) = 1000 * z;
        }

    }
    return image;
}

pcl::PointXYZRGB get3Dcoord(float x, float y)
{

}

cv::Point get2Dcoord(pcl::PointXYZRGB cloud_point)
{
    float x_rgbcam = RGB_FOCAL_X * cloud_point.x / cloud_point.z + RGB_C_X;
    float y_rgbcam = RGB_FOCAL_Y * cloud_point.y / cloud_point.z + RGB_C_Y;

    // "Interpolate" pixel coordinates 
    int px_rgbcam = cvRound(x_rgbcam);
    int py_rgbcam = cvRound(y_rgbcam);
    return Point(px_rgbcam, py_rgbcam);
}


pcl::PointCloud<pcl::PointXYZ> pointXYZRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB> rgbcloud)
{
    pcl::PointCloud<pcl::PointXYZ> xyzcloud;

    for (int i=0;i<rgbcloud.size();i++)
    {
        pcl::PointXYZ point;
        point.x=rgbcloud[i].x;
        point.y=rgbcloud[i].y;
        point.z=rgbcloud[i].z;
        xyzcloud.push_back(point);
    }
    return xyzcloud;

}
