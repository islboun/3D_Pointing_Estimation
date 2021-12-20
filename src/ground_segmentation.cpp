#include "ground_segmentation.h"




ground_seg::ground_seg(int in_num_iter, int in_num_lpr, double in_th_seeds, double in_th_dist) {

    this->num_iter = in_num_iter; //number of iteration to estimate plane
    this->num_lpr = in_num_lpr; //number of seed points
    this->th_seeds = in_th_seeds; // The initial height (the height of the sensor)
    this->th_dist = in_th_dist; // The confidence threshold for the test ground plane


}
bool point_cmp(pcl::PointXYZ a, pcl::PointXYZ b) {
    return (-a.y) < (-b.y); //comparision
}


void ground_seg::segment(pcl::PointCloud<pcl::PointXYZ> in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& not_ground_cloud) 
{


    pcl::PointCloud<pcl::PointXYZ>::Ptr g_seeds_pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_ground_pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_not_ground_pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_all_pc(new pcl::PointCloud<pcl::PointXYZ>());


    pcl::PointCloud<pcl::PointXYZ> all_points_cloud;
    pcl::PointCloud<pcl::PointXYZ> not_sorted_cloud;
    all_points_cloud = in_cloud;
    not_sorted_cloud = all_points_cloud;

    std::sort(all_points_cloud.points.begin(), all_points_cloud.end(), point_cmp);
    extract_initial_seeds(all_points_cloud, g_ground_pc);

    //g_ground_pc = g_seeds_pc;
    g_seeds_pc->clear();

    for (int i = 0; i < this->num_iter; i++) {


        double ground_threshold = estimate_plane(g_ground_pc);
        g_ground_pc->clear();
        g_not_ground_pc->clear();


        double result = 0;

        //threshold filter

        for (int r = 0; r < all_points_cloud.size(); r++) {
            double normalizer = sqrt(normal(0) * normal(0) + normal(1) * normal(1) + normal(2) * normal(2));
            result = (not_sorted_cloud.points[r].x * normal(0) + not_sorted_cloud.points[r].y * normal(1) + not_sorted_cloud.points[r].z * normal(2) + ground_threshold) / normalizer; // The distance between the point and the estimated  ground plane
            /*
            Testing the threshold
            If points is near the ground plane the points are labeled ground plane, otherwise, not ground plane

            */
            if (abs(result) <= th_dist)
            {


                g_ground_pc->points.push_back(not_sorted_cloud[r]);

            }
            else
            {
                g_not_ground_pc->points.push_back(not_sorted_cloud[r]);
            }
        }

    }

    ground_cloud->points.resize(g_ground_pc->size());
    not_ground_cloud->points.resize(g_not_ground_pc->size());

    for (size_t ground_it = 0; ground_it < g_ground_pc->size(); ground_it++)
        ground_cloud->points[ground_it] = g_ground_pc->points[ground_it];

    for (size_t non_ground_it = 0; non_ground_it < g_not_ground_pc->size(); non_ground_it++)
        not_ground_cloud->points[non_ground_it] = g_not_ground_pc->points[non_ground_it];


}






float ground_seg::estimate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_points) {

    Eigen::Matrix3f cov; //The covariance matrix for the estimation ground surface normal
    Eigen::Vector4f pc_mean; //The mean vector for the ground plane
    //pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    //viewer.showCloud(ground_points);
    //cv::waitKey(10000);


    pcl::computeMeanAndCovarianceMatrix(*ground_points, cov, pc_mean); // computation of the covariance and mean

    //svd

    JacobiSVD<MatrixXf> svd(cov, Eigen::ComputeFullU); //SVD for detecting surface normal

    //use the least singular vector as normal
    double minSingular = svd.singularValues()[0];
    int minInd = 0;
    for (int i = 1; i < svd.singularValues().size(); i++)
    {
        if (svd.singularValues()[i] < minSingular)
        {
            minSingular = svd.singularValues()[i];
            minInd = i;
        }

    }

    normal = (svd.matrixU().col(minInd)); //Normal vector
    //	cout<<"normal:"<<normal<<endl ;
    //mean ground seeds value

    Eigen::Vector3f seeds_mean = pc_mean.head<3>(); //The least significant vector(This is perpendicular to ground surface )


    d = -(normal.transpose() * seeds_mean)(0, 0); //The distance to the surface

    float  th_dist_d = d; // Test distance to find the last comparision or decision .
    //cout <<"th_dist_d:"<<th_dist_d<<endl;

    //return the equation parameters
    return th_dist_d;

}


void ground_seg::extract_initial_seeds(pcl::PointCloud<pcl::PointXYZ> p_sorted, pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_seeds)
{

    double sum = 0;


    /*

    Calculate average height
    */
    for (int i = 0; i < num_lpr; i++) {

        sum += -p_sorted.points[i].y;


    }

    double lpr_height = sum / num_lpr; //averaging
    ground_seeds->clear();

    //cout<<"lpr_height:"<<lpr_height<<endl;

    /*
    Finding initial seeds

    */
    for (int i = 0; i < p_sorted.points.size(); i++) {

        if (-p_sorted.points[i].y < lpr_height + th_seeds) {

            ground_seeds->points.push_back(p_sorted.points[i]);
        }

    }
}
