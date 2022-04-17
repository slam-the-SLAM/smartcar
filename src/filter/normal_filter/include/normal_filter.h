#ifndef __NORMAL_FILTER__
#define __NORMAL_FILTER__

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

#define PI 3.14159265
#define PRAD 180/PI

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZINormal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;


class NormalFilter{    

    double search_radius, thershold_max, thershold_min, height;
    std::vector<double> normal_v;
    Eigen::Matrix4d T;
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    pcl::search::KdTree<PointT>::Ptr tree;
    
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher transform_pub;
    ros::Publisher crop_pub;
    ros::Publisher theta_pub;
    ros::Publisher curvature_pub;
    ros::Publisher ground_pub;
    ros::Publisher no_ground_pub;

    void InitROS();
    void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in);
    void CalThetaDiff(PointCloudT::Ptr& cloud_in, NormalCloudT::Ptr& normal_in, PointCloudT::Ptr& cloud_out);
    void CalCurvature(PointCloudT::Ptr& cloud_in, NormalCloudT::Ptr& normal_in, PointCloudT::Ptr& cloud_out);
    void FilterCloud(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_false, PointCloudT::Ptr& cloud_true);
public:
    NormalFilter();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif