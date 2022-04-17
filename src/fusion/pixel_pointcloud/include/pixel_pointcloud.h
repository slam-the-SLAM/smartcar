#ifndef __PIXEL_POINTCLOUD_H__
#define __PIXEL_POINTCLOUD_H__

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>

class pixelPointCloud {
    //* 类中不写修饰符默认为private
    ros::NodeHandle nh;

    //* image_transport与ros区别
    image_transport::Subscriber sub_image_raw;
    image_transport::Publisher pub_image_rect;
    image_transport::Subscriber sub_image_aruco;
    image_transport::Publisher pub_image_aruco;
    image_transport::Publisher pub_image_depth;
    image_transport::Publisher pub_image_align_depth;

    ros::Subscriber sub_cloud_raw;
    ros::Publisher pub_cloud_transformed;
    // ros::Subscriber sub_cloud_transformed;
    ros::Publisher pub_cloud_colored;

    message_filters::Subscriber<sensor_msgs::Image> sub_image_rect;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_transformed;
    //* 注意此处选择指针形式，是因为需要在类中声明sync，要不无法进入回调函数
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2>* sync;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy>* sync;


    int image_width;
    int image_height;
    Eigen::Matrix3d camera_instrinsics;
    // Eigen::Matrix<double, 1, 5> distortion_coefficients;
    Eigen::Matrix<double, 1, 4> distortion_coefficients;

    Eigen::Matrix4d camera_lidar_extrinsic;

    void image_Callback(const sensor_msgs::Image::ConstPtr& image_msg_in);
    void aruco_Callback(const sensor_msgs::Image::ConstPtr& image_msg_in);
    void cloud_Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in);
    //* Image::ConstPtr与ImageConstPtr区别
    void sync_Callback(const sensor_msgs::Image::ConstPtr& image_msg_in, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in);
    // void sync_Callback(const sensor_msgs::ImageConstPtr& image_msg_in, const sensor_msgs::PointCloud2ConstPtr& cloud_msg_in);
    void initializer();
public:
    pixelPointCloud();
    cv::Mat camera_instrinsics_cv;
    cv::Mat distortion_coefficients_cv;
    cv::Mat map1,map2;

    // inline void set_Image_Size(const cv::Size& _image_size) {image_size = _image_size;}
    // inline void set_Camera_Instrinsics(const cv::Mat& _camera_instrinsics) {camera_instrinsics = _camera_instrinsics;}
    // inline void set_Distortion_Coefficients(const cv::Mat& _distortion_coefficients) {distortion_coefficients = _distortion_coefficients;}
    // inline void set_Camera_Lidar_Extrinsic(const cv::Mat& _camera_lidar_extrinsic) {camera_lidar_extrinsic = _camera_lidar_extrinsic;}

};

#endif
