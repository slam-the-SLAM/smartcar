#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

Eigen::Matrix4d _R;
// Eigen::Affine3d R_aff(_R);
ros::Publisher pub_cloud_rotated;
// ros::Publisher pub_image;

// void callback(const sensor_msgs::ImageConstPtr &image_msgs, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in)
// {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*cloud_msg_in, *cloud_in);

//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

//     Eigen::Affine3d R_aff(_R);
//     // std::cout << R_aff.matrix() << std::endl;

//     pcl::transformPointCloud(*cloud_in, *cloud_out, R_aff);

//     // *cloud_out = *cloud_in;

//     sensor_msgs::PointCloud2Ptr cloud_msg_out(new sensor_msgs::PointCloud2);
//     pcl::toROSMsg(*cloud_out, *cloud_msg_out);
//     cloud_msg_out->header.frame_id = "velodyne";
//     // cloud_msg_out->header.frame_id = cloud_msg_in->header.frame_id;
//     // cloud_msg_out->header.stamp = cloud_msg_in->header.stamp;
//     pub_cloud_rotated.publish(cloud_msg_out);
//     pub_image.publish(image_msgs);
// }

void cloud_Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in){
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg_in, *cloud_in);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Affine3d R_aff(_R);
    // std::cout << R_aff.matrix() << std::endl;

    pcl::transformPointCloud(*cloud_in, *cloud_out, R_aff);

    // *cloud_out = *cloud_in;

    sensor_msgs::PointCloud2Ptr cloud_msg_out(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_out, *cloud_msg_out);
    cloud_msg_out->header.frame_id = "velodyne";
    // cloud_msg_out->header.frame_id = cloud_msg_in->header.frame_id;
    cloud_msg_out->header.stamp = cloud_msg_in->header.stamp;
    pub_cloud_rotated.publish(cloud_msg_out);

}

int main(int argc, char **argv)
{
    _R <<   -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    ros::init(argc, argv, "pointcloud_rotate");

    ros::NodeHandle nh;

    // message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/color/image_raw", 10);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, "/hesai/pandar", 10);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub, pointcloud_sub);
    // sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber sub_cloud_in  = nh.subscribe<sensor_msgs::PointCloud2>("/hesai/pandar", 10, &cloud_Callback);

    // pub_image = nh.advertise<sensor_msgs::Image>("/image_raw", 10);
    pub_cloud_rotated = nh.advertise<sensor_msgs::PointCloud2>("/points_raw", 10);

    ros::spin();

    return 0;
}