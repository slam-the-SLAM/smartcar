#ifndef __LIDAR_MAPPING__
#define __LIDAR_MAPPING__

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>

// #include <pcl/registration/icp_nl.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <omp.h>

#include "custom_msg/save.h"

struct Pose6D{
    Eigen::Vector3d pos;
    Eigen::Quaterniond orient;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct DiffVAW{
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Quaterniond twist;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum MethodType{
    PCL_NDT = 0
};

class LidarMapping{
    
    Pose6D prev_pose, guess_pose, ndt_pose, fixed_pose; // need to prune
    DiffVAW guess_offset;
    PointCloudT::Ptr prev_cloud;
    PointCloudT::Ptr final_cloud;
    ros::Time prev_time;
    MethodType method_type;
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    // pcl::IterativeClosestPoint<PointT, PointT> icp;
    pcl::VoxelGrid<PointT> voxel_grid_filter;
    // config parameters to fill

    bool initial_scan_loaded = 0;
    Eigen::Matrix4d tf_btol, tf_ltob;

    float voxel_leaf_size;
    float ndt_res;      // Resolution
    double step_size;   // Step size
    double trans_eps;  // Transformation epsilon
    int max_iter;

    double min_scan_range;
    double max_scan_range;
    double min_add_scan_shift;

    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Subscriber save_sub;
    ros::Publisher map_pub;
    ros::Publisher pose_pub;

    // debug pub
    ros::Publisher scan_duration_pub;
    ros::Publisher ndt_duration_pub;

    void InitROS();
    void FilterCloud(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr &cloud_out);
    void CalGuessPose(Pose6D& _guess_pose, Pose6D& _prev_pose, double _scan_duration, DiffVAW& _guess_offset);
    void CalDiffPose(Pose6D& _ndt_pose, Pose6D& _prev_pose, double _scan_duration, DiffVAW& _guess_offset);
    void CalDiffQ(int _times, Eigen::Quaterniond q1, Eigen::Quaterniond& q_delta);
    void SaveCallback(const custom_msg::save::ConstPtr& save_msg_in);
    void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in);
public:
    LidarMapping();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif