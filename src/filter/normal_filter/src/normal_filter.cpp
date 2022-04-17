#include "normal_filter.h"

NormalFilter::NormalFilter() {
    InitROS();
}

void NormalFilter::InitROS() {
    ROS_DEBUG_STREAM("init ros");

    nh.param<double>("normal_filter/search_radius", search_radius, 0.5);
    nh.param<double>("normal_filter/thershold_min", thershold_min, 0);
    nh.param<double>("normal_filter/thershold_max", thershold_max, 10);
    nh.param<double>("normal_filter/height", height, 1.6);
    nh.param<std::vector<double>>("normal_filter/normal_v", normal_v, std::vector<double>(4));

    std::string cloud_input, cloud_theta, cloud_curvature, cloud_no_ground, cloud_ground, cloud_transform, cloud_crop;
    nh.param<std::string>("normal_filter/cloud_input", cloud_input, "/points_raw");
    nh.param<std::string>("normal_filter/cloud_transform", cloud_transform, "/cloud_transform");
    nh.param<std::string>("normal_filter/cloud_crop", cloud_crop, "/cloud_crop");
    nh.param<std::string>("normal_filter/cloud_theta", cloud_theta, "/cloud_theta");
    nh.param<std::string>("normal_filter/cloud_curvature", cloud_curvature, "/cloud_curvature");
    nh.param<std::string>("normal_filter/cloud_ground", cloud_ground, "/cloud_ground");
    nh.param<std::string>("normal_filter/cloud_no_ground", cloud_no_ground, "/cloud_no_ground");


    cloud_sub = nh.subscribe(cloud_input, 100, &NormalFilter::CloudCallback, this);
    transform_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_transform, 100);
    crop_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_crop, 100);
    theta_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_theta, 100);
    curvature_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_curvature, 100);
    ground_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_ground, 100);
    no_ground_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_no_ground, 100);

    tree.reset((new pcl::search::KdTree<PointT>()));

    double norm = normal_v[0]*normal_v[0]+normal_v[1]*normal_v[1]+normal_v[2]*normal_v[2];
    double theta = acos(normal_v[2]/norm);
    Eigen::Vector3d axis;
    axis << normal_v[1], -normal_v[0], 0;
    axis.normalize();
    Eigen::AngleAxisd aa(theta,axis);
    Eigen::Translation3d _t(0,0,0);
    T = (_t*aa.toRotationMatrix()).matrix();
}

void NormalFilter::CalThetaDiff(PointCloudT::Ptr& cloud_in, NormalCloudT::Ptr& normal_in, PointCloudT::Ptr& cloud_out) {

    if (cloud_in->empty() || normal_in->empty()) return;
    else {
#pragma omp for
        for(size_t i=0; i<cloud_in->size(); i++) {
            PointT p = cloud_in->points[i];
            p.intensity = acos(normal_in->points[i].normal_x * normal_v[0] + normal_in->points[i].normal_y * normal_v[1] + normal_in->points[i].normal_z * normal_v[2]);
            p.intensity *= PRAD;
            cloud_out->points.push_back(p);
        }
        return;
    }
}

void NormalFilter::CalCurvature(PointCloudT::Ptr& cloud_in, NormalCloudT::Ptr& normal_in, PointCloudT::Ptr& cloud_out) {
    if (cloud_in->empty() || normal_in->empty()) return;
    else {
#pragma omp for
        for(size_t i=0; i<cloud_in->size(); i++) {
            PointT p = cloud_in->points[i];
            p.intensity = normal_in->points[i].curvature;
            cloud_out->points.push_back(p);
        }
        return;
    }
}

void NormalFilter::FilterCloud(PointCloudT::Ptr& cloud_in, PointCloudT::Ptr& cloud_false, PointCloudT::Ptr& cloud_true){
    if (cloud_in->empty()) return;
    else {
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in);
        std::vector<int> indices;
#pragma omp for
        for(size_t i = 0; i < cloud_in->size(); i++) {
            if (thershold_min < cloud_in->points[i].intensity && cloud_in->points[i].intensity < thershold_max) {
                indices.push_back(i);
            }
        }
        ROS_DEBUG_STREAM("indices size: " << indices.size());
        extract.setIndices(boost::make_shared<std::vector<int>>(indices));
        extract.setNegative(false); // 如果设为true,可以提取指定index之外的点云
        extract.filter(*cloud_false);

        extract.setNegative(true);
        extract.filter(*cloud_true);
        return;
    }
}

void NormalFilter::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in) {
    ROS_DEBUG_STREAM("cloud callback");
    
    PointCloudT::Ptr cloud_raw(new PointCloudT());
    pcl::fromROSMsg(*cloud_msg_in, *cloud_raw);

    PointCloudT::Ptr cloud_transform(new PointCloudT());
    pcl::transformPointCloud(*cloud_raw, *cloud_transform, T);
    sensor_msgs::PointCloud2::Ptr cloud_transform_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_transform, *cloud_transform_msg);
    cloud_transform_msg->header.frame_id = cloud_msg_in->header.frame_id;
    cloud_transform_msg->header.stamp = cloud_msg_in->header.stamp;
    transform_pub.publish(*cloud_transform_msg);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_transform);
    std::vector<int> indices;
#pragma omp for
    for(size_t i=0; i<cloud_transform->size(); i++){
        if(cloud_transform->points[i].z > height) indices.push_back(i);
    }
    extract.setIndices(boost::make_shared<std::vector<int>>(indices));
    extract.setNegative(true);
    PointCloudT::Ptr cloud_crop(new PointCloudT());
    extract.filter(*cloud_crop);
    sensor_msgs::PointCloud2::Ptr cloud_crop_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_crop, *cloud_crop_msg);
    cloud_crop_msg->header.frame_id = cloud_msg_in->header.frame_id;
    cloud_crop_msg->header.stamp = cloud_msg_in->header.stamp;
    crop_pub.publish(*cloud_crop_msg);

    ne.setNumberOfThreads(16);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(search_radius);
    ne.setInputCloud(cloud_raw);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    NormalCloudT::Ptr cloud_normal(new NormalCloudT());
    ne.compute (*cloud_normal);

    PointCloudT::Ptr cloud_theta(new PointCloudT());
    CalThetaDiff(cloud_raw, cloud_normal, cloud_theta);

    sensor_msgs::PointCloud2::Ptr cloud_theta_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_theta, *cloud_theta_msg);
    cloud_theta_msg->header.frame_id = cloud_msg_in->header.frame_id;
    cloud_theta_msg->header.stamp = cloud_msg_in->header.stamp;
    theta_pub.publish(*cloud_theta_msg);
    
    PointCloudT::Ptr cloud_curvature(new PointCloudT());
    CalCurvature(cloud_raw, cloud_normal, cloud_curvature);

    sensor_msgs::PointCloud2::Ptr cloud_curvature_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_curvature, *cloud_curvature_msg);
    cloud_curvature_msg->header.frame_id = cloud_msg_in->header.frame_id;
    cloud_curvature_msg->header.stamp = cloud_msg_in->header.stamp;
    curvature_pub.publish(*cloud_curvature_msg);
    
    ROS_DEBUG_STREAM("thershold max: " << thershold_max);
    ROS_DEBUG_STREAM("thershold min: " << thershold_min);
    PointCloudT::Ptr cloud_ground(new PointCloudT());
    PointCloudT::Ptr cloud_no_ground(new PointCloudT());
    FilterCloud(cloud_theta, cloud_ground, cloud_no_ground);

    sensor_msgs::PointCloud2::Ptr cloud_ground_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_ground, *cloud_ground_msg);
    cloud_ground_msg->header.frame_id = cloud_msg_in->header.frame_id;
    cloud_ground_msg->header.stamp = cloud_msg_in->header.stamp;
    ground_pub.publish(*cloud_ground_msg);

    sensor_msgs::PointCloud2::Ptr cloud_no_ground_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_no_ground, *cloud_no_ground_msg);
    cloud_no_ground_msg->header.frame_id = cloud_msg_in->header.frame_id;
    cloud_no_ground_msg->header.stamp = cloud_msg_in->header.stamp;
    no_ground_pub.publish(*cloud_no_ground_msg);
}