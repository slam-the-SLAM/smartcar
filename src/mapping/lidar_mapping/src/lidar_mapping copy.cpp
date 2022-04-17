#include "lidar_mapping.h"

LidarMapping::LidarMapping()
{
    InitROS();
}

void LidarMapping::FilterCloud(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out)
{
    ROS_DEBUG_STREAM("filter start, origin size is: " << cloud_in->size());
    if (cloud_in->empty())
        return;
    else {
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud_in);
        std::vector<int> indices;
#pragma omp for
        for(size_t i = 0; i < cloud_in->size(); i++) {
            double r = sqrt(pow(cloud_in->points[i].x, 2.0) + pow(cloud_in->points[i].y, 2.0));
            if (min_scan_range < r && r < max_scan_range) {
                indices.push_back(i);
            }
        }
        extract.setIndices(boost::make_shared<std::vector<int>>(indices));
        extract.setNegative(false); // 如果设为true,可以提取指定index之外的点云
        extract.filter(*cloud_in);
        pcl::VoxelGrid<PointT> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
        voxel_grid_filter.setInputCloud(cloud_in);
        voxel_grid_filter.filter(*cloud_out);
        ROS_DEBUG_STREAM("filter done, filtered size is: " << cloud_out->size());
        return;
    }
}

void LidarMapping::CalGuessPose(Pose6D &_guess_pose, Pose6D &_prev_pose, double _scan_duration, DiffVAW &_guess_offset)
{
    _scan_duration /= 10e3;
    ROS_DEBUG_STREAM("CalGuessPose start");
    _guess_pose.pos = _prev_pose.pos + _scan_duration * _guess_offset.vel + 0.5 * pow(_scan_duration, 2) * _guess_offset.acc;
    _guess_pose.orient = _guess_offset.twist;
    ROS_DEBUG_STREAM("CalGuessPose done");
}

void LidarMapping::CalDiffPose(Pose6D &_ndt_pose, Pose6D &_prev_pose, double _scan_duration, DiffVAW &_guess_offset)
{
    _scan_duration /= 10e3;
    ROS_DEBUG_STREAM("CalDiffPose start");
    _guess_offset.acc = 2 / (pow(_scan_duration, 2)) * (_ndt_pose.pos - _prev_pose.pos - (_guess_offset.vel * _scan_duration));
    _guess_offset.vel = (_ndt_pose.pos - _prev_pose.pos) / _scan_duration;
    _guess_offset.twist = _ndt_pose.orient;
    ROS_DEBUG_STREAM("CalDiffPose done");
}

// void LidarMapping::CalGuessPose(Pose6D &_guess_pose, Pose6D &_prev_pose, double _scan_duration, DiffVAW &_guess_offset)
// {
//     ROS_DEBUG_STREAM("CalGuessPose start");
//     _guess_pose.pos = _prev_pose.pos + _scan_duration * _guess_offset.vel + 0.5 * pow(_scan_duration, 2) * _guess_offset.acc;
    
//     int times = log(_scan_duration)/log(2);
//     ROS_DEBUG_STREAM("times is " << times);
//     Eigen::Quaterniond q_tmp = _guess_offset.twist;
// #pragma omp for
//     for(size_t i = 0; i < times; i++) q_tmp *= q_tmp;

//     ROS_DEBUG_STREAM("q_tmp is " << q_tmp.coeffs().transpose());
//     _guess_pose.orient = q_tmp * _prev_pose.orient;
//     ROS_DEBUG_STREAM("CalGuessPose done");
// }

// void LidarMapping::CalDiffPose(Pose6D &_ndt_pose, Pose6D &_prev_pose, double _scan_duration, DiffVAW &_guess_offset)
// {
//     ROS_DEBUG_STREAM("CalDiffPose start");

//     _guess_offset.acc = 2 / (pow(_scan_duration, 2)) * (_ndt_pose.pos - _prev_pose.pos - (_guess_offset.vel * _scan_duration));
//     _guess_offset.vel = (_ndt_pose.pos - _prev_pose.pos) / _scan_duration;

//     int times = log(_scan_duration)/log(2);
//     Eigen::Quaterniond q_tmp = _ndt_pose.orient;
//     CalDiffQ(times, _prev_pose.orient, q_tmp);
//     _guess_offset.twist = q_tmp*(_prev_pose.orient.inverse());
//     ROS_DEBUG_STREAM("_guess_offset.twist is " << _guess_offset.twist.coeffs().transpose());
//     ROS_DEBUG_STREAM("CalDiffPose done");
// }

void LidarMapping::CalDiffQ(int _times, Eigen::Quaterniond q_start, Eigen::Quaterniond& q_mid){
    if(_times == 0) return;
    else {
        _times--;
        ROS_DEBUG_STREAM("times is " << _times);
        ROS_DEBUG_STREAM(" q_start is " << q_start.coeffs().transpose());
        Eigen::Matrix4d M = q_start.coeffs() * q_start.coeffs().transpose() + q_mid.coeffs() * q_mid.coeffs().transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(M);
        q_mid.coeffs() = es.eigenvectors().col(3);
        ROS_DEBUG_STREAM(" q_mid is " << q_mid.coeffs().transpose());
        CalDiffQ(_times, q_start, q_mid);
        return;
    }
}

void LidarMapping::InitROS()
{
    ROS_DEBUG_STREAM("initializer");
    int _method_type;
    nh.param<int>("lidar_mapping/method_type", _method_type, 0);
    method_type = MethodType(_method_type);

    std::vector<double> tf_btol_v;
    nh.param<std::vector<double>>("lidar_mapping/tf_btol", tf_btol_v, std::vector<double>());
    tf_btol = Eigen::Map<Eigen::Matrix4d>(tf_btol_v.data()).transpose();
    tf_ltob = tf_btol.inverse();

    nh.param<float>("lidar_mapping/voxel_leaf_size", voxel_leaf_size, 2.0);
    nh.param<float>("lidar_mapping/ndt_res", ndt_res, 1.0);
    nh.param<double>("lidar_mapping/step_size", step_size, 0.1);
    nh.param<double>("lidar_mapping/trans_eps", trans_eps, 0.01);
    nh.param<int>("lidar_mapping/max_iter", max_iter, 1);
    nh.param<double>("lidar_mapping/min_scan_range", min_scan_range, 3.0);
    nh.param<double>("lidar_mapping/max_scan_range", max_scan_range, 200.0);
    nh.param<double>("lidar_mapping/min_add_scan_shift", min_add_scan_shift, 1.0);

    std::string cloud_input, save_input, cloud_output, pose_out;
    nh.param<std::string>("lidar_mapping/cloud_input", cloud_input, "/points_no_ground");
    nh.param<std::string>("lidar_mapping/save_input", save_input, "/save_map");
    nh.param<std::string>("lidar_mapping/map_pub", cloud_output, "/final_cloud");
    nh.param<std::string>("lidar_mapping/pose_pub", pose_out, "/pose_out");

    // prev_cloud.reset(new PointCloudT());
    final_cloud.reset(new PointCloudT());
    guess_offset.acc = Eigen::Vector3d::Zero();
    guess_offset.vel = Eigen::Vector3d::Zero();
    guess_offset.twist = Eigen::Quaterniond::Identity();

    cloud_sub = nh.subscribe(cloud_input, 1000, &LidarMapping::CloudCallback, this);
    save_sub = nh.subscribe(save_input, 10, &LidarMapping::SaveCallback, this);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_output, 100);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_out, 100);

    scan_duration_pub = nh.advertise<std_msgs::Float64>("scan_time",10);
    ndt_duration_pub = nh.advertise<std_msgs::Float64>("cal_time",10);
}

void LidarMapping::SaveCallback(const custom_msg::save::ConstPtr &save_msg_in)
{
    double filter_res = save_msg_in->filter_res;
    std::string filename = save_msg_in->filename;
    std::cout << "output_callback" << std::endl;
    std::cout << "filter_res: " << filter_res << std::endl;
    std::cout << "filename: " << filename << std::endl;

    if (filter_res == 0.0) {
        std::cout << "Original: " << final_cloud->points.size() << " points." << std::endl;
        pcl::io::savePCDFileASCII(filename, *final_cloud);
        std::cout << "Saved " << final_cloud->points.size() << " data points to " << filename << "." << std::endl;
    } else {
        PointCloudT::Ptr map_filtered(new PointCloudT());
        pcl::VoxelGrid<PointT> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
        voxel_grid_filter.setInputCloud(final_cloud);
        voxel_grid_filter.filter(*map_filtered);
        std::cout << "Original: " << final_cloud->points.size() << " points." << std::endl;
        std::cout << "Filtered: " << map_filtered->points.size() << " points." << std::endl;
        pcl::io::savePCDFileASCII(filename, *map_filtered);
        std::cout << "Saved " << map_filtered->points.size() << " data points to " << filename << "." << std::endl;
    }
}

void LidarMapping::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_in)
{

    ROS_DEBUG_STREAM("CloudCallback in");

    PointCloudT::Ptr cur_cloud(new PointCloudT);
    pcl::fromROSMsg(*cloud_msg_in, *cur_cloud);

    // Add initial point cloud to base
    if (initial_scan_loaded == 0) {
        ROS_DEBUG_STREAM("initial_scan_loaded");
        pcl::transformPointCloud(*cur_cloud, *cur_cloud, tf_btol);
        *final_cloud += *cur_cloud;
        // *prev_cloud = *cur_cloud;
        prev_time = cloud_msg_in->header.stamp;
        initial_scan_loaded = 1;
        return;
    }

    PointCloudT::Ptr filtered_cloud(new PointCloudT);
    FilterCloud(cur_cloud, filtered_cloud);

    std_msgs::Float64 scan_duration;
    scan_duration.data = (cloud_msg_in->header.stamp - prev_time).toNSec();
    ROS_DEBUG_STREAM("scan_duration is: " << scan_duration.data/10e6 << " ms");
    scan_duration_pub.publish(scan_duration);
    prev_time = cloud_msg_in->header.stamp;

    clock_t cal_guess_pose_start = ros::Time::now().toNSec();
    CalGuessPose(guess_pose, prev_pose, scan_duration.data/10e6, guess_offset);
    clock_t cal_guess_pose_end = ros::Time::now().toNSec();
    ROS_DEBUG_STREAM("cal_guess_pose_duration is: " << (cal_guess_pose_end - cal_guess_pose_start)/10e6 << " ms");
    
    Eigen::Translation3d _t(guess_pose.pos);//平移矩阵预测值
    Eigen::Matrix4d guess_transform = (_t * guess_pose.orient.toRotationMatrix()).matrix();
    ROS_DEBUG_STREAM("guess_transform is : \n" << guess_transform);
    
    double fitness_score;
    bool has_converged;
    int final_num_iteration;
    double transformation_probability;
    Eigen::Matrix4d ndt_transform;
    PointCloudT::Ptr temp_cloud(new PointCloudT);
    
    clock_t ndt_start = ros::Time::now().toNSec();
    if (method_type == MethodType::PCL_NDT) {
        ROS_DEBUG_STREAM("ndt config start!");
        ndt.setTransformationEpsilon(trans_eps);
        ndt.setStepSize(step_size);
        ndt.setResolution(ndt_res);
        ndt.setMaximumIterations(max_iter);
        ndt.setInputSource(filtered_cloud);
        // ndt.setInputTarget(prev_cloud);
        ndt.setInputTarget(final_cloud);
        ROS_DEBUG_STREAM("ndt config done!");
        ROS_DEBUG_STREAM("ndt align start!");
        ndt.align(*temp_cloud, guess_transform.cast<float>());
        ROS_DEBUG_STREAM("ndt align done!");
        fitness_score = ndt.getFitnessScore();
        ndt_transform = ndt.getFinalTransformation().cast<double>();
        has_converged = ndt.hasConverged();
        final_num_iteration = ndt.getFinalNumIteration();
        transformation_probability = ndt.getTransformationProbability();
    }
    clock_t ndt_end = ros::Time::now().toNSec();
    std_msgs::Float64 ndt_duration;
    ndt_duration.data = (ndt_end - ndt_start)/10e6;
    ROS_DEBUG_STREAM("ndt_duration is: " << ndt_duration.data << " ms");
    ndt_duration_pub.publish(ndt_duration);

    PointCloudT::Ptr transformed_cloud(new PointCloudT);
    pcl::transformPointCloud(*cur_cloud, *transformed_cloud, ndt_transform);
    // ROS_DEBUG_STREAM("transform_cloud size is: " << transformed_cloud->size());
    // *prev_cloud = *cur_cloud;

    Eigen::Affine3d aff_base_link(ndt_transform * tf_ltob);
    ndt_pose.pos = aff_base_link.translation();
    ndt_pose.orient = Eigen::Quaterniond(aff_base_link.rotation());

    clock_t cal_diff_pose_start = ros::Time::now().toNSec();
    CalDiffPose(ndt_pose, prev_pose, scan_duration.data/10e6, guess_offset);
    clock_t cal_diff_pose_end = ros::Time::now().toNSec();
    ROS_DEBUG_STREAM("cal_diff_pose_duration is: " << (cal_diff_pose_end - cal_diff_pose_start)/10e6 << " ms");
    prev_pose = ndt_pose;

    // Calculate the shift between added_pos and current_pos
    double shift = sqrt(pow(ndt_pose.pos.x() - fixed_pose.pos.x(), 2.0) + pow(ndt_pose.pos.y() - fixed_pose.pos.y(), 2.0));
    if (shift >= min_add_scan_shift) {
        ROS_DEBUG_STREAM("shift >= min_add_scan_shift");
        *final_cloud += *transformed_cloud;
        fixed_pose = ndt_pose;
    }
    ROS_DEBUG_STREAM("final_cloud size is: " << final_cloud->size());

    sensor_msgs::PointCloud2::Ptr map_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*final_cloud, *map_msg);
    map_msg->header.frame_id = "map";
    map_msg->header.stamp = cloud_msg_in->header.stamp;
    map_pub.publish(*map_msg);

    geometry_msgs::PoseStamped current_pose_msg;
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = cloud_msg_in->header.stamp;
    current_pose_msg.pose.position.x = ndt_pose.pos.x();
    current_pose_msg.pose.position.y = ndt_pose.pos.y();
    current_pose_msg.pose.position.z = ndt_pose.pos.z();
    current_pose_msg.pose.orientation.x = ndt_pose.orient.x();
    current_pose_msg.pose.orientation.y = ndt_pose.orient.y();
    current_pose_msg.pose.orientation.z = ndt_pose.orient.z();
    current_pose_msg.pose.orientation.w = ndt_pose.orient.w();

    pose_pub.publish(current_pose_msg);

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence number: " << cloud_msg_in->header.seq << std::endl;
    std::cout << "Number of scan points: " << cur_cloud->size() << " points." << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_cloud->size() << " points." << std::endl;
    std::cout << "transformed_scan_ptr: " << transformed_cloud->points.size() << " points." << std::endl;
    std::cout << "map: " << final_cloud->points.size() << " points." << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "Number of iteration: " << final_num_iteration << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    Eigen::Vector3d ea = ndt_pose.orient.matrix().eulerAngles(2, 1, 0);
    std::cout << "(" << ndt_pose.pos.x() << ", " << ndt_pose.pos.y() << ", " << ndt_pose.pos.z() << ", " << 57.295779515 * ea[2]
              << ", " << 57.295779515 * ea[1] << ", " << 57.295779515 * ea[0] << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << ndt_transform << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
}