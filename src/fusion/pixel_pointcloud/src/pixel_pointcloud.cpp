#include "pixel_pointcloud.h"

// pixelPointCloud::pixelPointCloud():nh("~")
pixelPointCloud::pixelPointCloud() {
    initializer();
}

void pixelPointCloud::initializer(){

    std::cout << "initializer" << std::endl;

    std::string image_raw_input, image_rect_output, image_aruco_output, image_depth_output,image_depth_align_output;
    nh.param<std::string>("pixel_pointcloud/image_raw_input", image_raw_input, "/image_raw");
    nh.param<std::string>("pixel_pointcloud/image_rect_output", image_rect_output, "/image_rect");
    nh.param<std::string>("pixel_pointcloud/image_aruco_output", image_aruco_output, "/image_aruco");
    nh.param<std::string>("pixel_pointcloud/image_depth_output", image_depth_output, "/image_depth");
    nh.param<std::string>("pixel_pointcloud/image_depth_align_output", image_depth_align_output, "/image_depth_align");

    nh.param<int>("pixel_pointcloud/image_width", image_width, 1280);
    nh.param<int>("pixel_pointcloud/image_height", image_height, 720);

    std::vector<double> camera_instrinsics_v, distortion_coefficients_v;
    nh.param< std::vector<double> >("pixel_pointcloud/camera_instrinsics", camera_instrinsics_v, std::vector<double>());
    nh.param< std::vector<double> >("pixel_pointcloud/distortion_coefficients", distortion_coefficients_v, std::vector<double>());

    image_transport::ImageTransport it(nh);
    sub_image_raw = it.subscribe(image_raw_input, 10, &pixelPointCloud::image_Callback, this);
    pub_image_rect = it.advertise(image_rect_output, 10);
    // sub_image_aruco = it.subscribe(image_raw_input, 10, &pixelPointCloud::aruco_Callback, this);
    // pub_image_aruco = it.advertise(image_aruco_output, 10);
    pub_image_depth = it.advertise(image_depth_output, 10);
    pub_image_align_depth = it.advertise(image_depth_align_output, 10);

    camera_instrinsics = Eigen::Map<Eigen::Matrix3d>(camera_instrinsics_v.data()).transpose();
    distortion_coefficients = Eigen::Map< Eigen::Matrix<double, 1, 4> >(distortion_coefficients_v.data()).transpose();
    // distortion_coefficients = Eigen::Map< Eigen::Matrix<double, 1, 5> >(distortion_coefficients_v.data()).transpose();

    camera_instrinsics_cv = cv::Mat(3, 3, CV_64F);
    // cv::Mat distortion_coefficients_cv = cv::Mat(1, 5, CV_64F);
    distortion_coefficients_cv = cv::Mat(1, 4, CV_64F);
    cv::eigen2cv(camera_instrinsics, camera_instrinsics_cv);
    cv::eigen2cv(distortion_coefficients, distortion_coefficients_cv);
    map1,map2;


    std::string cloud_raw_input, cloud_transformed_output, cloud_colored_output;
    nh.param<std::string>("pixel_pointcloud/cloud_raw_input", cloud_raw_input, "/cloud_raw");
    nh.param<std::string>("pixel_pointcloud/cloud_transformed_output", cloud_transformed_output, "/cloud_transformed");
    nh.param<std::string>("pixel_pointcloud/cloud_colored_output", cloud_colored_output, "/cloud_colored");

    std::vector<double> camera_lidar_extrinsic_v;
    nh.param< std::vector<double> >("pixel_pointcloud/camera_lidar_extrinsic", camera_lidar_extrinsic_v, std::vector<double>());
    camera_lidar_extrinsic = Eigen::Map<Eigen::Matrix4d>(camera_lidar_extrinsic_v.data()).transpose();

    sub_cloud_raw = nh.subscribe(cloud_raw_input, 10, &pixelPointCloud::cloud_Callback, this);
    pub_cloud_transformed = nh.advertise<sensor_msgs::PointCloud2>(cloud_transformed_output, 10);
    pub_cloud_colored = nh.advertise<sensor_msgs::PointCloud2>(cloud_colored_output, 10);

    sub_image_rect.subscribe(nh,image_rect_output,10);
    sub_cloud_transformed.subscribe(nh,cloud_transformed_output,10);

    //* 注意sync初始化形式

    sync = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(10), sub_image_rect, sub_cloud_transformed);
    // sync = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2>(sub_image_rect, sub_cloud_transformed, 10);
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(sub_image_rect, sub_cloud_transformed, 10);

    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image_rect, sub_cloud_transformed);

    //* 注意此处bind函数内部，与非class函数的区别写法
    // sync.registerCallback(boost::bind(&pixelPointCloud::sync_Callback, _1, _2));
    sync->registerCallback(boost::bind(&pixelPointCloud::sync_Callback, this, _1, _2));

}

void pixelPointCloud::image_Callback(const sensor_msgs::Image::ConstPtr& image_msg_in){

    // std::cout << "rect" << std::endl;
    cv_bridge::CvImagePtr image_cv = cv_bridge::toCvCopy(image_msg_in, "bgr8");
    cv::Mat image_in = image_cv->image;

    // cv::Mat camera_instrinsics_cv = cv::Mat(3, 3, CV_64F);
    // // cv::Mat distortion_coefficients_cv = cv::Mat(1, 5, CV_64F);
    // cv::Mat distortion_coefficients_cv = cv::Mat(1, 4, CV_64F);
    // cv::eigen2cv(camera_instrinsics, camera_instrinsics_cv);
    // cv::eigen2cv(distortion_coefficients, distortion_coefficients_cv);
    // cv::Mat map1,map2;

    cv::Mat image_rect;
    // cv::undistort(image_in, image_rect, camera_instrinsics_cv, distortion_coefficients_cv);
    // cv::fisheye::undistortImage(image_in, image_rect, camera_instrinsics_cv, distortion_coefficients_cv);
    cv::fisheye::initUndistortRectifyMap(camera_instrinsics_cv, distortion_coefficients_cv, cv::Mat(), camera_instrinsics_cv, cv::Size(1280, 720), CV_32FC1, map1, map2);
    cv::remap(image_in, image_rect, map1, map2, cv::INTER_LINEAR);
    
    // cv::imshow("a", image_rect);
    // cv::waitKey(1);


    sensor_msgs::ImagePtr image_msg_rect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_rect).toImageMsg();
    image_msg_rect->header.frame_id = "sensor";
    // image_msg_rect->header.stamp = image_msg_in->header.stamp;
    image_msg_rect->header.stamp = ros::Time::now();
    pub_image_rect.publish(image_msg_rect);

}

void pixelPointCloud::aruco_Callback(const sensor_msgs::Image::ConstPtr& image_msg_in){
    cv_bridge::CvImagePtr image_cv = cv_bridge::toCvCopy(image_msg_in, "bgr8");
    cv::Mat image_in = image_cv->image;

    cv::Mat camera_instrinsics_cv = cv::Mat(3, 3, CV_64F);
    cv::eigen2cv(camera_instrinsics, camera_instrinsics_cv);
    // cv::eigen2cv(distortion_coefficients, distortion_coefficients_cv);
    cv::Mat distortion_coefficients_cv = (cv::Mat_<double>(4, 1) << -0.196668, 0.022176, 0.003651, -0.001057);
    aruco::CameraParameters camera_para;
    camera_para.setParams(camera_instrinsics_cv, distortion_coefficients_cv, cv::Size(image_width, image_height));
    double aruco_size = 0.20;
    try{
        // aruco::MarkerDetector_Impl Detector_imp;
        aruco::MarkerDetector Detector;
        Detector.setDictionary("ARUCO");
        cv::Mat image_out = image_in;
        std::vector<aruco::Marker> markers;
        markers.clear();
        Detector.detect(image_out, markers, camera_para, aruco_size);
        // if(markers.size() == 0) {
        //     // ROS_INFO("No marker found!");
        //     return;
        // }
        // ROS_INFO("markers:[%d]", markers.size());
        for(size_t i = 0; i < markers.size(); i++){
            // ROS_INFO("markers:[%d],i:[%d]", markers.size(), i);
            markers[i].draw(image_out, cv::Scalar(0,0,255),2);
            aruco::CvDrawingUtils::draw3dAxis(image_out, markers[i], camera_para);

            Eigen::Vector3d R,t; 
            cv::cv2eigen(markers[i].Rvec, R);
            // cv::cv2eigen(markers[i].Tvec, t);
            // Eigen::AngleAxisd R_aa(R.norm(),R.normalized());
            // Eigen::Matrix3d R_m = R_aa.toRotationMatrix();
            // Eigen::Vector3d R_ea = R_m.eulerAngles(2,1,0);
            // std::cout << R_ea.transpose() << std::endl;
            // std::cout << t.transpose() << std::endl;
        }
        sensor_msgs::ImagePtr image_msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_out).toImageMsg();
        image_msg_out->header.frame_id = "sensor";
        // image_msg_out->header.stamp = image_msg_in->header.stamp;
        image_msg_out->header.stamp = ros::Time::now();
        pub_image_aruco.publish(image_msg_out);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void pixelPointCloud::cloud_Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in){
    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg_in, *cloud_in);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Affine3d camera_lidar_extrinsic_aff(camera_lidar_extrinsic);
    pcl::transformPointCloud(*cloud_in, *cloud_out, camera_lidar_extrinsic_aff);

    sensor_msgs::PointCloud2Ptr cloud_msg_out(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_out, *cloud_msg_out);
    cloud_msg_out->header.frame_id = "sensor";
    // cloud_msg_out->header.stamp = cloud_msg_in->header.stamp;
    cloud_msg_out->header.stamp = ros::Time::now();
    pub_cloud_transformed.publish(cloud_msg_out);
    // std::cout << "cloud_Callback: " << cloud_msg_out->header.stamp << std::endl;

}

void pixelPointCloud::sync_Callback(const sensor_msgs::Image::ConstPtr& image_msg_in, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_in){

    cv_bridge::CvImagePtr image_cv = cv_bridge::toCvCopy(image_msg_in, "bgr8");
    cv::Mat image_in = image_cv->image;
    cv::Mat image_out = cv::Mat(cv::Size(image_width, image_height), CV_16UC1, cv::Scalar(0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg_in, *cloud_in);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

#pragma omp for
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = cloud_in->points.begin(); pt < cloud_in->points.end(); pt++){

        int _u = static_cast<int>(pt->x * camera_instrinsics(0,0) / pt->z + camera_instrinsics(0,2));
        int _v = static_cast<int>(pt->y * camera_instrinsics(1,1) / pt->z + camera_instrinsics(1,2));
        // std::cout << "_u: " << _u << " " << "_v: " << _v << " " << "pt->z: " << pt->z << " " << std::endl;

        pcl::PointXYZRGB colored_3d_point;
        
        if ((_u >= 0) && (_u < image_width) && (_v >= 0) && (_v < image_height) && pt->z > 0) {
            colored_3d_point.x = pt->x;
            colored_3d_point.y = pt->y;
            colored_3d_point.z = pt->z;
            
            // std::cout << "x: " << colored_3d_point.x << std::endl;
            // std::cout << "y: " << colored_3d_point.y << std::endl;
            // std::cout << "z: " << colored_3d_point.z << std::endl;

            cv::Vec3b rgb_pixel = image_in.at<cv::Vec3b>(_v, _u);
            colored_3d_point.r = rgb_pixel[2];
            colored_3d_point.g = rgb_pixel[1];
            colored_3d_point.b = rgb_pixel[0];
            cloud_out->points.push_back(colored_3d_point);

            // std::cout << "r: " << colored_3d_point.r << std::endl;
            // std::cout << "g: " << colored_3d_point.g << std::endl;
            // std::cout << "b: " << colored_3d_point.b << std::endl;
            
            image_out.at<double>(_v, _u) = colored_3d_point.z;
            
        }
    }
    
    // std::cout << "sync_Callback end##" << std::endl;

    sensor_msgs::PointCloud2Ptr cloud_msg_out(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_out, *cloud_msg_out);
    cloud_msg_out->header.frame_id = "sensor";
    // cloud_msg_out->header.stamp = cloud_msg_in->header.stamp;
    // cloud_msg_out->header.stamp = ros::Time::now();
    pub_cloud_colored.publish(cloud_msg_out);

    sensor_msgs::ImagePtr image_msg_align_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_in).toImageMsg();
    image_msg_align_out->header.frame_id = "sensor";
    // image_msg_out->header.stamp = image_msg_in->header.stamp;
    // image_msg_out->header.stamp = ros::Time::now();
    pub_image_align_depth.publish(image_msg_align_out);

    sensor_msgs::ImagePtr image_msg_out = cv_bridge::CvImage(std_msgs::Header(), "mono16", image_out).toImageMsg();
    image_msg_out->header.frame_id = "sensor";
    // image_msg_out->header.stamp = image_msg_in->header.stamp;
    // image_msg_out->header.stamp = ros::Time::now();
    pub_image_depth.publish(image_msg_out);
}
