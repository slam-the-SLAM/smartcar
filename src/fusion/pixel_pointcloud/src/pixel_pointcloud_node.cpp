#include "pixel_pointcloud.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pixel_pointcloud_node");
    pixelPointCloud pixel_pointcloud_node;
    ros::MultiThreadedSpinner spinner(4);
    ros::spin();
    return 0;
}
