#include "lidar_mapping.h"


int main(int argc, char **argv)
{
    // std::cout << "----------omp: <" << omp_get_num_procs()  << "> ---------" << std::endl;
    ros::init(argc, argv, "lidar_mapping_node");
    LidarMapping lidar_mapping_node;
    // ros::MultiThreadedSpinner spinner(10);
    ros::spin();
    return 0;
}
