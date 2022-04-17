#include "normal_filter.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "normal_filter_node");
    NormalFilter normal_filter_node;
    ros::MultiThreadedSpinner spinner(16);
    spinner.spin();
    return 0;
}