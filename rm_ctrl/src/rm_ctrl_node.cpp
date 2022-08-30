#include "rm_ctrl/rm_ctrl.hpp"

using namespace std;
using namespace Eigen;


int main(int argc, char **argv) {
    ros::init(argc, argv, "cloud_publisher");
    ros::NodeHandle nh("~");
	rm_ctrl::RmCtrl ctr_node(nh);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
