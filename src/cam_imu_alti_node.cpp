#include "acc_integrator.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "cam_imu_alti_node");
  ros::NodeHandle nh;
  accInt int_(nh);
  ros::spin();
  return 0;
}
