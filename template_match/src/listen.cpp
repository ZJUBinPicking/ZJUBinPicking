#include "template_match.hpp"

int main(int argc, char **argv) {
  //   ros::NodeHandle nh;
  //   ros::Subscriber bat_sub =
  //       nh.subscribe("/camera/depth/points", 10, cloudCB);  //接收点云
  //   ros::spin();
  ros::init(argc, argv, "pcl_write");

  template_match match;
  match.mainloop();
  return 0;
}