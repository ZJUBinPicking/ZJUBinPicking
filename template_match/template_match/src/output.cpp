#include <pcl/io/pcd_io.h>  //which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

main(int argc, char **argv) {
  ros::init(argc, argv, "output_clouds");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/points", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile(
      "/home/gjx/orbslam/catkin_ws/src/ZJUBinPicking/pcd_files/test3.pcd",
      cloud);
  // Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  output.header.frame_id =
      "odom";  // this has been done in order to be able to visualize our
               // PointCloud2 message on the RViz visualizer
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}