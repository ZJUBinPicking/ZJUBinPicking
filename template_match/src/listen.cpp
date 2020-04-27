
#include <pcl/console/time.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

typedef pcl::PointXYZ PointT;
using namespace std;

void cloudCB(const sensor_msgs::PointCloud2 &input) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(input, cloud);  //从ROS类型消息转为PCL类型消息
  //   pcl::io::savePCDFileASCII("/home/gjx/orbslam/test.pcd",
  //                             cloud);  //保存pcd
  //   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mycloud(
  //       new pcl::PointCloud<pcl::PointXYZRGBA>);
  //   // 加载pcd文件到cloud
  //   pcl::io::loadPCDFile("/home/gjx/orbslam/write_pcd_test.pcd", *mycloud);
  //   pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //   //这里会一直阻塞直到点云被渲染
  //   viewer.showCloud(mycloud);

  //   // 循环判断是否退出
  //   while (!viewer.wasStopped()) {
  //     // 你可以在这里对点云做很多处理
  //   }
  pcl::PointCloud<PointT>::Ptr mycloud(new pcl::PointCloud<PointT>);
  pcl::console::TicToc tt;

  std::cerr << "ReadImage...\n", tt.tic();
  pcl::PCDReader reader;
  reader.read("/home/gjx/orbslam/write_pcd_test.pcd", *mycloud);
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> crop;

  crop.setMin(Eigen::Vector4f(-0.2, -0.2, 0.0, 1.0));  //给定立体空间
  crop.setMax(
      Eigen::Vector4f(-2.0, 1.0, 2.0, 1.0));  //数据随意给的，具体情况分析
  crop.setInputCloud(mycloud);
  crop.setKeepOrganized(true);
  crop.setUserFilterValue(0.1f);
  crop.filter(*cloud_filtered);
  std::cerr << "The points data:  " << cloud_filtered->points.size()
            << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addPointCloud<pcl::PointXYZ>(mycloud, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "sample cloud2", v2);
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  viewer->addCoordinateSystem(1.0);

  viewer->initCameraParameters();
  while (!viewer->wasStopped()) {
    // viewer->spinOnce(100);
    // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  };
}
main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_write");
  ros::NodeHandle nh;
  ros::Subscriber bat_sub =
      nh.subscribe("/camera/depth/points", 10, cloudCB);  //接收点云
  ros::spin();
  return 0;
}