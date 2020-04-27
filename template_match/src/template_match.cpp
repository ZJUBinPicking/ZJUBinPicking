#include "template_match.hpp"
int user_data;
void viewerOneOff(pcl::visualization::PCLVisualizer &viewer) {
  //设置背景颜色
  viewer.setBackgroundColor(1.0, 0.5, 1.0);
  // //球体坐标
  // pcl::PointXYZ o;
  // o.x = 0;
  // o.y = 0;
  // o.z = 0;
  // //添加球体
  // viewer.addSphere(o, 1, "sphere", 0);
  std::cout << "i only run once" << std::endl;
}

void viewerPsycho(pcl::visualization::PCLVisualizer &viewer) {
  static unsigned count = 0;
  std::stringstream ss;
  ss << "Once per viewer loop: " << count++;
  viewer.removeShape("text", 0);
  viewer.addText(ss.str(), 200, 300, "text", 0);

  // FIXME: possible race condition here:
  user_data++;
}

template_match::template_match() { init(); }

void template_match::init() {
  ros::NodeHandle nh;
  bat_sub = nh.subscribe("/camera/depth/points", 1, &template_match::cloudCB,
                         this);  //接收点云
  ros::spinOnce();
}

void template_match::showCloud(pcl::PointCloud<PointT>::Ptr cloud1,
                               pcl::PointCloud<PointT>::Ptr cloud2) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addPointCloud<pcl::PointXYZ>(cloud1, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud2", v2);
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  viewer->addCoordinateSystem(1.0);

  viewer->initCameraParameters();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  };
}

void template_match::cloudCB(const sensor_msgs::PointCloud2 &input) {
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

  std::cout << "ReadImage...\n", tt.tic();
  pcl::PCDReader reader;
  reader.read("/home/gjx/orbslam/test.pcd", *mycloud);
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> crop;

  crop.setMin(Eigen::Vector4f(0, 0, 0.0, 1.0));  //给定立体空间
  crop.setMax(
      Eigen::Vector4f(5.0, 5.0, 5.0, 1.0));  //数据随意给的，具体情况分析
  crop.setInputCloud(mycloud);
  crop.setKeepOrganized(true);
  crop.setUserFilterValue(0.1f);
  crop.filter(*cloud_filtered);
  std::cout << "The points data:  " << cloud_filtered->points.size()
            << std::endl;
  // showCloud(mycloud, cloud_filtered);
}
void template_match::mainloop() {
  ros::Rate loop_rate_class(10);
  while (1) {
    // cout << "666" << endl;
    ros::spinOnce();
    loop_rate_class.sleep();
  }
}