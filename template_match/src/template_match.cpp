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
  ros::param::get("~min_x", min_x);
  ros::param::get("~min_y", min_y);
  ros::param::get("~min_z", min_z);
  ros::param::get("~max_x", max_x);
  ros::param::get("~max_y", max_y);
  ros::param::get("~max_z", max_z);
  ros::param::get("~model_size", model_size);
  ros::param::get("~theta", theta);
  ros::param::get("~dx", dx);
  ros::param::get("~dy", dy);
  ros::param::get("~dz", dz);
  this->model_ = PointCloud::Ptr(new PointCloud);
  pcl::io::loadPCDFile("/home/gjx/orbslam/model_1.pcd", *model_);
  // ros::spinOnce();
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

  // if (!cloud_->empty()) cloud_->~PointCloud();
  cloud_ = PointCloud::Ptr(&cloud);
  // pcl::PointCloud<PointT>::Ptr mycloud(&cloud);
  pcl::console::TicToc tt;

  std::cout << "ReadImage...\n", tt.tic();
  //   pcl::PCDReader reader;
  //   reader.read(
  //       "/home/gjx/orbslam/pcl-master/doc/tutorials/content/sources/"
  //       "cylinder_segmentation/table_scene_mug_stereo_textured_cylinder.pcd",
  //       *mycloud);

  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> crop;

  crop.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));  //给定立体空间
  crop.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
  //数据随意给的，具体情况分析
  crop.setInputCloud(cloud_);
  crop.setKeepOrganized(true);
  crop.setUserFilterValue(0.1f);
  crop.filter(*cloud_filtered);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");  //设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits(0.5, 0.53);      //设置在过滤字段的范围
  pass.setFilterLimitsNegative(false);  //保留还是过滤掉范围内的点
  pass.filter(*cloud_filtered);
  std::cout << "The points data:  " << cloud_filtered->points.size()
            << std::endl;
  // model_->resize(model_size);
  pcl::io::savePCDFileASCII("/home/gjx/orbslam/filter.pcd",
                            *cloud_filtered);  //保存pcd
  // showCloud(model_, cloud_filtered);
  cloud_ = cloud_filtered;
  cluster();
}
void template_match::cluster() {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.02);  // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_);
  ec.extract(cluster_indices);
  // showCloud(cloud,cloud_filtered);
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud_->points[*pit]);  //*
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: "
              << cloud_cluster->points.size() << " data points." << std::endl;

    std::stringstream ss;

    ss << "cloud_cluster_" << j;
    goals.push_back(cloud_cluster);
    j++;
  }
  cout << goals.size() << endl;
  match();
}

void template_match::match() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(goals[1]);

  FeatureCloud template_cloud;
  template_cloud.setInputCloud(model_);
  object_templates.push_back(template_cloud);

  // const float depth_limit = 1.0;
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(cloud);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(0, depth_limit);
  // pass.filter(*cloud);

  // ... and downsampling the point cloud
  // const float voxel_grid_size = 0.005f;
  // pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  // vox_grid.setInputCloud(cloud);
  // vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
  // // vox_grid.filter (*cloud); // Please see this
  // pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(
  //     new pcl::PointCloud<pcl::PointXYZ>);
  // vox_grid.filter(*tempCloud);
  // cloud = tempCloud;

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud(cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size(); ++i) {
    template_align.addTemplateCloud(object_templates[i]);
  }
  template_align.setTargetCloud(target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment(best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation =
      best_alignment.final_transformation.block<3, 3>(0, 0);
  Eigen::Vector3f translation =
      best_alignment.final_transformation.block<3, 1>(0, 3);
  // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // transform_2.translation(translation);
  // // The same rotation matrix as before; theta radians arround Z axis
  // transform_2.rotation = rotation;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
  //     new pcl::PointCloud<pcl::PointXYZ>());
  // // You can either apply transform_1 or transform_2; they are the same
  // pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);
  // transformed_cloud->resize(model_size);
  printf("\n");
  printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1),
         rotation(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1),
         rotation(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1),
         rotation(2, 2));
  printf("\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1),
         translation(2));
  showCloud(cloud, model_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr template_match::transclouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, double theta, double dx,
    double dy, double dz) {
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see
  // https://en.wikipedia.org/wiki/Rotation_matrix)
  // float theta = M_PI / 2;  // The angle of rotation in radians
  transform_1(0, 0) = cos(theta);
  transform_1(0, 1) = -sin(theta);
  transform_1(1, 0) = sin(theta);
  transform_1(1, 1) = cos(theta);
  //    (row, column)

  // Define a translation of 2.5 meters on the x axis.
  transform_1(0, 3) = dx;
  transform_1(1, 3) = dy;
  transform_1(2, 3) = dz;

  // Print the transformation
  printf("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;
  /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << dx, dy, dz;
  // The same rotation matrix as before; theta radians arround Z axis
  transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
  // Print the transformation
  printf("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;
  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);
  transformed_cloud->resize(model_size);
  showCloud(source_cloud, transformed_cloud);
}

void template_match::mainloop() {
  // ros::Rate loop_rate_class(10);
  while (1) {
    // cout << "666" << endl;
    ros::spinOnce();
    // transclouds(this->model_, this->theta, this->dx, this->dy, this->dz);
    // loop_rate_class.sleep();
  }
}