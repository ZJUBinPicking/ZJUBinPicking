#include "DBSCAN/DBSCAN.hpp"

void viewerOneOff(pcl::visualization::PCLVisualizer &viewer, double x, double y,
                  double z, string name) {
  pcl::PointXYZ o;
  o.x = x;
  o.y = y;
  o.z = z;
  viewer.addSphere(o, 0.001, 255, 0, 0, name, 0);
}

void showCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,
               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud2", v2);
  viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
  // viewer->addCoordinateSystem(1.0);

  viewer->initCameraParameters();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  };
}

void DBSCAN::normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  // create an empty kdtree representation, and pass it to the normal estimation
  // object its content will be filled inside the object, based on the given
  // input dataset(as no other search surface is given)
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);

  // use all neighbours in a sphere of radius 3cm
  // ne.setRadiusSearch(0.03);
  ne.setKSearch(5);
  // compute the features
  // cloud_normals->points.size() should have the same size as the input
  // cloud->points.size
  ne.compute(*cloud_normals);
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal,
                                     pcl::PrincipalCurvatures>
      pc;
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(
      new pcl::PointCloud<pcl::PrincipalCurvatures>);
  pc.setInputCloud(cloud);
  pc.setInputNormals(cloud_normals);
  pc.setSearchMethod(tree);
  // pc.setRadiusSearch(0.05);
  pc.setKSearch(5);
  // normal visualization
  pcl::visualization::PCLVisualizer viewer("PCL viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);
  // viewer.addPointCloudPrincipalCurvatures(
  //     cloud, cloud_normals, cloud_curvatures, 10, 1.0, "cloud_curvatures");

  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
}

// void DBSCAN::curve(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//   ne.setInputCloud(cloud);
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
//       new pcl::search::KdTree<pcl::PointXYZ>());
//   ne.setSearchMethod(tree);  //设置搜索方法
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
//       new pcl::PointCloud<pcl::Normal>);
//   // ne.setRadiusSearch(0.05); //设置半径邻域搜索
//   ne.setKSearch(5);
//   ne.compute(*cloud_normals);  //计算法向量
//   //计算曲率-------------------------------------------------------------------------------------
//   pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal,
//                                      pcl::PrincipalCurvatures>
//       pc;
//   pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(
//       new pcl::PointCloud<pcl::PrincipalCurvatures>);
//   pc.setInputCloud(cloud);
//   pc.setInputNormals(cloud_normals);
//   pc.ef(tree);
//   // pc.setRadiusSearch(0.05);
//   pc.setKSearch(5);
//   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//   string a = "cloud_normals";
//   string b = "cloud_normals a";

//   viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud,
//   cloud_normals); viewer.addPointCloudPrincipalCurvatures<pcl::PointXYZ,
//                                           pcl::PrincipalCurvatures>(
//       cloud, cloud_curvatures);

//   while (!viewer.wasStopped()) {
//     viewer.spinOnce();
//   }
// }
// https://blog.csdn.net/GoodLi199309/article/details/80537310
// https://blog.csdn.net/lming_08/article/details/18360329
void DBSCAN::if_continue(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal,

                                     pcl::PrincipalCurvatures>
      pc;
  pc.setInputCloud(cloud1);

  // Pass the original data (before downsampling) as the search surface
  pc.setSearchSurface(cloud2);

  ne.setInputCloud(cloud1);

  // Pass the original data (before downsampling) as the search surface
  ne.setSearchSurface(cloud2);

  // Create an empty kdtree representation, and pass it to the normal estimation
  // object. Its content will be filled inside the object, based on the given
  // surface dataset.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  pc.setSearchMethod(tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(
      new pcl::PointCloud<pcl::PrincipalCurvatures>);
  // Use all neighbors in a sphere of radius 3cm
  // ne.setRadiusSearch(0.01);
  ne.setKSearch(5);
  pc.setKSearch(5);

  // Compute the features
  ne.compute(*cloud_normals);
  pc.setInputNormals(cloud_normals);
  pc.compute(*cloud_curvatures);
  cout << "normals" << cloud_normals->size() << endl;
  cout << "cloud_curvatures" << cloud_curvatures->size() << endl;
}
// https://blog.csdn.net/simonyucsdy/article/details/102696516
void DBSCAN::start_scan() {
  start = clock();
  select_kernel();
  find_independent();
}
void DBSCAN::select_kernel() {
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  float resolution = 0.0001f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
  if (method_ == KD_TREE) {
    kdtree.setInputCloud(cloud_);
  } else if (method_ == OCT_TREE) {
    octree.setInputCloud(cloud_);
    octree.addPointsFromInputCloud();
  }

  for (auto i = 0; i < cloud_->points.size(); i++) {
    if (method_ == KD_TREE) {
      kdtree.radiusSearch(cloud_->points[i], eps, neighbourPoints[i],
                          neighbourDistance[i]);
    } else if ((method_ == OCT_TREE)) {
      octree.radiusSearch(cloud_->points[i], eps, neighbourPoints[i],
                          neighbourDistance[i]);
    }
    if (neighbourPoints[i].size() >= MinPts) {
      cluster_type.push_back(CORE_POINT);
      core_points.push_back(i);
      //   cout << "core cluster" << neighbourPoints[i].size() << endl;
    } else if (neighbourPoints[i].size() > MinbPts) {
      cluster_type.push_back(BOUND_POINT);
      bound_points.push_back(i);
    } else {
      cluster_type.push_back(NOISE_POINT);
      // auto iter_1 = cloud_->begin() + i;
      // auto iter_2 = neighbourPoints.begin() + i;
      // auto iter_3 = neighbourDistance.begin() + i;

      // cloud_->erase(iter_1);
      // neighbourPoints.erase(iter_2);
      // neighbourDistance.erase(iter_3);
    }
  }
  cout << "core_points" << core_points.size() << endl;
  cout << "bound_points" << bound_points.size() << endl;
}
vector<int> DBSCAN::vectors_intersection(vector<int> v1, vector<int> v2) {
  vector<int> v;
  sort(v1.begin(), v1.end());
  sort(v2.begin(), v2.end());
  set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(),
                   back_inserter(v));  //求交集
  return v;
}

//两个vector求并集
vector<int> DBSCAN::vectors_set_union(vector<int> v1, vector<int> v2) {
  vector<int> v;
  sort(v1.begin(), v1.end());
  sort(v2.begin(), v2.end());
  set_union(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
  return v;
}
vector<int> DBSCAN::vectors_set_diff(vector<int> v1, vector<int> v2) {
  vector<int> v;
  sort(v1.begin(), v1.end());
  sort(v2.begin(), v2.end());
  set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
  return v;
}
void DBSCAN::find_independent() {
  cout << "start find intersection of cluster and union them" << endl;
  for (int i = 0; i < core_points.size(); i++) {
    if (neighbourPoints[core_points[i]].size() == 0) continue;
    for (int j = 0; j < core_points.size(); j++) {
      if (j == i || neighbourPoints[core_points[j]].size() == 0) continue;
      vector<int> result;
      result = vectors_intersection(neighbourPoints[core_points[i]],
                                    neighbourPoints[core_points[j]]);
      if (result.size() > 20) {
        if (neighbourPoints[core_points[i]].size() < 500 &&
            neighbourPoints[core_points[j]].size() < 500 &&
            (neighbourPoints[core_points[i]].size() +
             neighbourPoints[core_points[j]].size()) < 600) {
          neighbourPoints[core_points[i]] = vectors_set_union(
              neighbourPoints[core_points[i]], neighbourPoints[core_points[j]]);
          neighbourPoints[core_points[j]].clear();
        }
        // else {
        //   neighbourPoints[core_points[j]] = vectors_set_diff(
        //       neighbourPoints[core_points[i]],
        //       neighbourPoints[core_points[j]]);
        //   // neighbourPoints[core_points[j]].clear();
        // }
      }
    }
  }
  for (int i = 0; i < bound_points.size() && use_edge; i++) {
    int max_intersect = 0;
    int max_index = -1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr bound(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (auto j = 0; j < neighbourPoints[bound_points[i]].size(); j++) {
      pcl::PointXYZ point;

      point.x = cloud_->points[neighbourPoints[bound_points[i]][j]].x;
      point.y = cloud_->points[neighbourPoints[bound_points[i]][j]].y;
      point.z = cloud_->points[neighbourPoints[bound_points[i]][j]].z;
      bound->points.push_back(point);
    }
    // normal(bound);
    for (int j = 0; j < core_points.size(); j++) {
      if (neighbourPoints[core_points[j]].size() == 0) continue;

      vector<int> result;
      result = vectors_intersection(neighbourPoints[bound_points[i]],
                                    neighbourPoints[core_points[j]]);
      if (result.size() > max_intersect) {
        max_intersect = result.size();
        max_index = core_points[j];
      }
    }
    if (max_index != -1) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr core(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (auto k = 0; k < neighbourPoints[max_index].size(); k++) {
        pcl::PointXYZ point;

        point.x = cloud_->points[neighbourPoints[max_index][k]].x;
        point.y = cloud_->points[neighbourPoints[max_index][k]].y;
        point.z = cloud_->points[neighbourPoints[max_index][k]].z;
        core->points.push_back(point);
      }
      // if_continue(bound, core);
      neighbourPoints[max_index] = vectors_set_union(
          neighbourPoints[max_index], neighbourPoints[bound_points[i]]);
    }
  }
  for (auto i = 0; i < core_points.size(); i++) {
    if (neighbourPoints[core_points[i]].size() == 0 ||
        neighbourPoints[core_points[i]].size() < 200)
      continue;

    cout << "find_cluster" << neighbourPoints[core_points[i]].size() << endl;
    for (int j = 0; j < core_points.size(); j++) {
      if (j == i || neighbourPoints[core_points[j]].size() == 0) continue;
      vector<int> result;
      result = vectors_intersection(neighbourPoints[core_points[i]],
                                    neighbourPoints[core_points[j]]);
      if (result.size() > 200) {
        if (neighbourPoints[core_points[i]].size() >
            neighbourPoints[core_points[j]].size()) {
          neighbourPoints[core_points[i]] = vectors_set_diff(
              neighbourPoints[core_points[i]], neighbourPoints[core_points[j]]);
        }
      }
    }
  }
  vector<int> final_cluster;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  this->cluster_center =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto i = 0; i < core_points.size(); i++) {
    if (neighbourPoints[core_points[i]].size() == 0 ||
        neighbourPoints[core_points[i]].size() < 200)
      continue;
    cout << "find_cluster" << neighbourPoints[core_points[i]].size() << endl;
    auto iter_1 = cloud_->begin() + i;
    auto iter_2 = neighbourPoints.begin() + i;
    auto iter_3 = neighbourDistance.begin() + i;
    int R = rand() % 255;
    int G = rand() % 255;
    int B = rand() % 255;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);

    for (auto j = 0; j < neighbourPoints[core_points[i]].size(); j++) {
      pcl::PointXYZRGB point;
      point.r = R;
      point.g = G;
      point.b = B;
      point.x = cloud_->points[neighbourPoints[core_points[i]][j]].x;
      point.y = cloud_->points[neighbourPoints[core_points[i]][j]].y;
      point.z = cloud_->points[neighbourPoints[core_points[i]][j]].z;
      result_cloud->points.push_back(point);
      cloud_cluster->points.push_back(
          cloud_->points[neighbourPoints[core_points[i]][j]]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    sum_points += cloud_cluster->points.size();
    points_num.push_back(cloud_cluster->points.size());
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    // std::cout << "The XYZ coordinates of the centroid are: (" <<
    // centroid[0]
    //           << ", " << centroid[1] << ", " << centroid[2] << ")."
    //           << std::endl;
    cluster_center->points.push_back(
        pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
    cluster_centroid.push_back(centroid);
    this->result_cloud_.push_back(cloud_cluster);
  }
  cluster_center->width = cluster_center->points.size();
  cluster_center->height = 1;
  cluster_center->is_dense = true;
  cout << "the sum of clusters: " << this->result_cloud_.size() << endl;
  result_cloud->width = result_cloud->points.size();
  result_cloud->height = 1;
  result_cloud->is_dense = true;

  end = clock();
  double endtime = (double)(end - start) / CLOCKS_PER_SEC;
  cout << "Total time:" << end << "  " << start << "  " << endtime << "s"
       << endl;  // s为单位
  vector<pcl::PointXYZ> cluster_vectors;
  vector<pcl::PointXYZ> centroidXYZ_value;
  if (cluster_center->points.size() > 1) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cluster_center);

    for (int i = 0; i < cluster_center->points.size(); i++) {
      cluster temp;
      temp.index = i;
      cluster_score.push_back(temp);
      vector<int> indices;
      vector<float> dists;
      kdtree.nearestKSearch(cluster_center->points[i],
                            int(cluster_center->points.size() / 2) + 1, indices,
                            dists);
      double sum = 0;
      for (auto dist : dists) {
        sum = sum + dist;
      }
      cluster_score[i].dense = sum;
      // pca(result_cloud_[i]);
      // Eigen::Vector4f centroid;
      // Eigen::Matrix3f covariance_matrix;

      // // Extract the eigenvalues and eigenvectors
      // Eigen::Vector3f eigen_values;
      // Eigen::Matrix3f eigen_vectors;

      // // pcl::compute3DCentroid(*result_cloud_[i], centroid);

      // // Compute the 3x3 covariance matrix
      // pcl::computeCovarianceMatrix(*result_cloud_[i], cluster_centroid[i],
      //                              covariance_matrix);
      // pcl::eigen33(covariance_matrix, eigen_vectors, eigen_values);
      // PointXYZ centroidXYZ;
      // centroidXYZ.getVector4fMap() = cluster_centroid[i];
      // centroidXYZ_value.push_back(centroidXYZ);
      // PointXYZ Point1 =
      //     PointXYZ((cluster_centroid[i](0) + eigen_vectors.col(0)(0)),
      //              (cluster_centroid[i](1) + eigen_vectors.col(0)(1)),
      //              (cluster_centroid[i](2) + eigen_vectors.col(0)(2)));
      // cluster_vectors.push_back(Point1);

      // _viewer->addLine<pcl::PointXYZRGB> (centroid, eigen_vectors, "line");
    }
    // sort(cluster_score.begin(), cluster_score.end(), cmp);
  } else {
    cluster temp;
    temp.index = 0;
    cluster_score.push_back(temp);
  }
  // for (int i = 0; i < cluster_score.size(); i++) {
  //   cluster_score[i].dense_index = i + 1;
  //   // cout << cluster_score[i].dense_index << "  " <<
  //   cluster_score[i].dense
  //   //      << " " << cluster_score[i].index << endl;
  // }
  if (view_on) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addPointCloud<pcl::PointXYZRGB>(result_cloud, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->addPointCloud<pcl::PointXYZ>(origin_cloud_, "sample cloud2", v2);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    // viewer->addCoordinateSystem(1.0);

    viewer->initCameraParameters();
    for (int i = 0; i < cluster_centroid.size(); i++) {
      std::stringstream ss;
      ss << "cloud_cluster_" << i;
      viewerOneOff(*viewer, cluster_centroid[i][0], cluster_centroid[i][1],
                   cluster_centroid[i][2], ss.str());
      // showCloud(result_cloud, result_cloud_[i]);
      // std::stringstream ss2;
      // ss2 << "arrow" << i;
      // viewer->addArrow(cluster_vectors.at(i), centroidXYZ_value.at(i), 0.5,
      // 0.5,
      //                  0.5, false, ss2.str());
    }
    while (!viewer->wasStopped()) {
      viewer->spinOnce(100);
      // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    };
  }
}
// int main() {
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new
//   pcl::PointCloud<pcl::PointXYZ>);

//   pcl::io::loadPCDFile(
//       "/home/gjx/orbslam/catkin_ws/src/ZJUBinPicking/pcd_files/filter_1.pcd",
//       *cloud);
//   double dense = 0.01;
//   double result_dense;
//   double min = 10000000;
//   // cout << dense << endl;
//   // for (int i = 0; i < 5; i++) {
//   //   DBSCAN gather(cloud, dense, 40);  // 0.011
//   //   gather.view_on = 0;
//   //   gather.start_scan();

//   //   double avr = gather.sum_points / gather.points_num.size();
//   //   double var = 0;
//   //   for (int i = 0; i < gather.points_num.size(); i++) {
//   //     var += (gather.points_num[i] - avr) * (gather.points_num[i] - avr);
//   //   }
//   //   var = var / gather.points_num.size();
//   //   cout << "var" << var << endl;

//   //   if (var < min) {
//   //     min = var;
//   //     result_dense = dense;
//   //   }
//   //   dense -= 0.0001;
//   // }
//   cout << "result dense" << result_dense << endl;
//   DBSCAN gather(cloud, 0.01, 45);  // 0.01 35
//   gather.start_scan();
// }