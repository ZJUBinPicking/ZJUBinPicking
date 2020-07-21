#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ctime>
#include <iostream>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace pcl;
using namespace std;
using namespace cv;
struct cluster {
  int index;
  int dense_index;
  double dense;
  int height_index;
  double height;
  int pose_index;
  double pose;
  double score;
};
class DBSCAN {
 public:
  DBSCAN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double eps, double MinPts)
      : eps(eps), MinPts(MinPts) {
    cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ = cloud;
    origin_cloud_ =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    origin_cloud_ = cloud;
    cluster_type = vector<int>(cloud->points.size());
    neighbourPoints.resize(cloud->points.size());
    neighbourDistance.resize(cloud->points.size());
  }
  double MinPts;
  double eps;
  double MinbPts = 25;
  clock_t start, end;
  ~DBSCAN() {}
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud_;
  vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_center;
  vector<int> cluster_type;
  vector<int> core_points;
  vector<int> bound_points;
  vector<int> result_points;
  bool view_on = 1;
  vector<Eigen::Vector4f> cluster_centroid;
  int method_ = 0;
  int use_edge = 1;
  int sum_points = 0;
  vector<int> points_num;
  const int CORE_POINT = 0;
  const int BOUND_POINT = 1;
  const int NOISE_POINT = 2;
  const int KD_TREE = 0;
  const int OCT_TREE = 1;
  vector<cluster> cluster_score;
  std::vector<std::vector<int>> neighbourPoints;
  std::vector<std::vector<float>> neighbourDistance;
  void start_scan();
  void select_kernel();
  void find_independent();
  vector<int> vectors_set_diff(vector<int> v1, vector<int> v2);
  vector<int> vectors_intersection(vector<int> v1, vector<int> v2);
  vector<int> vectors_set_union(vector<int> v1, vector<int> v2);
  void normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void curve(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  void if_continue(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
};