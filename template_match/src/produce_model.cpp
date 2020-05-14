#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>
#include <iostream>

#define pi 3.1415926
void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,
               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
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
int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.width = 2500;  // number of points from the cylinder + number of points
                       // from the plane = 693+2601 = 3294
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  size_t i = 0;

  // number of points from the cylinder = number of points fron a circle plane *
  // number of planes (number of points on the Z-axis) =
  // {[(6.2-0)/0.1]+1}*(11-0+1)=693
  for (int j = 0; j <= 10 * pi; j++)  // number of points fron a circle plane =
                                      // floor(20*3.1415926)-0+1 = 63
    for (int k = 0; k <= 30; k++)  // number of planes (number of points on the
                                   // Z-axis) = 10-0+1 = 11
    {
      cloud.points[i].x = cosf((float)j / 5) * 0.0075 + 0.1;
      cloud.points[i].y = sinf((float)j / 5) * 0.0075 + 0.1;
      cloud.points[i].z = (float)k * 5 / 3000 + 0.1;
      //   cloud.points[i].r = 255;
      //   cloud.points[i].g = 0;
      //   cloud.points[i].b = 0;

      i++;
    }

  // number of points from the plane = number of points on the X-axis * number
  // of points on the Y-axis = (50-0+1)*(50-0+1)=2601
  for (int l = 0; l <= 10 * pi;
       l++)  // number of points on the X-axis = 50-0+1 = 51
    for (int m = 0; m <= 10;
         m++)  // number of points on the Y-axis = 50-0+1 = 51
    {
      cloud.points[i].x = m * 0.00075 * cosf((float)l / 5) + 0.1;
      cloud.points[i].y = m * 0.00075 * sinf((float)l / 5) + 0.1;
      cloud.points[i].z = 0.1;

      //   cloud.points[i].r = 0;
      //   cloud.points[i].g = 255;
      //   cloud.points[i].b = 0;

      i++;
    }
  for (int l = 0; l <= 10 * pi;
       l++)  // number of points on the X-axis = 50-0+1 = 51
    for (int m = 0; m <= 10;
         m++)  // number of points on the Y-axis = 50-0+1 = 51
    {
      cloud.points[i].x = m * 0.00075 * cosf((float)l / 5) + 0.1;
      cloud.points[i].y = m * 0.00075 * sinf((float)l / 5) + 0.1;
      cloud.points[i].z = 0.15;

      //   cloud.points[i].r = 0;
      //   cloud.points[i].g = 255;
      //   cloud.points[i].b = 0;

      i++;
    }
  //   // draw a plane
  //   for (int l = 0; l <= 50; l++) {
  //     for (int m = 0; m <= 50; m++) {
  //       cloud.points[i].x = l;
  //       cloud.points[i].y = m;
  //       cloud.points[i].z = -10;
  //       i++;
  //     }
  //   }
  pcl::io::savePCDFileASCII(
      "/home/gjx/orbslam/catkin_ws/src/ZJUBinPicking/pcd_files/cylinder.pcd",
      cloud);  // save the data to a PCD file

  pcl::PointCloud<pcl::PointXYZ>* ptr = &cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(ptr);

  showCloud(mycloud, mycloud);

  return (0);
}