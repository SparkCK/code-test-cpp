#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

int main() {
  pcl::visualization::PCLVisualizer viewer("Multi-Viewport Viewer");

  // 创建四个视口
  int vp_1, vp_2, vp_3, vp_4;
  viewer.createViewPort(0.0, 0.0, 0.5, 0.5, vp_1);
  viewer.createViewPort(0.5, 0.0, 1.0, 0.5, vp_2);
  viewer.createViewPort(0.0, 0.5, 0.5, 1.0, vp_3);
  viewer.createViewPort(0.5, 0.5, 1.0, 1.0, vp_4);

  // 在每个视口设置不同的相机视角
  viewer.setCameraPosition(0, 0, 10, 0, 0, 0, 0, 1, 0);  // 俯视图

  viewer.setBackgroundColor(0.0, 0.0, 0.0, vp_1);
  viewer.setBackgroundColor(0.25, 0.25, 0.25, vp_2);
  viewer.setBackgroundColor(0.5, 0.5, 0.5, vp_3);
  viewer.setBackgroundColor(0.75, 0.75, 0.75, vp_4);

  // 添加一个点云到所有视口
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(1, 1, 1));
  cloud->push_back(pcl::PointXYZ(-1, -1, -1));

  viewer.addPointCloud(cloud, "cloud", vp_1);
  viewer.addPointCloud(cloud, "cloud", vp_2);
  viewer.addPointCloud(cloud, "cloud", vp_3);
  viewer.addPointCloud(cloud, "cloud", vp_4);

  viewer.addCoordinateSystem(2.0, "origin", vp_1);
  viewer.addCoordinateSystem(2.0, "origin", vp_2);
  viewer.addCoordinateSystem(2.0, "origin", vp_3);
  viewer.addCoordinateSystem(2.0, "origin", vp_4);

  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud", vp_1);
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud", vp_2);
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud", vp_3);
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud", vp_4);

  // 启动可视化
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }

  return 0;
}
