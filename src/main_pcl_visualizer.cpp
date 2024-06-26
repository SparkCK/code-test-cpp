#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/common/shapes.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <eigen3/Eigen/Dense>

inline std::string formatNum(double num) noexcept(false) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << num;
  return oss.str();
}

inline std::string formatNum(float num) noexcept(false) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << num;
  return oss.str();
}

int main() {
  auto r = 1.0, g = 0.0, b = 0.0;
  auto x = 1.0, y = 0.0, z = 0.0;
  auto l = 4.0, w = 4.0, h = 2.0;

  // 创建一个点云对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 填充点云数据（示例数据）
  cloud->points.resize(3);
  cloud->points[0].x = 1.0;
  cloud->points[0].y = 0.0;
  cloud->points[0].z = 0.0;

  cloud->points[1].x = 0.0;
  cloud->points[1].y = 2.0;
  cloud->points[1].z = 0.0;

  cloud->points[2].x = 0.0;
  cloud->points[2].y = 0.0;
  cloud->points[2].z = 3.0;

  // 创建可视化对象
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");

  // 设置窗口大小
  viewer.setPosition(100, 100);  // 设置窗口左上角的屏幕位置
  viewer.setSize(1800, 900);     // 设置窗口的宽度和高度

  // 创建两个视口唯一id（一个窗口中可以有多个视口）
  int v1(0);
  int v2(1);
  int v3(2);
  int v4(3);

  // 创建视口 xmin(0-1, 0->左边框, 1->右边框), ymin(0-1, 0->底边框, 1->上边框),
  // xmax, ymax, viewport_id
  viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v2);
  viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v3);
  viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v4);

  /// 设置背景颜色（如果没有指定port，那么背景颜色会应用到所有视口，除了那些指定了其他背景色的视口）
  // viewer.setBackgroundColor(1.0, 0.5, 0.5);

  // 设置视口相机参数 //全局参数
  viewer.setCameraPosition(0, 0, 10, 0, 0, 0, 0, 1, 0, v1);

  // 在第一个视口中添加坐标系 scale(坐标轴的长度), name(坐标轴的名称),
  // viewport_id
  viewer.addCoordinateSystem(1.0, "origin", v1);

  // 在第一个视口中添加点云
  viewer.addPointCloud(cloud, "cloud1", v1);
  viewer.setBackgroundColor(0.0, 0.0, 0.0, v1);
  // 设置点云的点大小
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud1", v1);

  viewer.addText("Viewport 1", 10, 10, "v1 text", v1);

  // 第二个视口
  //  viewer.setCameraPosition(0, 0, 0, 0, 0, 1, v2);

  viewer.addCoordinateSystem(1.0, "origin", v2);
  viewer.addCoordinateSystem(2.0, 1.0, 0.0, 0.0, "x_offset", v2);

  viewer.addPointCloud(cloud, "cloud2", v2);
  viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud2", v2);
  // 设置视口标题
  viewer.addText("Viewport 2", 10, 10, "v2 text", v2);

  // 第三个视口
  viewer.setBackgroundColor(0.3, 0.3, 0.3, v3);
  viewer.addCoordinateSystem(1.0, "origin", v3);
  viewer.addText("Viewport 3", 10, 10, "v3 text", v3);

  pcl::ModelCoefficients circle_coef;
  circle_coef.values.resize(3);
  circle_coef.values[0] = 0.0;
  circle_coef.values[1] = 0.0;
  circle_coef.values[2] = 2.0;
  auto circle = pcl::visualization::create2DCircle(circle_coef);
  viewer.addCircle(circle_coef, "circle", v3);
  // 设置圆的线条粗细
  viewer.setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "circle", v3);

  std::string cube_id = "cube1";
  // 设置长方体的参数
  r = 1.0, g = 0.0, b = 0.0;
  x = 0.0, y = 0.0, z = -1.0;
  l = 4.0, w = 2.0, h = 2.0;
  Eigen::Vector3f translation0(x, y, z);  // 平移向量
  double yaw_1 = DEG2RAD(90.0);           // 绕z轴旋转角度
  Eigen::Quaternionf rotation0{
      Eigen::AngleAxisf(yaw_1, Eigen::Vector3f::UnitZ())};  // 四元数（无旋转）
  viewer.addCube(translation0, rotation0, l, w, h, cube_id, v3);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                     r, g, b, cube_id, v3);  // color
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                     0.5, cube_id, v3);  // 80% opacity

  std::string arrow_id = "arrow1";
  double arrow_length = l * 0.5 + 1.0;  // 箭头的长度
  pcl::PointXYZ arrow_start{x, y, z};   // 箭头的起点在立方体的中心
  pcl::PointXYZ arrow_delta{arrow_length * cos(yaw_1),
                            arrow_length * sin(yaw_1), 0.0};
  pcl::PointXYZ arrow_end{arrow_start.x + arrow_delta.x,
                          arrow_start.y + arrow_delta.y,
                          arrow_start.z + arrow_delta.z};
  viewer.addArrow(arrow_end, arrow_start, r, g, b, false, arrow_id, v3);
  // 设置箭头的线宽
  viewer.setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, arrow_id,
      v3);  // 设置线宽为5.0

  // 在长方体的z轴上方显示文字
  std::string text_id = "id";
  std::string text = "l=" + formatNum(l);
  pcl::PointXYZ text_position{x, y,
                              z + h / 2.0 + 0.1};  // 文字位置在长方体的z轴上方
  r = 1.0, g = 1.0, b = 1.0;
  viewer.addText3D(text, text_position, 0.2, r, g, b, text_id, v3);

  {
    cube_id = "cube_1_1";
    // 设置长方体的参数
    r = 0.0, g = 1.0, b = 0.0;
    x = 0.0, y = 5.0, z = -0.0;
    l = 4.0, w = 2.0, h = 2.0;
    Eigen::Vector3f translation_1_1(x, y, z);  // 平移向量
    double yaw_1_1 = DEG2RAD(-45.0);           // 绕z轴旋转角度
    Eigen::Quaternionf rotation_1_1{Eigen::AngleAxisf(
        yaw_1_1, Eigen::Vector3f::UnitZ())};  // 四元数（无旋转）
    viewer.addCube(translation_1_1, rotation_1_1, l, w, h, cube_id, v3);
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                       r, g, b, cube_id, v3);  // color
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, cube_id,
        v3);  // 80% opacity

    arrow_id = "arrow_1_1";
    arrow_length = l * 0.5 + 1.0;          // 箭头的长度
    arrow_start = pcl::PointXYZ{x, y, z};  // 箭头的起点在立方体的中心
    arrow_delta = pcl::PointXYZ{arrow_length * cos(yaw_1_1),
                                arrow_length * sin(yaw_1_1), 0.0};
    arrow_end = pcl::PointXYZ{arrow_start.x + arrow_delta.x,
                              arrow_start.y + arrow_delta.y,
                              arrow_start.z + arrow_delta.z};
    viewer.addArrow(arrow_end, arrow_start, r, g, b, false, arrow_id, v3);
    // 设置箭头的线宽
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5.0, arrow_id,
        v3);  // 设置线宽为5.0

    // 在长方体的z轴上方显示文字
    text_id = "id_1_1";
    text = "l=" + formatNum(l);
    text_position =
        pcl::PointXYZ{x, y, z + h / 2.0 + 0.1};  // 文字位置在长方体的z轴上方
    r = 1.0, g = 1.0, b = 1.0;
    viewer.addText3D(text, text_position, 0.2, r, g, b, text_id, v3);
  }
  ////////////////////////////
  // 第四个视口
  viewer.setBackgroundColor(0.2, 0.6, 0.7, v4);
  viewer.addCoordinateSystem(5.0, "origin", v4);
  viewer.addText("Viewport 4", 10, 10, "v4 text", v4);

  // pcl::ModelCoefficients cube_coef;
  // cube_coef.values.resize(10);
  // cube_coef.values[0] = -2.0; // x
  // cube_coef.values[1] = 2.0;  // y
  // cube_coef.values[2] = -1.0; // z
  // cube_coef.values[3] = 0.0;  // rotation around x
  // cube_coef.values[4] = 0.0;  // rotation around y
  // cube_coef.values[5] = 0.5;  // DEG2RAD(45.0);  // rotation around z
  // cube_coef.values[6] = 0.0;  // width
  // cube_coef.values[7] = 4.0;  // height
  // cube_coef.values[8] = 2.0;  // depth

  // viewer.addCube(cube_coef, "cube1", v4);
  // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
  // r, g, b, "cube1", v4); // color
  // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
  // 0.5, "cube1", v4);   // 80% opacity

  // 设置长方体的参数
  Eigen::Vector3f translation(2.0, 2.0, 1.0);  // 平移向量
  double yaw = DEG2RAD(0.0);                   // 绕z轴旋转角度
  Eigen::Quaternionf rotation{
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())};  // 四元数（无旋转）
  // 添加长方体到可视化器
  r = 0.0;
  g = 1.0;
  b = 0.0;
  l = 4.0, w = 4.0, h = 2.0;
  viewer.addCube(translation, rotation, l, w, h, "cube2", v4);
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                     r, g, b, "cube2", v4);  // color
  viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                     1.0, "cube2", v4);  // 80% opacity

  r = 0.0;
  g = 0.0;
  b = 1.0;
  viewer.addCube(6.0, 10.0, 0.0, 4.0, 2.0, 4.0, r, g, b, "cube3", v4);

  // 启动可视化循环
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }

  return 0;
}
