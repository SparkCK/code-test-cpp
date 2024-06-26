#include <Eigen/Dense>
#include <fstream>
#include <iostream>

int main() {
  // 定义一个4x4的Eigen矩阵
  Eigen::Matrix4d transform;

  // 打开文件
  std::ifstream file("1692065400105008.txt");
  if (!file.is_open()) {
    std::cerr << "无法打开文件" << std::endl;
    return 1;
  }

  // 读取文件内容并填充到矩阵中
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      file >> transform(i, j);
    }
  }

  // 打印矩阵
  std::cout << "读取的矩阵内容：" << std::endl;
  std::cout << transform << std::endl;

  Eigen::Affine3d affine(transform);

  std::cout << "affine1:" << affine.data() << std::endl;
  std::cout << "affine2:" << affine.cols() << std::endl;
  std::cout << "affine3:" << affine.rows() << std::endl;

  // 关闭文件
  file.close();

  Eigen::Affine3d aff;
  aff.setIdentity();
  std::cout << "aff1:" << aff.data() << std::endl;
  std::cout << "aff2:" << aff.cols() << std::endl;
  std::cout << "aff3:" << aff.rows() << std::endl;

  Eigen::Matrix4d matrix_aff = aff.matrix();

  std::cout << "matrix_aff:\n" << matrix_aff << std::endl;
  return 0;
}
