#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <iomanip>

int main() {
    double num1 = 1.0;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << num1;
    std::string str1 = oss.str();
    std::cout << "num1 = " << num1 << std::endl;
    std::cout << "str1 = " << str1 << std::endl;
    return 0;
}
