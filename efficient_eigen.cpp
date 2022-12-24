#include <iostream>
#include <Eigen/Core>

int main(int argc, char* argv[]) {
  Eigen::MatrixXd m(2, 2);
  m << 1, 0, 0, 1;
  std::cout << m << "\n\n";
  return 0;
}
