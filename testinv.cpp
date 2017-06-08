#include <iostream>
#include "eigen3/Eigen/Dense"
#include <vector>
using Eigen::MatrixXd;
class compute_jacobian
{
private:
  MatrixXd jacob;
  MatrixXd jacob_inv;
public:
  compute_jacobian()
  {
    jacob.resize(6,6);
    jacob_inv.resize(6,6);
  }
  void simplify_jacobian(std::vector<double>);
  void pseudo_inv(std::vector<double>,MatrixXd);
};
void compute_jacobian::simplify_jacobian(std::vector<double> joint_states)
{
  double q1 = joint_states[0];
  double q2 = joint_states[1];
  double q3 = joint_states[2];
  double q4 = joint_states[3];
  double q5 = joint_states[4];
  double q6 = joint_states[5];

  jacob(0,0) = 270*sin(q1)*sin(q2) + 72*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) + 70*sin(q1)*sin(q2 + q3) - 72*sin(q1)*cos(q5)*cos(q2 + q3) - 302*sin(q1)*cos(q2 + q3) + 72*sin(q4)*sin(q5)*cos(q1);
  jacob(0,1) = -2*(36*sin(q5)*cos(q4)*cos(q2 + q3) + 36*sin(q2 + q3)*cos(q5) + 151*sin(q2 + q3) + 135*cos(q2) + 35*cos(q2 + q3))*cos(q1);
  jacob(0,2) = -2*(36*sin(q5)*cos(q4)*cos(q2 + q3) + 36*sin(q2 + q3)*cos(q5) + 151*sin(q2 + q3) + 35*cos(q2 + q3))*cos(q1);
  jacob(0,3) = 72*(sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q5);
  jacob(0,4) = 72*sin(q1)*sin(q4)*cos(q5) - 72*sin(q5)*cos(q1)*cos(q2 + q3) - 72*sin(q2 + q3)*cos(q1)*cos(q4)*cos(q5);
  jacob(0,5) = 0;

  jacob(1,0) = 72*sin(q1)*sin(q4)*sin(q5) - 270*sin(q2)*cos(q1) - 72*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 70*sin(q2 + q3)*cos(q1) + 72*cos(q1)*cos(q5)*cos(q2 + q3) + 302*cos(q1)*cos(q2 + q3);
  jacob(1,1) = -2*(36*sin(q5)*cos(q4)*cos(q2 + q3) + 36*sin(q2 + q3)*cos(q5) + 151*sin(q2 + q3) + 135*cos(q2) + 35*cos(q2 + q3))*sin(q1);
  jacob(1,2) = -2*(36*sin(q5)*cos(q4)*cos(q2 + q3) + 36*sin(q2 + q3)*cos(q5) + 151*sin(q2 + q3) + 35*cos(q2 + q3))*sin(q1);
  jacob(1,3) = 72*(sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4))*sin(q5);
  jacob(1,4) = -72*sin(q1)*sin(q5)*cos(q2 + q3) - 72*sin(q1)*sin(q2 + q3)*cos(q4)*cos(q5) - 72*sin(q4)*cos(q1)*cos(q5);
  jacob(1,5) = 0;

  jacob(2,0) = 0;
  jacob(2,1) = -270*sin(q2) - 72*sin(q5)*sin(q2 + q3)*cos(q4) - 70*sin(q2 + q3) + 72*cos(q5)*cos(q2 + q3) + 302*cos(q2 + q3);
  jacob(2,2) = -72*sin(q5)*sin(q2 + q3)*cos(q4) - 70*sin(q2 + q3) + 72*cos(q5)*cos(q2 + q3) + 302*cos(q2 + q3);
  jacob(2,3) = -72*sin(q4)*sin(q5)*cos(q2 + q3);
  jacob(2,4) = -72*sin(q5)*sin(q2 + q3) + 72*cos(q4)*cos(q5)*cos(q2 + q3);
  jacob(2,5) = 0;

  jacob(3,0) = sin(q1);
  jacob(3,1) = sin(q1);
  jacob(3,2) = cos(q1)*cos(q2 + q3);
  jacob(3,3) = sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1);
  jacob(3,4) = (sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3);
  jacob(3,5) = (sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3);

  jacob(4,0) = -cos(q1);
  jacob(4,1) = -cos(q1);
  jacob(4,2) = sin(q1)*cos(q2 + q3);
  jacob(4,3) = sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4);
  jacob(4,4) = -(sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3);
  jacob(4,5) = -(sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3);

  jacob(5,0) = 0;
  jacob(5,1) = 0;
  jacob(5,2) = sin(q2 + q3);
  jacob(5,3) = -sin(q4)*cos(q2 + q3);
  jacob(5,4) = sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5);
  jacob(5,5) = sin(q5)*cos(q4)*cos(q2 + q3) + sin(q2 + q3)*cos(q5);

  std::cout << jacob << std::endl;
}
void compute_jacobian::pseudo_inv(std::vector<double> joint_states,MatrixXd end_vel)
{
  simplify_jacobian(joint_states);
  Eigen::JacobiSVD<MatrixXd> svd(jacob, Eigen::ComputeThinU | Eigen::ComputeThinV);
  MatrixXd Cp = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
  MatrixXd diff = Cp - jacob;
  MatrixXd D = svd.singularValues().asDiagonal();
  Cp =svd.matrixV() * D.inverse() * svd.matrixU().transpose();
  std::cout << "diff:\n" << diff.array().abs().sum() << "\n";
  std::cout << "inv:\n" << Cp*end_vel << "\n";
}

int main(int argv, char** argc)
{
  MatrixXd m(2,2);
  MatrixXd m1 = MatrixXd::Random(6, 1);
  compute_jacobian obj;
  // m(0,0) = 3;
  // m(1,0) = 2.5;
  // m(0,1) = -1;
  // m(1,1) = m(1,0) + m(0,1);
  // std::cout << m << std::endl;
  std::vector<double> joint_states;

  for(int i=0;i<5;++i)
    joint_states.push_back(0);

  obj.pseudo_inv(joint_states,m1);
  return 0;
}
