#include "Utils.h"
#include <cstdlib>
#include <iostream>
#include <ctime>
using namespace kin;
bool Utils::seed = false;
Eigen::Vector3d
Utils::
computeLinearVelocity(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, double dt_inv)
{
  Eigen::Vector3d lin_vel = (p1-p0)*dt_inv;
  return lin_vel;
}

Eigen::Vector3d
Utils::
computeAngularVelocity(const Eigen::Matrix3d& R0, const Eigen::Matrix3d& R1, double dt_inv)
{
  Eigen::AngleAxisd aa(R1.transpose() * R0);

  Eigen::Vector3d ang_vel;
  ang_vel = aa.angle()*aa.axis()*dt_inv;

  return ang_vel;
}

Eigen::Matrix3d
Utils::
eulerXYZToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                        -+
  // | r00 r01 r02 |   |  cy*cz           -cy*sz            sy    |
  // | r10 r11 r12 | = |  cz*sx*sy+cx*sz   cx*cz-sx*sy*sz  -cy*sx |
  // | r20 r21 r22 |   | -cx*cz*sy+sx*sz   cz*sx+cx*sy*sz   cx*cy |
  // +-           -+   +-                                        -+

  Eigen::Matrix3d ret;

  double cx = cos(_angle[0]);
  double sx = sin(_angle[0]);
  double cy = cos(_angle[1]);
  double sy = sin(_angle[1]);
  double cz = cos(_angle[2]);
  double sz = sin(_angle[2]);

  ret(0, 0) = cy*cz;
  ret(1, 0) = cx*sz + cz*sx*sy;
  ret(2, 0) = sx*sz - cx*cz*sy;

  ret(0, 1) = -cy*sz;
  ret(1, 1) =  cx*cz - sx*sy*sz;
  ret(2, 1) =  cz*sx + cx*sy*sz;

  ret(0, 2) =  sy;
  ret(1, 2) = -cy*sx;
  ret(2, 2) =  cx*cy;

  return ret;
}
Eigen::Matrix3d
Utils::
eulerZXYToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                        -+
  // | r00 r01 r02 |   |  cy*cz-sx*sy*sz  -cx*sz   cz*sy+cy*sx*sz |
  // | r10 r11 r12 | = |  cz*sx*sy+cy*sz   cx*cz  -cy*cz*sx+sy*sz |
  // | r20 r21 r22 |   | -cx*sy            sx      cx*cy          |
  // +-           -+   +-                                        -+

  Eigen::Matrix3d ret;

  double cz = cos(_angle(0));
  double sz = sin(_angle(0));
  double cx = cos(_angle(1));
  double sx = sin(_angle(1));
  double cy = cos(_angle(2));
  double sy = sin(_angle(2));

  ret(0, 0) =  cy*cz-sx*sy*sz;
  ret(1, 0) =  cz*sx*sy+cy*sz;
  ret(2, 0) = -cx*sy;

  ret(0, 1) = -cx*sz;
  ret(1, 1) =  cx*cz;
  ret(2, 1) =  sx;

  ret(0, 2) =  cz*sy+cy*sx*sz;
  ret(1, 2) = -cy*cz*sx+sy*sz;
  ret(2, 2) =  cx*cy;

  return ret;
}
Eigen::Matrix3d
Utils::
eulerZYXToMatrix(const Eigen::Vector3d& _angle) {
  // +-           -+   +-                                      -+
  // | r00 r01 r02 |   |  cy*cz  cz*sx*sy-cx*sz  cx*cz*sy+sx*sz |
  // | r10 r11 r12 | = |  cy*sz  cx*cz+sx*sy*sz -cz*sx+cx*sy*sz |
  // | r20 r21 r22 |   | -sy     cy*sx           cx*cy          |
  // +-           -+   +-                                      -+

  Eigen::Matrix3d ret;

  double cz = cos(_angle[0]);
  double sz = sin(_angle[0]);
  double cy = cos(_angle[1]);
  double sy = sin(_angle[1]);
  double cx = cos(_angle[2]);
  double sx = sin(_angle[2]);

  ret(0, 0) =  cz*cy;
  ret(1, 0) =  sz*cy;
  ret(2, 0) = -sy;

  ret(0, 1) = cz*sy*sx - sz*cx;
  ret(1, 1) = sz*sy*sx + cz*cx;
  ret(2, 1) = cy*sx;

  ret(0, 2) = cz*sy*cx + sz*sx;
  ret(1, 2) = sz*sy*cx - cz*sx;
  ret(2, 2) = cy*cx;

  return ret;
}

double
Utils::
computeAngleDiff(double a0, double a1)
{
  Eigen::AngleAxisd aa0(a0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd aa1(a1, Eigen::Vector3d::UnitY());

  Eigen::AngleAxisd aa01(aa0.toRotationMatrix().transpose()*(aa1.toRotationMatrix()));

  return aa01.angle()*(aa01.axis()[1]);
}
// double
// Utils::
// uniformd(double min, double max)
// {
//   if(seed == false)
//     std::srand(std::time(nullptr));
// }
int
Utils::
uniformi(int min, int max)
{
  if(seed == false)
    std::srand(std::time(nullptr));
  int val = std::rand();
  val = val%(max-min) + min;
  return val;
}
// Eigen::VectorXd
// Utils::
// uniformXd(double min, double max)
// {
//   if(seed == false)
//     std::srand(std::time(nullptr));
// }