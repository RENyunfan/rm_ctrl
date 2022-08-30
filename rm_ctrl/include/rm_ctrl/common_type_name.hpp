#ifndef _EIGEN_TYPE_NAME_
#define _EIGEN_TYPE_NAME_

#include "Eigen/Dense"
#include "fmt/color.h"
#include "ros/ros.h"
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "data/"+name))
#define simplify_print_log 1
using namespace fmt;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 3, 3> Mat33;

typedef Eigen::Matrix<double, 3, 3> StatePVA;
typedef Eigen::Matrix<double, 3, 4> StatePVAJ;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DynamicMat;
typedef Eigen::MatrixX4d MatX4;

#endif