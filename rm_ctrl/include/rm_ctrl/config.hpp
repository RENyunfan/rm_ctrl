#ifndef _RM_CTRL_CONFIG_
#define _RM_CTRL_CONFIG_

#include "vector"
#include "Eigen/Dense"
#include "string"
#include "common_type_name.hpp"
#include "ros/ros.h"

using namespace std;
namespace rm_ctrl {

class RmCtrlConfig {
 public:
  double kpx,kpy,kvx,kvy,kyaw,kyawdot;
  string cmd_topic, odom_topic, ctrl_topic;

  template<class T>
  bool LoadParam(string param_name, T &param_value, T default_value) {
	if (nh_.getParam(param_name, param_value)) {
	  printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
	  cout << param_value << endl;
	  return true;
	} else {
	  printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
	  param_value = default_value;
	  cout << param_value << endl;
	  return false;
	}
  }

  template<class T>
  bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
	if (nh_.getParam(param_name, param_value)) {
	  printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
	  for (int i = 0; i < param_value.size(); i++) {
		cout << param_value[i] << " ";
	  }
	  cout << endl;
	  return true;
	} else {
	  printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
	  param_value = default_value;
	  for (int i = 0; i < param_value.size(); i++) {
		cout << param_value[i] << " ";
	  }
	  cout << endl;
	  return false;
	}
  }

  ros::NodeHandle nh_;

  RmCtrlConfig() {};

  RmCtrlConfig(const ros::NodeHandle &nh_priv) {
	nh_ = nh_priv;
	LoadParam("position_gain/x", kpx, 2.0);
	LoadParam("position_gain/y", kpy, 2.0);
	LoadParam("velocity_gain/x", kvx, 2.0);
	LoadParam("velocity_gain/y",kvy, 2.0);
	LoadParam("attitude_gain/yaw",kyaw, 2.0);
	LoadParam("attitude_gain/yaw_dot",kyawdot, 2.0);

	LoadParam("topics/cmd_topic", cmd_topic, string("/planning/pos_cmd"));
	LoadParam("topics/odom_topic", odom_topic, string("/lidar_slam/odom"));
	LoadParam("topics/ctrl_topic", ctrl_topic, string("/cmd_vel"));

  }

};
}

#endif