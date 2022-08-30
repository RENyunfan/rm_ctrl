#ifndef _RM_CTRL_
#define _RM_CTRL_

#include "vector"
#include "Eigen/Dense"
#include "string"
#include "common_type_name.hpp"
#include "ros/ros.h"
#include "config.hpp"
#include "quadrotor_msgs/PositionCommand.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "quadrotor_msgs/TrakingPerformance.h"
#include "tf/transform_datatypes.h"
using namespace std;
namespace rm_ctrl {

geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3 &a, const geometry_msgs::Vector3 &b) {
  geometry_msgs::Vector3 c;
  c.x = a.x + b.x;
  c.y = a.y + b.y;
  c.z = a.z + b.z;
  return c;
}

geometry_msgs::Vector3 operator-(const geometry_msgs::Vector3 &a, const geometry_msgs::Vector3 &b) {
  geometry_msgs::Vector3 c;
  c.x = a.x - b.x;
  c.y = a.y - b.y;
  c.z = a.z - b.z;
  return c;
}

geometry_msgs::Point operator-(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
  geometry_msgs::Point c;
  c.x = a.x - b.x;
  c.y = a.y - b.y;
  c.z = a.z - b.z;
  return c;
}

class RmCtrl {

 private:
  RmCtrlConfig cfg_;
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_, odom_sub_;
  ros::Publisher vel_pub_, tpm_pub_;
  ros::Timer main_ctrl_timer_;
  double odom_rcv_t_{0};
  double cmd_rcv_t_{0};
  geometry_msgs::TwistStamped twist_cmd;
  quadrotor_msgs::TrakingPerformance tpm_;

  struct Feedback {
	double v_x, v_y;
	double p_x, p_y;
	double yaw;
  } fdb_;
  quadrotor_msgs::PositionCommand cmd_;

 public:
  RmCtrl(ros::NodeHandle &nh) : nh_(nh) {
	cfg_ = RmCtrlConfig(nh_);
	cmd_sub_ = nh_.subscribe(cfg_.cmd_topic, 10, &RmCtrl::CmdCallback, this);
	odom_sub_ = nh_.subscribe(cfg_.odom_topic, 10, &RmCtrl::OdomCallback, this);
	vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(cfg_.odom_topic, 10);
	tpm_pub_ = nh_.advertise<quadrotor_msgs::TrakingPerformance>("rm_tracking_performance", 10);
	main_ctrl_timer_ = nh_.createTimer(ros::Duration(0.01), &RmCtrl::MainCtrlCallback, this);
  }

 private:
  double angleMainValue(double ang) {
	while (ang > M_PI) {
	  ang -= 2 * M_PI;
	}
	while (ang < -M_PI) {
	  ang += 2 * M_PI;
	}
	return ang;
  }

  void MainCtrlCallback(const ros::TimerEvent &e) {
	static ros::Time last_call = ros::Time::now();
	ros::Time cur_t = ros::Time::now();
	bool have_odom = cur_t.toSec() - odom_rcv_t_ < 0.5;
	bool have_cmd = cur_t.toSec() - cmd_rcv_t_ < 0.5;
	if ((cur_t - last_call).toSec() > 1) {
	  last_call = cur_t;
	  if (!have_odom) {
		print(fg(color::gold), " -- [RM] No odom.\n");
	  } else {
		print(fg(color::green), " -- [RM] Ctrl woring.\n");
	  };
	  if (!have_cmd) {
		print(fg(color::gold), " -- [RM] No ctrl command, waiting.\n");
	  }
	}

	if (!have_cmd || !have_odom) {
	  twist_cmd.header.stamp = ros::Time::now();
	  twist_cmd.twist.linear.x = 0;
	  twist_cmd.twist.linear.y = 0;
	  twist_cmd.twist.angular.z = 0;
	  twist_cmd.header.frame_id = "world";
	  vel_pub_.publish(twist_cmd);
	  return;
	}

	{
	  tpm_.pos_error = tpm_.desire_pos - tpm_.feedback_pos;
	  tpm_.vel_error = tpm_.desire_vel - tpm_.feedback_vel;
	  tpm_.att_error.z = tpm_.yaw - tpm_.feedback_att.z;
	  tpm_.header.stamp = ros::Time::now();
	  tpm_.header.frame_id = "world";
	  tpm_pub_.publish(tpm_);
	}

	double outOmg = (cfg_.kyaw * angleMainValue(cmd_.yaw - fdb_.yaw) + cfg_.kyawdot * cmd_.yaw_dot);

	double outVx = cfg_.kvx * cmd_.velocity.x + cfg_.kpx * (cmd_.position.x - fdb_.p_x);

	double outVy = cfg_.kvy * cmd_.velocity.y + cfg_.kpy * (cmd_.position.y - fdb_.p_y);;

	twist_cmd.header.stamp = ros::Time::now();
	twist_cmd.twist.linear.x = outVx;
	twist_cmd.twist.linear.y = outVy;
	twist_cmd.twist.angular.z = outOmg;
	twist_cmd.header.frame_id = "world";
	vel_pub_.publish(twist_cmd);


  }

  void OdomCallback(const nav_msgs::OdometryConstPtr &odom) {
	odom_rcv_t_ = ros::Time::now().toSec();
	fdb_.p_x = odom->pose.pose.position.x;
	fdb_.p_y = odom->pose.pose.position.y;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);
	double roll, pitch, yaw;//定义存储r\p\y的容器
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
	fdb_.yaw = yaw;
	fdb_.v_x = odom->twist.twist.linear.x;
	fdb_.v_y = odom->twist.twist.linear.y;

	tpm_.feedback_pos = odom->pose.pose.position;
	tpm_.feedback_vel = odom->twist.twist.linear;
	tpm_.feedback_att.x = roll;
	tpm_.feedback_att.y = pitch;
	tpm_.feedback_att.z = yaw;

  }

  void CmdCallback(const quadrotor_msgs::PositionCommandConstPtr &msg) {
	cmd_ = *msg;
	cmd_rcv_t_ = ros::Time::now().toSec();

	tpm_.desire_pos = msg->position;
	tpm_.desire_vel = msg->velocity;
	tpm_.yaw = msg->yaw;
	tpm_.yaw_dot = msg->yaw_dot;
  }

};
}

#endif