/**
 * @file interaction.h 
 */
#pragma once

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include "analyze_poses.h"

/**
 * @class Interaction
 */
class Interaction {

public:
  Interaction( ros::NodeHandle *_nh );
  bool setHuman( bool _is_left );
  bool setRobot( int _type,
		 bool _is_left );
  bool getCommonPoints( std::vector<Eigen::VectorXd> &_human_confs,
			std::vector<Eigen::VectorXd> &_robot_confs,
			std::vector<geometry_msgs::Point> &_P_world,
			double _dv );
  void align_improvement( const std::vector<geometry_msgs::Point> &_P_world,
			  const std::vector<Eigen::VectorXd> &_human_confs,
			  std::vector<Eigen::VectorXd> &_robot_confs );
  bool view_interaction( std::vector<Eigen::VectorXd> &_human_confs,
			 std::vector<Eigen::VectorXd> &_robot_confs,
			 std::vector<geometry_msgs::Point> &_P_world,
			 double _dt );
  void joint_robot_cb( const sensor_msgs::JointState::ConstPtr &_msg );
  void joint_human_cb( const sensor_msgs::JointState::ConstPtr &_msg );
  void init_target_marker();
  void pub_a_bit();
  void stop();
  
protected:
  AnalyzePoses mHuman;
  AnalyzePoses mRobot;
  ros::NodeHandle* mNh;

  // 3 publishers
  ros::Publisher mPub_human;
  ros::Publisher mPub_robot;
  ros::Publisher mPub_target;

  ros::Subscriber mSub_human;
  ros::Subscriber mSub_robot;

  sensor_msgs::JointState mMsg_robot_joints;
  sensor_msgs::JointState mMsg_human_joints;
  visualization_msgs::Marker mMarker_target;
  
};

