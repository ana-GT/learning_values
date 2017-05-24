/**
 * @file interaction.cpp
 * @brief 
 */
#include "interaction.h"
#include <visualization_msgs/Marker.h>
#include <tf_conversions/tf_kdl.h>

/**
 * @function joint_robot_cb
 * @brief Joint state callback for robot
 */
void Interaction::joint_robot_cb( const sensor_msgs::JointState::ConstPtr &_msg ) {
  mMsg_robot_joints = *_msg;
}
void Interaction::joint_human_cb( const sensor_msgs::JointState::ConstPtr &_msg ) {
  mMsg_human_joints = *_msg;
}


/**
 * @function Interaction
 * @brief Constructor
 */
Interaction::Interaction( ros::NodeHandle *_nh ) {
  mNh = _nh;
}

/**
 * @function setHuman
 * @brief Set human model
 */
bool Interaction::setHuman( bool _is_left ) {
  
  bool b; b = mHuman.setChain( _is_left, HUMAN_MODEL );

  std::string topic_in_human, topic_out_human, name;
  name=std::string("human");
  topic_in_human = "/" + name + "/joint_states";
  topic_out_human = "/" + name + "/analyze_joint_states";

  mSub_human= mNh->subscribe(topic_in_human, 1,
			     &Interaction::joint_human_cb,
			     this );  
  mPub_human = mNh->advertise<sensor_msgs::JointState>(topic_out_human, 1 );

  return b;
}

/**
 * @function setRobot
 * @brief 
 */
bool Interaction::setRobot( int _type,
			    bool _is_left ) {
  
  bool b = mRobot.setChain( _is_left, _type );

  std::string topic_in_robot, topic_out_robot, name;

  switch(_type ) {
  case BAXTER: { name=std::string("baxter"); } break;
  case SCHUNK_LWA4: { name=std::string("schunk"); } break;
  }
  topic_in_robot = "/" + name + "/joint_states";
  topic_out_robot = "/" + name + "/analyze_joint_states";

  mSub_robot= mNh->subscribe(topic_in_robot, 1,
			     &Interaction::joint_robot_cb,
			     this );  
  mPub_robot = mNh->advertise<sensor_msgs::JointState>(topic_out_robot, 1 );

  return b;
}

/**
 * @function pub_a_bit
 */
void Interaction::pub_a_bit() {
  
  for( int i = 0; i < 1000; ++i ) {
    ros::spinOnce();
    mPub_robot.publish( mMsg_robot_joints );
    mPub_human.publish( mMsg_human_joints );    
    usleep(0.001*1e6);
  }

}

/**
 * @function stop
 */
void Interaction::stop() {
  mHuman.stop();
  mRobot.stop();  
}

/**
 * @function getCommonPoints
 * @brief 
 */
bool Interaction::getCommonPoints( std::vector<Eigen::VectorXd> &_human_confs,
				   std::vector<Eigen::VectorXd> &_robot_confs,
				   std::vector<geometry_msgs::Point> &_P_world,
				   double _dv ) {

  printf("Init marker \n");
  init_target_marker();

  
  // Find reachability points in robot's workspace
  std::vector<Eigen::VectorXd> human_confs_all;
  std::vector<geometry_msgs::Point> P_world_all;
  std::vector<unsigned int> indices;

  // 1. Get reachable voxels by robot and check if they are within limits
  
  mHuman.generate_reachable_voxels2( human_confs_all,
				     P_world_all,
				     mRobot.Tf_world_base,
				     mRobot.xyz_base_min,
				     mRobot.xyz_base_max );
  
  mRobot.prune_reachable_voxels2( P_world_all,
				  indices,
				  _robot_confs );


  // Store them
  printf("Store them %d \n", indices.size() );
  _human_confs.resize(0);
  _P_world.resize(0);
  for( int i = 0; i < indices.size(); ++i ) {
    _human_confs.push_back( human_confs_all[ indices[i] ] );
    _P_world.push_back( P_world_all[ indices[i] ] );
  }

  // Improvement: Align robot to human hand
  align_improvement( _P_world,
		     _human_confs,
		     _robot_confs );
  
  return (_human_confs.size() > 0 );
}

/**
 * @function align_improvement
 */
void Interaction::align_improvement( const std::vector<geometry_msgs::Point> &_P_world,
				     const std::vector<Eigen::VectorXd> &_human_confs,
				     std::vector<Eigen::VectorXd> &_robot_confs ) {

  KDL::ChainFkSolverPos_recursive fk_robot( mRobot.get_chain() ); 
  KDL::ChainFkSolverPos_recursive fk_human( mHuman.get_chain() ); 
  KDL::Frame ee_base_human, ee_base_robot;
  KDL::JntArray q_robot( mRobot.get_chain().getNrOfJoints() );
  KDL::JntArray q_human( mHuman.get_chain().getNrOfJoints() );
  tf::Transform Tf_base_human, Tf_world_human_ee;
  tf::Transform Tf_base_robot, Tf_world_robot_ee;  
  tf::Vector3 x_wh, z_wr;
  KDL::JntArray qi;
    int changed = 0;
  for( int i = 0; i < _human_confs.size(); ++i ) {
    
    // Get z of end link of human (x_wh)
    q_human.data = _human_confs[i];
    fk_human.JntToCart( q_human,
			ee_base_human );
    tf::transformKDLToTF( ee_base_human, Tf_base_human );
    Tf_world_human_ee = mHuman.Tf_world_base * Tf_base_human;
    x_wh = Tf_world_human_ee.getBasis().getColumn(0);
    // Get z of end link of robot (z_wr)
    q_robot.data = _robot_confs[i];
    fk_robot.JntToCart( q_robot,
			ee_base_robot );
    tf::transformKDLToTF( ee_base_robot, Tf_base_robot );
    Tf_world_robot_ee = mRobot.Tf_world_base * Tf_base_robot;
    z_wr = Tf_world_human_ee.getBasis().getColumn(0);
    int rc;
    KDL::Twist tol;
    tol.rot = KDL::Vector(0.10,0.10,0.10);
    tol.vel = KDL::Vector(0.005, 0.005, 0.005 );

    // Find the rotation between z_wh and -x_wh
    Eigen::Quaterniond q_w;
    q_w.setFromTwoVectors( Eigen::Vector3d(z_wr.getX(), z_wr.getY(), z_wr.getZ() ),
			 Eigen::Vector3d(-x_wh.getX(), -x_wh.getY(), -x_wh.getZ() ) );
    // Rotate R_wh
    tf::Quaternion q1; q1 = Tf_world_robot_ee.getRotation();
    tf::Quaternion q2( q_w.x(), q_w.y(), q_w.z(), q_w.w() );
    Tf_world_robot_ee.setRotation(  q2* q1 );
    // Get R_br
    Tf_base_robot = mRobot.Tf_base_world * Tf_world_robot_ee;
    // Get IK on robot
    KDL::JntArray qc( _robot_confs[i].size() );
    qc.data = _robot_confs[i];
    KDL::Frame Ta;
    tf::TransformTFToKDL( Tf_base_robot, Ta );
    rc = mRobot.get_ik_solver()->CartToJnt( qc,
					    Ta,
					    qi,
					    tol );
    // if true, replace in robot confs
    if( rc >= 0 ) {
      changed++;
      _robot_confs[i] = qi.data;
    }
  }

  printf("Changed %d out of %d initial \n", changed, _human_confs.size() );

}


/**
 * @function view_interaction
 */
bool Interaction::view_interaction( std::vector<Eigen::VectorXd> &_human_confs,
				    std::vector<Eigen::VectorXd> &_robot_confs,
				    std::vector<geometry_msgs::Point> &_P_world,
				    double _dt ) {

  if( _human_confs.size() != _robot_confs.size() || _human_confs.size() != _P_world.size() ) {
    return false;
  }

  mPub_target = mNh->advertise<visualization_msgs::Marker>("target_marker", 0 );

  sensor_msgs::JointState js_msg;
  for( int i = 0; i < _human_confs.size(); ++i ) {

    ros::spinOnce();    
    
    printf("Sending %d / %d \n", i, _human_confs.size() );
    std::cout << _human_confs[i].transpose() << std::endl;
    js_msg = mMsg_human_joints;
    js_msg.header.frame_id = "analyze_joint_states";
    for( int j = 0; j < mHuman.joint_names.size(); ++j ) {      
      for( int k = 0; k < js_msg.name.size(); ++k ) {
	if( mHuman.joint_names[j].compare( js_msg.name[k] ) == 0 ) {
	  js_msg.position[k] = _human_confs[i](j); break;
	} // if
      }// for k
    } // for j
    
    js_msg.header.stamp = ros::Time::now();   
    mPub_human.publish( js_msg );

    // Robot
    js_msg = mMsg_robot_joints;
    js_msg.header.frame_id = "analyze_joint_states";
    for( int j = 0; j < mRobot.joint_names.size(); ++j ) {      
      for( int k = 0; k < js_msg.name.size(); ++k ) {
	if( mRobot.joint_names[j].compare( js_msg.name[k] ) == 0 ) {
	  js_msg.position[k] = _robot_confs[i](j); break;
	} // if
      }// for k
    } // for j
    
    js_msg.header.stamp = ros::Time::now();   
    mPub_robot.publish( js_msg );


    // Marker
    mMarker_target.header.stamp = ros::Time::now();
    mMarker_target.pose.position = _P_world[i];
    mPub_target.publish( mMarker_target );
    usleep(_dt*1e6);
  }

}

/**
 * @function init_marker
 */
void Interaction::init_target_marker() {
  
  std_msgs::ColorRGBA blue;
  blue.r = 0.0; blue.g = 0.0; blue.b = 1.0; blue.a = 1.0; // Blue
  
  mMarker_target.header.frame_id = "world";

  mMarker_target.header.stamp = ros::Time::now();
  mMarker_target.action = visualization_msgs::Marker::ADD;
  mMarker_target.type = visualization_msgs::Marker::SPHERE;
  mMarker_target.scale.x = 0.075;
  mMarker_target.scale.y = 0.075;
  mMarker_target.scale.z = 0.075;
  mMarker_target.pose.position.x = 0.00;
  mMarker_target.pose.position.y = 0.00;
  mMarker_target.pose.position.z = 0.00;
  
  mMarker_target.color = blue;
  mMarker_target.lifetime = ros::Duration();
  mMarker_target.id = 23;  
}

  
