/**
 * @file analyze_poses.h
 * @brief Analyze, well, poses. Duh
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <stdio.h> 
#include <fstream>
#include <Eigen/Geometry>
#include <tf/tf.h>

#include "ChainFkSolverPos_updated.h"

enum ENTITY_TYPE {
  SCHUNK_LWA4 = 0,
  HUMAN_MODEL = 1,
  BAXTER = 2,
  NUM_ENTITY_TYPES = 3
};

/**
 * @structure Robot_data_t
 */
struct Robot_Data_t {
  int num_joints[2];
  std::vector<std::string> joint_names[2];
  std::string urdf_param;
  std::string base_link[2];
  std::string shoulder_link[2];  
  std::string tip_link[2];
  KDL::JntArray q_comf[2];
  Eigen::Vector3d xyz_base_min[2];
  Eigen::Vector3d xyz_base_max[2];
  double z_palm_direction;
  std::string world_frame;

};

/**
 *
 */
class AnalyzePoses {
 public:
  
  AnalyzePoses();
  void stop();
  void tfListener();
  void fill_robot_info();
  bool setChain( bool _left = true,
		 int _robot_type = HUMAN_MODEL );

  bool generate_random_config( int _N,
			       std::vector<Eigen::VectorXd> &_qs );
  
  bool generate_sets_manip( int _N,
			    std::vector<Eigen::VectorXd> &_worst,
			    std::vector<Eigen::VectorXd> &_bad,
			    std::vector<Eigen::VectorXd> &_fair,
			    std::vector<Eigen::VectorXd> &_good );

  bool generate_best_worst_voxel( double _dv,
				  std::vector<Eigen::VectorXd> &_best,
				  std::vector<Eigen::VectorXd> &_worst,
				  std::vector<geometry_msgs::Point> &_P_target_world );
  void generate_reachable_voxels( double _dv,
				  std::vector<Eigen::VectorXd> &_qs,
				  std::vector<geometry_msgs::Point> &_P_world,
				  bool _use_lim,
				  tf::Transform _Tf_world_c,
				  Eigen::Vector3d _xyz_c_min,
				  Eigen::Vector3d _xyz_c_max );
  void prune_reachable_voxels( std::vector<geometry_msgs::Point> _P_world,
			       std::vector<unsigned int> &_indices,
			       std::vector<Eigen::VectorXd> &_qs );
  
  void getGuess( const Eigen::Vector3d &_P_world_target,
		 std::vector<KDL::Frame> &_Tf_base_target_guess );
  
  void get_suggestions( Eigen::Vector3d S, Eigen::Vector3d Target,
			std::vector<Eigen::Matrix3d> &_R_hands );
  double metric_manip( KDL::JntArray _js );
  KDL::JntArray gen_random_config();
  KDL::Chain get_chain() { return chain; }

  
 protected:
  TRAC_IK::TRAC_IK* ik_solver;
  KDL::JntArray low_lim, up_lim;
 
  KDL::Chain chain;
  KDL::JntArray q_comf;
  bool is_left;
  double z_palm_direction;
 public:
  std::vector<std::string> joint_names;
  Robot_Data_t robot_info[NUM_ENTITY_TYPES];

  tf::StampedTransform Tf_base_shoulder;  
  tf::StampedTransform Tf_shoulder_base;
  tf::StampedTransform Tf_world_base;
  tf::StampedTransform Tf_base_world;  
  tf::StampedTransform Tf_world_shoulder;
  
  // Frames
  std::string world_frame;
  std::string base_frame;
  std::string tip_frame;   
  std::string shoulder_frame;

  Eigen::Vector3d xyz_base_min, xyz_base_max;
  boost::thread tf_listener_t;

  bool run;
};

