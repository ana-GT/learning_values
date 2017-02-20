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
#include <flann/flann.hpp>


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

  Robot_Data_t()  {
    knn_base[0] = new flann::Index<flann::L2<double> >(flann::KDTreeSingleIndexParams() );
    knn_base[1] = new flann::Index<flann::L2<double> >(flann::KDTreeSingleIndexParams() );
  }

  int get_closest_P_base( Eigen::Vector3d _P_base,
			  int _side,
                          double &_distance ) {
    int nearest;
    const flann::Matrix<double> queryMatrix((double*)(_P_base.data()), 1, 3);
    flann::Matrix<int> nearestMatrix(&nearest, 1, 1); double distance;
    flann::Matrix<double> distanceMatrix(flann::Matrix<double>(&distance, 1, 1));
    knn_base[_side]->knnSearch( queryMatrix, nearestMatrix, distanceMatrix, 1, 
				flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));
    _distance = distance;
  return nearest;

  }
  
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

  std::string conf_file[2];

  std::vector<Eigen::Vector3d> P_base[2];
  std::vector<Eigen::Quaterniond> R_base[2];
  std::vector<Eigen::VectorXd> qs[2];
  std::vector<int> num_m[2];
  
  flann::Index<flann::L2<double> >* knn_base[2];  
};

/**
 *
 */
class AnalyzePoses {
 public:
  
  AnalyzePoses();
  void stop();
  void tfListener();

  static void fill_robot_info();
  static void fill_confs();
  
  bool setChain( bool _left = true,
		 unsigned int _robot_type = HUMAN_MODEL );

  bool generate_random_config( int _N,
			       std::vector<Eigen::VectorXd> &_qs );
  
  bool generate_sets_manip( int _N,
			    std::vector<Eigen::VectorXd> &_worst,
			    std::vector<Eigen::VectorXd> &_bad,
			    std::vector<Eigen::VectorXd> &_fair,
			    std::vector<Eigen::VectorXd> &_good );

  bool generate_best_worst_voxel( double _dv,
				  int _n,
				  std::vector<Eigen::VectorXd> &_best,
				  std::vector<Eigen::VectorXd> &_worst,
				  std::vector<geometry_msgs::Point> &_P_target_world );
  void generate_reachable_voxels( double _dv,
				  int _n,
				  std::vector<Eigen::VectorXd> &_qs,
				  std::vector<geometry_msgs::Point> &_P_world,
				  tf::Transform _Tf_world_c,
				  Eigen::Vector3d _xyz_c_min,
				  Eigen::Vector3d _xyz_c_max );
  void generate_reachable_voxels2( std::vector<Eigen::VectorXd> &_qs,
				   std::vector<geometry_msgs::Point> &_P_world,
				   tf::Transform _Tf_world_c,
				   Eigen::Vector3d _xyz_c_min,
				   Eigen::Vector3d _xyz_c_max );
  void prune_reachable_voxels( std::vector<geometry_msgs::Point> _P_world,
			       int _n,
			       std::vector<unsigned int> &_indices,
			       std::vector<Eigen::VectorXd> &_qs );

  void prune_reachable_voxels2( std::vector<geometry_msgs::Point> _P_world,
				std::vector<unsigned int> &_indices,
				std::vector<Eigen::VectorXd> &_qs );

  
  void getGuess( const Eigen::Vector3d &_P_world_target,
		 int _n,
		 std::vector<KDL::Frame> &_Tf_base_target_guess );

  void getGuess( const Eigen::Vector3d &_P_base_target,
		 int _n,
		 std::vector<Eigen::Quaterniond> &_orientations );
  
  void get_suggestions( Eigen::Vector3d S, Eigen::Vector3d Target,
			std::vector<Eigen::Matrix3d> &_R_hands );
  double metric_manip( KDL::JntArray _js );
  KDL::JntArray gen_random_config();
  KDL::Chain get_chain() { return chain; }

  void write_robot_info( std::ofstream &_output,
			 KDL::Frame _orientation,
			 KDL::JntArray _q_max,
			 KDL::JntArray _q_min,
			 int _num_m,
			 double _m_max,
			 double _m_min );
  void generate_and_store( std::string _name,
			   double _dv,
			   int _n );
  TRAC_IK::TRAC_IK* get_ik_solver() { return ik_solver; }
  
 protected:
  TRAC_IK::TRAC_IK* ik_solver;
  KDL::JntArray low_lim, up_lim;
 
  KDL::Chain chain;
  KDL::ChainJntToJacSolver* jnt_to_jac_solver;
  
  KDL::JntArray q_comf;
  double z_palm_direction;
 public:
  std::vector<std::string> joint_names;

  tf::StampedTransform Tf_base_shoulder;  
  tf::StampedTransform Tf_shoulder_base;
  tf::StampedTransform Tf_world_base;
  tf::StampedTransform Tf_base_world;  
  tf::StampedTransform Tf_world_shoulder;
  
  // Frames
  unsigned int robot_type;
  unsigned int side;
  
  std::string world_frame;
  std::string base_frame;
  std::string tip_frame;   
  std::string shoulder_frame;

  Eigen::Vector3d xyz_base_min, xyz_base_max;
  boost::thread tf_listener_t;

  static Robot_Data_t robot_info[NUM_ENTITY_TYPES];

  
  bool run;
};

