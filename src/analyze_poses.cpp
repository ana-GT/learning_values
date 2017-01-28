/**
 * @file analyze_poses.cpp
 */
#include "analyze_poses.h"
#include <algorithm>
#include <iostream>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <tf_conversions/tf_kdl.h>

void print_Tf( tf::Transform _Tf ) {
  tf::Vector3 rx, ry, rz, p;
  rx = _Tf.getBasis().getRow(0);
  ry = _Tf.getBasis().getRow(1);
  rz = _Tf.getBasis().getRow(2);
  p = _Tf.getOrigin();
  printf("\n%f %f %f %f \n %f %f %f %f \n %f %f %f %f \n",
	 rx.getX(), rx.getY(), rx.getZ(), p.getX(),
	 ry.getX(), ry.getY(), ry.getZ(), p.getY(),
	 rz.getX(), rz.getY(), rz.getZ(), p.getZ() );
}

  
double random_val(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

/**
 * @function fill_robot_info
 */
void AnalyzePoses::fill_robot_info() {

  int ri; // Robot index
  int nj; // Number of joints
  Eigen::VectorXd q_def;
  
  //***************************************
  ri = 0; nj = 7; q_def.resize(nj);
  // 1. Schunk
  robot_info[ri].world_frame = std::string("world");
  robot_info[ri].urdf_param = std::string( "schunk_description" );
  robot_info[ri].z_palm_direction = 1;
  
  for( int l = 0; l < 2; ++l ) { // Left/Right

    std::string side;
    if( l == 0 ) { side = "left"; } else { side="right"; }
    
    robot_info[ri].num_joints[l] = nj;
    robot_info[ri].joint_names[l].resize(nj);
    robot_info[ri].base_link[l] = "mid_link"; //side + "_arm_base_link";
    robot_info[ri].tip_link[l] = side + "_sdh_grasp_link";
    robot_info[ri].shoulder_link[l] = side + "_arm_base_link";

    for( int i = 0; i < nj; ++i ) { char n[3];
      robot_info[ri].joint_names[l][i] = side + "_arm_" + std::to_string(i+1) + "_joint";
    }
    
  } // for l
    
  // Left
  robot_info[ri].q_comf[0].resize( nj );
  q_def << -30, -45, 30, -90, -30, -60, 0; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[0].data = q_def;

  robot_info[ri].xyz_base_min[0] << 0.05, 0.05, -0.50;
  robot_info[ri].xyz_base_max[0] << 0.55, 0.55, 0.60;

  // Right
  robot_info[ri].q_comf[1].resize( nj );
  q_def << -30, -45, 30, -90, -30, -60, 0; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[1].data = q_def;
  
  robot_info[ri].xyz_base_min[1] << 0.05, -0.55, -0.50;
  robot_info[ri].xyz_base_max[1] << 0.55, -0.05, 1.60;

  
  //***********************
  // Human
  ri = 1; nj = 7; q_def.resize(nj);
  robot_info[ri].world_frame = std::string("world");
  robot_info[ri].urdf_param = std::string( "human_description" );
  robot_info[ri].z_palm_direction = -1;

  std::string names[7] = {"shoulder_0", "shoulder_1","shoulder_2",
			  "elbow_0","wrist_0", "wrist_1", "wrist_2"};
  
  for( int l = 0; l < 2; ++l ) { 
    robot_info[ri].num_joints[l] = nj;
    robot_info[ri].joint_names[l].resize(nj);
    std::string side; if( l == 0 ) { side = "left"; } else { side="right"; }
    robot_info[ri].base_link[l] = "human/spine"; //"human/" + side + "_arm_mount";
    robot_info[ri].tip_link[l] = "human/" + side + "_hand";  
    robot_info[ri].shoulder_link[l] = "human/" + side + "_shoulder_0";
    
    for( int i = 0; i < nj; ++i ) {
      robot_info[ri].joint_names[l][i] = side + "_" + names[i];
    }
  }

  // Left
  robot_info[ri].q_comf[0].resize( nj );
  q_def << 45, -45, 0, -45, 0, 0, -45; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[0].data = q_def;

  robot_info[ri].xyz_base_min[0] << 0.05, 0.05, -0.20;
  robot_info[ri].xyz_base_max[0] << 0.80, 0.80, 0.80;

  // Right
  robot_info[ri].q_comf[1].resize( nj );
  q_def << 45, 45,0, 45,0,0,45; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[1].data = q_def;

  robot_info[ri].xyz_base_min[1] << 0.05, -0.80, -0.20;
  robot_info[ri].xyz_base_max[1] << 0.80, -0.05, 0.80;

  //***********************
  // Baxter
  ri = 2; nj = 7; q_def.resize(nj);

  robot_info[ri].world_frame = std::string("world");
  robot_info[ri].urdf_param = std::string( "baxter_description" );
  robot_info[ri].z_palm_direction = 1;

  std::string bnames[7] = {"s0", "s1","e0", "e1","w0", "w1", "w2"};
  
  for( int l = 0; l < 2; ++l ) { // Left/Right
    robot_info[ri].num_joints[l] = nj;
    robot_info[ri].joint_names[l].resize(nj);
    std::string side; if( l == 0 ) { side = "left"; } else { side="right"; }
    robot_info[ri].base_link[l] = "torso"; //side + "_arm_mount";
    robot_info[ri].tip_link[l] = side + "_gripper"; // Used to be hand_ee  
    robot_info[ri].shoulder_link[l] = side + "_upper_elbow";
    
    for( int i = 0; i < nj; ++i ) {
      robot_info[ri].joint_names[l][i] = side + "_" + bnames[i];
    }
  }

  // Left
  robot_info[ri].q_comf[0].resize( nj );
  q_def << 0, 45, -90, 90, 45, 45, 0; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[0].data = q_def;

  robot_info[ri].xyz_base_min[0] << 0.15, 0.05, 0.00;
  robot_info[ri].xyz_base_max[0] << 1.00, 0.90, 0.90;

  // Right
  robot_info[ri].q_comf[1].resize( nj );
  q_def << 0, 45, 90, 90, -45 , 45, 0; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[1].data = q_def;

  robot_info[ri].xyz_base_min[1] << 0.15, -0.90, 0.00;
  robot_info[ri].xyz_base_max[1] << 1.00, -0.05, 0.90;
  
}


/**
 * @function AnalyzePoses
 * @brief Constructor
 */
AnalyzePoses::AnalyzePoses() :
  ik_solver(0),
  world_frame(""),
  base_frame(""),
  shoulder_frame("") {

  fill_robot_info();
  
  // Start thread
  run = true;
  tf_listener_t = boost::thread( &AnalyzePoses::tfListener, this );

}

void AnalyzePoses::stop() {
  run = false;
  usleep(0.1*1e6);
  tf_listener_t.interrupt();
  ik_solver = 0;
}

/**
 * @function tfListener
 */
void AnalyzePoses::tfListener() {
  
  tf::TransformListener listener;
  double frequency = 100.0; // Hz
  ros::Rate tf_rate( frequency );

  listener.waitForTransform( world_frame,
			     base_frame, ros::Time(), ros::Duration(0.05) );

  while( ros::ok() && run == true ) {

    if( world_frame.size() > 4 ) {
      try {
	
	listener.lookupTransform( world_frame,
				  base_frame, ros::Time(),
				  this->Tf_world_base );
      } catch ( tf::TransformException ex ) {
	ROS_ERROR( "Error when listening transform %s", ex.what() );
      }

      try {
	listener.lookupTransform( base_frame,
				  shoulder_frame, ros::Time(),
				  this->Tf_base_shoulder );
      }  catch ( tf::TransformException ex ) {
	ROS_ERROR( "Error when listening transform %s", ex.what() );
      }


      this->Tf_base_world.setData( this->Tf_world_base.inverse() );
      this->Tf_world_shoulder.setData(  this->Tf_world_base * this->Tf_base_shoulder );
      this->Tf_shoulder_base.setData( this->Tf_base_shoulder.inverse() );

    } // end if    

    tf_rate.sleep();
  } // while end
//  printf("GOT OUT\n");
}


/**
 * @function setChain
 * @brief Set left or right arm
 */
bool AnalyzePoses::setChain( bool _left,
			     int _robot_type ) {

  this->is_left = _left;
  std::string urdf_param;

  int index;
  urdf_param = robot_info[_robot_type].urdf_param;
  this->z_palm_direction = robot_info[_robot_type].z_palm_direction;
  
  if( _left ) { index = 0; } else { index = 1; }  
  this->q_comf = robot_info[_robot_type].q_comf[index];
  this->joint_names = robot_info[_robot_type].joint_names[index];
  this->world_frame = robot_info[_robot_type].world_frame;

  this->base_frame = robot_info[_robot_type].base_link[index];
  this->shoulder_frame = robot_info[_robot_type].shoulder_link[index];
  this->tip_frame = robot_info[_robot_type].tip_link[index];

  this->xyz_base_min = robot_info[_robot_type].xyz_base_min[index];
  this->xyz_base_max = robot_info[_robot_type].xyz_base_max[index];
  
  double timeout_in_secs = 0.005;
  double error=1e-5; 
  TRAC_IK::SolveType type=TRAC_IK::Speed;

  ik_solver = 0;
  ik_solver = new TRAC_IK::TRAC_IK( base_frame,
				    tip_frame,
				    urdf_param,
				    timeout_in_secs,
				    error,
				    type );  
  

  bool valid = this->ik_solver->getKDLChain( this->chain );
  if( !valid ) {
    printf("There was not valid KDL chain found \n");
    return false;
  }

  valid = this->ik_solver->getKDLLimits( low_lim, up_lim );
  if( !valid ) {
    printf("There were not valid limits found! \n") ;
    return false;
  }


  return true;
}

/**
 * @function generate_random_config
 * @brief Generate random joint configurations within joint limits
 */
bool AnalyzePoses::generate_random_config( int _N,
					   std::vector<Eigen::VectorXd> &_qs ) {
  _qs.resize(0);
  for( unsigned int i = 0; i < _N; ++i ) {
    _qs.push_back( this->gen_random_config().data );
  }
}

/**
 * @function generate_sets_manip
 * @brief Generate a random of joint confs and separate them by manip
 */
bool AnalyzePoses::generate_sets_manip( int _N,
					std::vector<Eigen::VectorXd> &_worst,
					std::vector<Eigen::VectorXd> &_bad,
					std::vector<Eigen::VectorXd> &_fair,
					std::vector<Eigen::VectorXd> &_good ) {

  std::vector<Eigen::VectorXd> qs;
  std::vector<double> m;
  
  this->generate_random_config( _N, qs );
  KDL::JntArray q;
  for( int i = 0; i < _N; ++i ) {
    q.data = qs[i];
    m.push_back( metric_manip( q ) ); 
  }

  // Get min and max
  double m_min, m_max, p1, p2, p3;
  m_min = *(std::min_element( m.begin(), m.end() ) );
  m_max = *(std::max_element( m.begin(), m.end() ) );

  p1 = (m_min*3 + m_max*1)/4.0;
  p2 = (m_min*2 + m_max*2)/4.0;
  p3 = (m_min*1 + m_max*3)/4.0;  

  for( int i = 0; i < _N; ++i ) {
    if( m[i] < p1 ) { _worst.push_back( qs[i] ); }
    else if( m[i] < p2 ) { _bad.push_back( qs[i] ); }
    else if( m[i] < p3 ) { _fair.push_back( qs[i] ); }
    else { _good.push_back( qs[i] ); }
  } 
  
  return true;
}

/**
 * @function generate_reachable_voxels
 */
void AnalyzePoses::generate_reachable_voxels( double _dv,
					      std::vector<Eigen::VectorXd> &_qs,
					      std::vector<geometry_msgs::Point> &_P_world,
					      bool _use_lim,
					      tf::Transform _Tf_world_c,
					      Eigen::Vector3d _xyz_c_min,
					      Eigen::Vector3d _xyz_c_max ) {

  _qs.resize(0);
  _P_world.resize(0);
  
  int rc; KDL::JntArray qi;
  
  KDL::Twist tol;
  tol.rot = KDL::Vector(0.5,0.5,0.5);
  tol.vel = KDL::Vector(0.005, 0.005, 0.005 );
  
  // Generate 3D positions to check
  Eigen::Vector3d P_bt;
  std::vector<Eigen::Vector3d> P_base_targets;
  std::vector<geometry_msgs::Point> P_world_targets;
  std::vector<KDL::Frame> Tf_base_target_guess;
  geometry_msgs::Point p;
  tf::Vector3 p_wt, p_ct;
  
  int ns = 0;
  int i, j, k;
  printf("Tf_world base human:  \n" );
  print_Tf( _Tf_world_c );

  printf("Tf_world base robot:\n" );
  print_Tf( this->Tf_world_base );

  
  tf::Transform Tcw; Tcw = _Tf_world_c.inverse();
  for( i = 0; i < (int)((xyz_base_max(0)-xyz_base_min(0))/_dv); ++i ) {
    for( j = 0; j < (int)((xyz_base_max(1)-xyz_base_min(1))/_dv); ++j ) {
      for( k = 0; k < (int)((xyz_base_max(2)-xyz_base_min(2))/_dv); ++k ) {	
	// Generate suggestions
	ros::spinOnce();
	P_bt << this->xyz_base_min + Eigen::Vector3d( _dv*i, _dv*j, _dv*k );
	// See if it is in limit argument
	// 1. Put in world frame
	p_wt = this->Tf_world_base * tf::Vector3(P_bt(0), P_bt(1), P_bt(2) );
	// Put it in base reference frame
	p_ct = Tcw * p_wt;
	
	// If within limits, store it
	if( p_ct.getX() > _xyz_c_min(0) && p_ct.getX() < _xyz_c_max(0) &&
	    p_ct.getY() > _xyz_c_min(1) && p_ct.getY() < _xyz_c_max(1) &&
	    p_ct.getZ() > _xyz_c_min(2) && p_ct.getZ() < _xyz_c_max(2) ) {
	  p.x = p_wt.getX(); p.y = p_wt.getY(); p.z = p_wt.getZ();
	  P_world_targets.push_back( p );
	  P_base_targets.push_back( P_bt );
	}
      }  // for k
    }  // for j
  } // for i

  // Try them out
  for( int i = 0; i < P_base_targets.size(); ++i ) {
    
    ros::spinOnce();	
    this->getGuess( P_base_targets[i],
		    Tf_base_target_guess );
	
    double m_min = 1000; double m_max = 0; double m;
    KDL::JntArray q_m_min, q_m_max;
    
    for( int l = 0; l < Tf_base_target_guess.size(); ++l ) {
      // Generate a lot of random seeds 1000 to get different final poses
      rc = ik_solver->CartToJnt( q_comf, Tf_base_target_guess[l], qi, tol );
      if( rc < 0 ) { ns++; continue; }
      else {
	m = metric_manip(qi);
	if( m < m_min ) { m_min = m; q_m_min = qi; } 
	if( m > m_max ) { m_max = m; q_m_max = qi; }
      } // else
    } // for l
    if( m_min != 1000 && m_max != 0 ) {
      _qs.push_back( q_m_max.data ); 
      _P_world.push_back( P_world_targets[i] );	 
    }
    
  } // for i
}

/**
 * @function prune_reachable_voxels
 */
void AnalyzePoses::prune_reachable_voxels( std::vector<geometry_msgs::Point> _P_world,
					   std::vector<unsigned int> &_indices,
					   std::vector<Eigen::VectorXd> &_qs ) {
  // Check with these points
  _qs.resize(0);
  _indices.resize(0);
  
  int rc; KDL::JntArray qi;
  
  KDL::Twist tol;
  tol.rot = KDL::Vector(0.5,0.5,0.5);
  tol.vel = KDL::Vector(0.005, 0.005, 0.005 );
  
  // Generate 3D positions to check 
  Eigen::Vector3d P_base_target;
  std::vector<KDL::Frame> Tf_base_target_guess;
  
  int ns = 0;
  int i, j, k;
  printf("Trying %d points \n", _P_world.size() );
  for( i = 0; i < _P_world.size(); ++i ) {
    
    ros::spinOnce(); tf::Vector3 Pi_base_target;
    Pi_base_target = this->Tf_base_world * tf::Vector3(_P_world[i].x,
						       _P_world[i].y,
						       _P_world[i].z );
    P_base_target << Pi_base_target.getX(),
      Pi_base_target.getY(),
      Pi_base_target.getZ();
    this->getGuess( P_base_target,
		    Tf_base_target_guess );
    double m_min = 1000; double m_max = 0; double m;
    KDL::JntArray q_m_min, q_m_max;    
    for( int l = 0; l < Tf_base_target_guess.size(); ++l ) {
      // Generate a lot of random seeds 1000 to get different final poses
      rc = ik_solver->CartToJnt( q_comf, Tf_base_target_guess[l], qi, tol );
      if( rc < 0 ) { ns++; continue; }
      else {
	m = metric_manip(qi);
	if( m < m_min ) { m_min = m; q_m_min = qi; } 
	if( m > m_max ) { m_max = m; q_m_max = qi; }
      } // else
    } // for l
    if( m_min != 1000 && m_max != 0 ) {
      _qs.push_back( q_m_max.data );
      _indices.push_back( i );
    }

  } // i

  printf("Qs out size: %d \n", _qs.size() );
}

/**
 * @function generate_best_worst
 */
bool AnalyzePoses::generate_best_worst_voxel( double _dv,
					      std::vector<Eigen::VectorXd> &_best,
					      std::vector<Eigen::VectorXd> &_worst,
					      std::vector<geometry_msgs::Point> &_P_target_world ) {

 int rc; KDL::JntArray qi;
 
 KDL::Twist tol;
 tol.rot = KDL::Vector(0.5,0.5,0.5);
 tol.vel = KDL::Vector(0.005, 0.005, 0.005 );
   
 // Generate 3D positions to check 
 printf("Start calculation in voxels \n");
 Eigen::Vector3d P_base_target;
 std::vector<KDL::Frame> Tf_base_target_guess;

 int ns = 0;
 int i, j, k;
 for( i = 0; i < (int)((xyz_base_max(0)-xyz_base_min(0))/_dv); ++i ) {
   printf("I (x) : %d \n", i );
   for( j = 0; j < (int)((xyz_base_max(1)-xyz_base_min(1))/_dv); ++j ) {
     for( k = 0; k < (int)((xyz_base_max(2)-xyz_base_min(2))/_dv); ++k ) {
       ros::spinOnce();
       // Generate suggestions
       P_base_target << xyz_base_min + Eigen::Vector3d( _dv*i, _dv*j, _dv*k );

       this->getGuess( P_base_target, Tf_base_target_guess );

       double m_min = 1000; double m_max = 0; double m;
       KDL::JntArray q_m_min, q_m_max;

       for( int l = 0; l < Tf_base_target_guess.size(); ++l ) {
	 // Generate a lot of random seeds 1000 to get different final poses
	 rc = ik_solver->CartToJnt( q_comf, Tf_base_target_guess[l], qi, tol );
	 if( rc < 0 ) { ns++; continue; }
	 else {
	   m = metric_manip(qi);
	   if( m < m_min ) { m_min = m; q_m_min = qi; } 
	   if( m > m_max ) { m_max = m; q_m_max = qi; }
	 } // else
       } // for l
       if( m_min != 1000 && m_max != 0 ) {
	 _best.push_back( q_m_min.data ); //( qi.data );
	 _worst.push_back( q_m_max.data );
	 geometry_msgs::Point p;
	 tf::Vector3 pi;
	 pi = this->Tf_world_base * tf::Vector3(P_base_target(0), P_base_target(1), P_base_target(2) );
	 p.x = pi.getX(); p.y = pi.getY(); p.z = pi.getZ();
	 _P_target_world.push_back( p );	 
       }
     
     }  // for k
   }  // for j
 } // for i

 
 printf("End calculation in voxels : No Solution :%d/%d \n", ns, i*j*k);

}


//////////////////// HELPERS //////////////////////

/**
 * @function metric_manip
 * @brief Standard manipulability
 */
double AnalyzePoses::metric_manip( KDL::JntArray _js ) {
  
  KDL::Jacobian J;
  KDL::ChainJntToJacSolver jnt_to_jac_solver( chain );
  J.resize( chain.getNrOfJoints() );
  jnt_to_jac_solver.JntToJac( _js, J );
  /*  
  Eigen::MatrixXd Jm; Jm = J.data;
  Eigen::MatrixXd JJt = ( Jm*Jm.transpose() );
  double d = JJt.determinant();
  return sqrt(d);
  */
  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver( J.data );
  Eigen::MatrixXd singular_values = svdsolver.singularValues();
  
  double error = 1.0;
  for(unsigned int i=0; i < singular_values.rows(); ++i)
    error *= singular_values(i,0);
  return error;
  
}


/**
 * @function gen_random_config
 * @brief Generte random joint configuration
 */
KDL::JntArray AnalyzePoses::gen_random_config() {

  KDL::JntArray q( low_lim.data.size() );
  
  for( unsigned int j = 0; j < low_lim.data.size(); ++j ) {
    q(j) = random_val( low_lim(j), up_lim(j) );
  }

  return q;
}

/**
 * @function getGuess
 */
void AnalyzePoses::getGuess( const Eigen::Vector3d &_P_base_target,
			     std::vector<KDL::Frame> &_Tf_base_target_guess ) {

  _Tf_base_target_guess.resize(0);
	
  Eigen::Vector3d z_base_def; Eigen::Vector3d z_base_st;
  Eigen::Vector3d y_base_def; Eigen::Vector3d y_base_st;
  Eigen::Vector3d x_base_def; Eigen::Vector3d x_base_st;
  Eigen::Matrix3d M_base_def; Eigen::Matrix3d M_base_st;
  
  // 1. Default approach direction (base frame)
  if( is_left ) { z_base_def << 0,-1,0; }
  else { z_base_def << 0, 1, 0; }
  
  z_base_def *= this->z_palm_direction;
  // 2. Coarse approximation (shoulder-target)
  tf::Vector3 bs; bs = this->Tf_base_shoulder.getOrigin();
  z_base_st << _P_base_target(0) - bs.getX(),
    _P_base_target(1) - bs.getY(),
    _P_base_target(2) - bs.getZ();
  z_base_st.normalize();
  z_base_st *= this->z_palm_direction; 

  // 3. Get y as perpendicular to z and in xy plane (a,b,0)
  x_base_def << -z_base_def(1), z_base_def(0), 0;
  y_base_def = z_base_def.cross( x_base_def ); 
  M_base_def.col(0) = x_base_def.transpose();
  M_base_def.col(1) = y_base_def.transpose();
  M_base_def.col(2) = z_base_def.transpose();
  
  y_base_st << -z_base_st(1), z_base_st(0), 0;
  x_base_st = y_base_st.cross( z_base_st ); 
  M_base_st.col(0) = x_base_st.transpose();
  M_base_st.col(1) = y_base_st.transpose();
  M_base_st.col(2) = z_base_st.transpose();
  
  Eigen::Quaterniond q_base_def( M_base_def );
  Eigen::Quaterniond q_base_st( M_base_st );
  
  // 4. Get vector between these two
  Eigen::Quaterniond q; KDL::Frame fi;
  for( int i = 0; i < 10; ++i ) {
    ros::spinOnce();
    q = q_base_st.slerp( 0.1*i, q_base_def );
    tf::Transform T_base_target;
    T_base_target.setIdentity();
    T_base_target.setOrigin(tf::Vector3(_P_base_target(0),
					_P_base_target(1),
					_P_base_target(2) ) );
    T_base_target.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
    tf::TransformTFToKDL( T_base_target, fi );    
    _Tf_base_target_guess.push_back( fi );
  }
}


/**
 * @function get_suggestions
 */
void AnalyzePoses::get_suggestions( Eigen::Vector3d S, Eigen::Vector3d Target,
		     std::vector<Eigen::Matrix3d> &_R_hands ) {

  _R_hands.resize(0);
  
  // Distances
  double d_es = 0.34; // Shoulder to elbow
  double d_eh = 0.37; // Elbow to hand (elbow-wrist + wrist-hand)
  double d_ht = 0.05; // Hand to target

   // Points
   Eigen::Vector3d E; // Elbow
   Eigen::Vector3d H; // Hand

   // Meanwhile
   Eigen::Vector3d u_st; 
   double d_et; double d_st; double d_e_ts;
   double alpha, gamma, beta;
   Eigen::Vector3d elbow_circle_center;
   
   // Calculate   
   u_st = Target - S; // Unit vector from Shoulder to Target 
   u_st.normalize();

   d_st = (Target - S ).norm();
   d_et = sqrt( d_eh*d_eh + d_ht*d_ht );
   alpha = acos( (d_st*d_st + d_es*d_es - d_et*d_et)/(2*d_st*d_es) );   

   if( std::isnan(alpha) ) {
     printf("ALPHA IS NAN!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
     _R_hands.resize(0);
     return;
   }
   
   elbow_circle_center = S + d_es*cos(alpha)*u_st; 
   d_e_ts = d_es*sin(alpha); // Radius around line joining shoulder and target

   // Store gamma and beta
   gamma = asin( d_e_ts/ d_et );
   beta = asin( d_eh / d_et );

   if( std::isnan(gamma) || std::isnan(beta) ) {
     printf("GAMMA OR BETA IS NAN!!!!!!!!!!!!!!!!!!\n");
     _R_hands.resize(0);
     return;
   }
   
   Eigen::Vector3d N_tse;
   Eigen::Vector3d u_se;
   Eigen::Vector3d th_unit; 
   
   // Found rotation   
   Eigen::Vector3d u_radius_elbow_0, u_radius_elbow_i;

   if( u_st(2) != 0 ) {
     u_radius_elbow_0 << 1, 0, -(u_st(0)*1)/u_st(2);
   } else if( u_st(1) != 0 ) {
     u_radius_elbow_0 << 1, -(u_st(0)*1)/u_st(1), 0;
   } else if( u_st(0) != 0 ) {
     u_radius_elbow_0 << -(u_st(1)*1)/u_st(0), 1, 0;
   } else {
     printf("Shoulder and target are the same. Wrong!!!!!!!!!!!\n");
     return;
   }

   // Find rest of points by rotating this one up to 360
     Eigen::Matrix3d rot; Eigen::Vector3d xi, yi, zi;

   for( int i = 0; i < 36; ++i ) {
     u_radius_elbow_i = Eigen::AngleAxisd( i*10.0/180.0*3.1416, u_st )*u_radius_elbow_0;
     E = elbow_circle_center + d_e_ts*u_radius_elbow_i;
     u_se = E - S; u_se.normalize();
     N_tse = u_se.cross( u_st );    
     
     th_unit = Eigen::AngleAxisd(gamma + beta, N_tse )*(-u_st);
     th_unit.normalize();
     H = Target + d_ht*th_unit;
     zi = (H - Target); zi.normalize();
     xi = (H -E); xi.normalize();
     yi = zi.cross( xi );
     rot << xi(0), yi(0), zi(0),
       xi(1), yi(1), zi(1),
       xi(2), yi(2), zi(2);
     _R_hands.push_back( rot );
   }       
   
}
