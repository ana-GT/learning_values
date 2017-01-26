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
    robot_info[ri].base_link[l] = side + "_arm_base_link";
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

  robot_info[ri].xyz_world_min[0] << 0.05, 0.05, 0.50;
  robot_info[ri].xyz_world_max[0] << 0.55, 0.55, 1.60;

  // Right
  robot_info[ri].q_comf[1].resize( nj );
  q_def << -30, -45, 30, -90, -30, -60, 0; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[1].data = q_def;
  
  robot_info[ri].xyz_world_min[1] << 0.05, -0.55, 0.50;
  robot_info[ri].xyz_world_max[1] << 0.55, -0.05, 1.60;

  
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
    robot_info[ri].base_link[l] = "human/" + side + "_arm_mount";
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

  robot_info[ri].xyz_world_min[0] << 0.05, 0.05, 0.00;
  robot_info[ri].xyz_world_max[0] << 0.55, 0.70, 0.80;

  // Right
  robot_info[ri].q_comf[1].resize( nj );
  q_def << 45, 45,0, 45,0,0,45; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[1].data = q_def;

  robot_info[ri].xyz_world_min[1] << 0.05, -0.70, 0.00;
  robot_info[ri].xyz_world_max[1] << 0.55, -0.05, 0.80;

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
    robot_info[ri].base_link[l] = side + "_arm_mount";
    robot_info[ri].tip_link[l] = side + "_hand_ee"; // Used to be hand_ee  
    robot_info[ri].shoulder_link[l] = side + "_upper_elbow";
    
    for( int i = 0; i < nj; ++i ) {
      robot_info[ri].joint_names[l][i] = side + "_" + bnames[i];
    }
  }

  // Left
  robot_info[ri].q_comf[0].resize( nj );
  q_def << 0, 45, -90, 90, 45, 45, 0; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[0].data = q_def;

  robot_info[ri].xyz_world_min[0] << 0.15, 0.45, 0.00;
  robot_info[ri].xyz_world_max[0] << 0.70, 0.90, 0.60;

  // Right
  robot_info[ri].q_comf[1].resize( nj );
  q_def << 0, 45, 90, 90, -45 , 45, 0; q_def *= (3.1416/180.0);
  robot_info[ri].q_comf[1].data = q_def;

  robot_info[ri].xyz_world_min[1] << 0.15, -0.90, 0.00;
  robot_info[ri].xyz_world_max[1] << 0.70, -0.45, 0.60;
  
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
  double frequency = 40.0; // Hz
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
				     world_frame, ros::Time(),
				     this->Tf_base_world );
      } catch ( tf::TransformException ex ) {
	ROS_ERROR( "Error when listening transform %s", ex.what() );
      }
      try {
	listener.lookupTransform( world_frame,
				  shoulder_frame, ros::Time(),
				  this->Tf_world_shoulder );
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

      try { listener.lookupTransform( shoulder_frame,
				      base_frame, ros::Time(),
				      this->Tf_shoulder_base );
      } catch ( tf::TransformException ex ) {
	ROS_ERROR( "Error when listening transform %s", ex.what() );
      }
      
    } // end if    
    else { printf("Size is less than 4: %d \n", world_frame.size() ); }
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

  this->xyz_world_min = robot_info[_robot_type].xyz_world_min[index];
  this->xyz_world_max = robot_info[_robot_type].xyz_world_max[index];
  
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
 Eigen::Vector3d P_world_target;
 tf::Transform Tf_base_target_guess;
 printf("Lim x: %f %f \n", xyz_world_min(0), xyz_world_max(0)  );
 printf("Lim y: %f %f \n", xyz_world_min(1), xyz_world_max(1)  );
 printf("Lim z: %f %f \n", xyz_world_min(2), xyz_world_max(2)  );
 int ns = 0;
 int i, j, k;
 for( i = 0; i < (int)((xyz_world_max(0)-xyz_world_min(0))/_dv); ++i ) {
   printf("I (x) : %d \n", i );
   for( j = 0; j < (int)((xyz_world_max(1)-xyz_world_min(1))/_dv); ++j ) {
     for( k = 0; k < (int)((xyz_world_max(2)-xyz_world_min(2))/_dv); ++k ) {
       ros::spinOnce();
       // Generate suggestions
       P_world_target << xyz_world_min + Eigen::Vector3d( _dv*i, _dv*j, _dv*k );

       this->getGuess( P_world_target, Tf_base_target_guess );
       
       KDL::Frame fi;
       fi.p = KDL::Vector( Tf_base_target_guess.getOrigin().getX(),
			   Tf_base_target_guess.getOrigin().getY(),
			   Tf_base_target_guess.getOrigin().getZ() );
       tf::Quaternion dq = Tf_base_target_guess.getRotation();
       fi.M = KDL::Rotation::Quaternion( dq.getAxis().getX(), dq.getAxis().getY(),
					 dq.getAxis().getZ(), dq.getW() );
             
       // Generate a lot of random seeds 1000 to get different final poses
       rc = ik_solver->CartToJnt( q_comf, fi, qi, tol );
       if( rc < 0 ) { ns++; continue; }
       else {
	 _best.push_back(qi.data); //( qi.data );
	 _worst.push_back( qi.data );
	 geometry_msgs::Point p;
	 p.x = P_world_target(0); p.y = P_world_target(1); p.z = P_world_target(2);
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
  
  Eigen::MatrixXd Jm; Jm = J.data;
  Eigen::MatrixXd JJt = ( Jm*Jm.transpose() );
  double d = JJt.determinant();
  return sqrt(d);
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
void AnalyzePoses::getGuess( const Eigen::Vector3d &_P_world_target,
			     tf::Transform &_Tf_base_target_guess ) {

  Eigen::Vector3d z_world_def; Eigen::Vector3d z_world_st;
  Eigen::Vector3d y_world_def; Eigen::Vector3d y_world_st;
  Eigen::Vector3d x_world_def; Eigen::Vector3d x_world_st;
  Eigen::Matrix3d M_world_def; Eigen::Matrix3d M_world_st;
  
  // 1. Default approach direction (world frame)
  if( is_left ) { z_world_def << 0, -1, 0; }
  else { z_world_def << 0, 1, 0; }
  z_world_def *= this->z_palm_direction;
  // 2. Coarse approximation (shoulder-target)
  tf::Vector3 ws= this->Tf_world_shoulder.getOrigin();
  z_world_st << _P_world_target(0) - ws.getX(),
    _P_world_target(1) - ws.getY(),
    _P_world_target(2) - ws.getZ();
  z_world_st.normalize();
  z_world_st *= this->z_palm_direction; 

  // 3. Get y as perpendicular to z and in xy plane (a,b,0)
  y_world_def << -z_world_def(1), z_world_def(0), 0;
  x_world_def = y_world_def.cross( z_world_def ); 
  M_world_def.col(0) = x_world_def.transpose();
  M_world_def.col(1) = y_world_def.transpose();
  M_world_def.col(2) = z_world_def.transpose();
  
  y_world_st << -z_world_st(1), z_world_st(0), 0;
  x_world_st = y_world_st.cross( z_world_st ); 
  M_world_st.col(0) = x_world_st.transpose();
  M_world_st.col(1) = y_world_st.transpose();
  M_world_st.col(2) = z_world_st.transpose();
  
  Eigen::Quaterniond q_world_def( M_world_def );
  Eigen::Quaterniond q_world_st( M_world_st );
  
  // 4. Get vector between these two
  Eigen::Quaterniond q;
  q = q_world_st.slerp( 0.5, q_world_def );
  tf::Transform T_world_target;
  T_world_target.setIdentity();
  T_world_target.setOrigin(tf::Vector3(_P_world_target(0), _P_world_target(1), _P_world_target(2) ) );
  T_world_target.setRotation( tf::Quaternion( q.x(), q.y(), q.z(), q.w() ) );
  _Tf_base_target_guess = this->Tf_base_world * T_world_target; 
  
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
