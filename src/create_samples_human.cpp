
#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <stdio.h> 
#include <fstream>
#include <Eigen/Geometry>

#include <learning_values/base_ann.h>

void regularize( const std::vector<Eigen::VectorXd> &_states,
		 const std::vector<double> &_metrics,
		 Eigen::MatrixXd &_x,
		 Eigen::MatrixXd &_t ) {

  int xdim = _states[0].size();
  int N = _states.size();
  _x.resize( xdim, N );
  _t.resize( 1, N );

  for( int i = 0; i < N; ++i ) {
    _x.col(i) = _states[i].transpose();
    _t(0,i) = _metrics[i];
  }

  // 1. Substract mean
  Eigen::VectorXd mean(xdim);
  for( int i = 0; i < xdim; ++i ) {
    mean(i) = _x.row(i).sum()/(double)N;
  }

  for( int j = 0; j < N; ++j ) {
    for( int i = 0; i < xdim; ++i ) {
      _x(i,j) -=  mean(i);
    }
  }

  // 2. Std
  Eigen::VectorXd stdv(xdim);
  for( int i = 0; i < xdim; ++i ) {
    stdv(i) = sqrt(_x.row(i).squaredNorm()/(double)N);
  }

  for( int j = 0; j < N; ++j ) {
    for( int i = 0; i < xdim; ++i ) {
      _x(i,j) /=  stdv(i);
    }
  }
  
}


/**
 * @function metric_1
 */
double metric_1( KDL::JntArray _js ) {

  Eigen::VectorXd comfortable(7);
  comfortable << 0, 36, 0, 68, -90, 0, -25;
  comfortable *= (3.1416/180.0);

  Eigen::VectorXd c(7);
  c << 4,2,1,1,1,1,1;

  Eigen::VectorXd dq(7);
  
  for( int i = 0; i < 7; ++i ) {
    dq(i) = fabs( _js(i) - c(i) );
  }
  
  return sqrt( c.dot(dq) );
}

double metric_2( KDL::JntArray _js,
		 KDL::Chain _chain ) {

  KDL::Jacobian J;
  KDL::ChainJntToJacSolver jnt_to_jac_solver( _chain );
  J.resize( _chain.getNrOfJoints() );
  jnt_to_jac_solver.JntToJac( _js, J );
  
  std::cout << "Rows: " << J.rows() << " columns: "<< J.columns() << std::endl;
  Eigen::MatrixXd Jm; Jm = J.data;
  Eigen::MatrixXd JJt = ( Jm*Jm.transpose() );
  double d = JJt.determinant();
  return sqrt(d);
}

double random_val(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

/**
 *
 */
int main( int argc, char* argv[] ) {

  srand( time(NULL) );
  
  ros::init( argc, argv, "test" );
  ros::NodeHandle nh;
  
  std::string base_link("base_link");
  std::string tip_link("hand_link");	
  std::string urdf_param("/robot_description");
  double timeout_in_secs = 0.005;
  double error=1e-5; 
  TRAC_IK::SolveType type=TRAC_IK::Speed;
  
  TRAC_IK::TRAC_IK ik_solver( base_link,
			      tip_link,
			      urdf_param,
			      timeout_in_secs,
			      error,
			      type );  

 KDL::Chain chain;
 KDL::JntArray low_lim, up_lim;
 bool valid = ik_solver.getKDLChain( chain );
 if( !valid ) {
   printf("There was not valid KDL chain found \n");
   return 0;
 }

 valid = ik_solver.getKDLLimits( low_lim, up_lim );
 if( !valid ) {
   printf("There were not valid limits found! \n");
   return 0;
 }

 printf("Num chain joints: %d. Limits size: %d \n",
	chain.getNrOfJoints(), low_lim.data.size() );

 // Forward kinematic solver
 int num_samples = 10000;
 KDL::ChainFkSolverPos_recursive fk_solver( chain ); 
 std::vector<KDL::JntArray> jointList;
 KDL::JntArray q( chain.getNrOfJoints() );
 KDL::Frame end_effector_pose;
 int rc;
 KDL::JntArray result;
 KDL::JntArray nominal( chain.getNrOfJoints() );
 double m;
 for( unsigned int i = 0; i < nominal.data.size(); ++i ) {
   nominal(i) = (low_lim(i) + up_lim(i))/2.0;
 }
 
 for( unsigned int i = 0; i < num_samples; ++i ) {
   for( unsigned int j = 0; j < low_lim.data.size(); ++j ) {
     q(j) = random_val( low_lim(j), up_lim(j) );
   }
   jointList.push_back( q );
 }

 std::vector<Eigen::VectorXd> states;
 std::vector<double> metrics;
 
 for( unsigned int i = 0; i < num_samples; ++i ) {

   fk_solver.JntToCart( jointList[i], end_effector_pose );
   //rc = ik_solver.CartToJnt( nominal, end_effector_pose, result );
   //if( rc < 0 ) { printf("[%d] Did not solve IK! \n", i ); continue; }
   
   // Get metric
   m = metric_1( jointList[i] );
   m = metric_2( jointList[i], chain );
   // Store
   Eigen::Vector3d p;
   p << end_effector_pose.p.x(),
     end_effector_pose.p.y(),
     end_effector_pose.p.z(); 

   states.push_back( p );
   metrics.push_back( m );
   
 }

 
 base_ann red( 3, 10, 1 );
 Eigen::MatrixXd x, t, y;
 regularize( states, metrics, x, t );
 if( !red.train( x, t, y ) ) { printf("Error training! \n"); return 1; }
 
 /*
 int rc = ik_solver.CartToJnt( KDL::JntArray joint_seed,
			       KDL::Frame desired_end_effector_pose,
			       KDL::JntArray& return_joints,
			       KDL::Twist tolerances );
 */			       

 // Store for visualization
 std::ofstream output("workspace.txt", std::ofstream::out );
 double metric_min = 10; double metric_max = 0;
 for( int i = 0; i < states.size(); ++i ) {
   output << states[i].transpose() << " " << metrics[i] << " " << t(0,i) << std::endl;
   if( metrics[i] < metric_min ) { metric_min = metrics[i]; }
   if( metrics[i] > metric_max ) { metric_max = metrics[i]; }
 }
 output.close();
 std::cout << "Min metric: "<< metric_min << " metric max: "<< metric_max << std::endl;

 
 return 0;
}

