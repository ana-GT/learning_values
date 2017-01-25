/**
 * @file base_ann.cpp
 */

#include <learning_values/base_ann.h>
#include <math.h>
#include <stdio.h>
#include <iostream>

/**
 * @function base_ann
 * @brief Constructor
 */
base_ann::base_ann( int _input_dim,
		    int _hidden_dim,
		    int _output_dim ) {

  output_dim = _output_dim;
  hidden_dim = _hidden_dim;
  input_dim = _input_dim;
  
  w1.resize( hidden_dim, input_dim + 1 );
  w2.resize( output_dim, hidden_dim + 1 );
  
}

/**
 * @function rand_val
 */
double base_ann::rand_val( double _min_val,
			   double _max_val ) {
  return _min_val + (_max_val - _min_val)*(rand()/(double)RAND_MAX);
}

/**
 * @function random_fill_w
 */
void base_ann::random_fill_w() {

  // w1
  for( int i = 0; i < w1.cols(); ++i ) {
    for( int j = 0; j < w1.rows(); ++j ) {
      w1(j,i) = rand_val( -1.0, 1.0 );
    }
  }

  // w2
  for( int i = 0; i < w2.cols(); ++i ) {
    for( int j = 0; j < w2.rows(); ++j ) {
      w2(j,i) = rand_val( -1.0, 1.0 );
    }
  }

}

/**
 * @function forward_pass
 */
bool base_ann::forward_pass( const Eigen::MatrixXd &_x,
			     Eigen::MatrixXd &_y,
			     Eigen::MatrixXd &_z ) {

  int N = _x.cols();
  if( _x.rows() != input_dim + 1 ) {
    printf("Pass x augmented! EXIT\n");
    return false;
  }
  
  // 2.1. Pass forward
  _z.resize( hidden_dim + 1, N );
  _y.resize( output_dim, N );
  
  Eigen::MatrixXd a( hidden_dim, N );
  
  a = w1*_x;
  
  for( int i = 0; i < a.cols(); ++i ) { // N 
    for( int j = 0; j < a.rows(); ++j ) {
      _z(j,i) = tanh( a(j,i) );
    }
  }  
  _z.row( hidden_dim ) = Eigen::VectorXd::Ones(N).transpose();
  _y = w2*_z;
  
  return true;
}




/****/
double base_ann::backprop_step( Eigen::MatrixXd &_dE_w1,
				Eigen::MatrixXd &_dE_w2,
				const Eigen::MatrixXd &x,
				const Eigen::MatrixXd &_t,
				int N ) {
    
  // 2.2. Calculate the errors
  Eigen::MatrixXd y, z;
  Eigen::MatrixXd dhz( hidden_dim, N );

  forward_pass( x, y, z );

  for( int i = 0; i < dhz.cols(); ++i ) {
    for( int j = 0; j < dhz.rows(); ++j ) {
      dhz(j,i) = (1 - z(j,i)*z(j,i) ); 
    }
  }
  
  
  // d_k
  Eigen::MatrixXd d_k( output_dim, N );
  Eigen::MatrixXd d_j( hidden_dim, N );
  
  d_k = (y - _t);
  
  for( int j = 0; j < hidden_dim; ++j ) {
    d_j.row(j) = w2.col(j).transpose() * d_k;
  }
  
  d_j = dhz.cwiseProduct( d_j );
  
  
  // 2.3. Get the derivatives
  //_dE_w1( hidden_dim, input_dim + 1 );
  //_dE_w2( output_dim, hidden_dim + 1 );
  
  _dE_w1 = d_j * x.transpose();
  _dE_w2 = d_k*z.transpose();
    
  _dE_w1 /= (double)N;
  _dE_w2 /= (double)N;

  return d_k.norm();
}

/**
 *
 */
double base_ann::get_error( Eigen::MatrixXd _x,
			    Eigen::MatrixXd _t ) {

  // 2.1. Pass forward
  Eigen::MatrixXd y, z;
  forward_pass( _x, y, z );
    
  // 2.2. Calculate the errors

  // d_k
  Eigen::MatrixXd de;
  de = y - _t;
  Eigen::VectorXd err( _t.cols() );
  for( int i = 0; i < err.size(); ++i ) {
    err(i) = de.col(i).norm();
  }

  for( int i = 0; i < err.size(); ++i ) {
    std::cout << "["<<i<<"] y: " << y(0,i) << ", t: "<< _t(0,i) << " err: "<< err(i) << std::endl;
  }
  //  std::cout << "Err: " << err.transpose() << std::endl;
  // Error
  return err.norm();
  
}

/**
 * @Let's start with an online version of update
 **/
bool base_ann::train( const Eigen::MatrixXd &_x,
		      const Eigen::MatrixXd &_t,
		      Eigen::MatrixXd &_y ) {

  // 1. Initialize w
  random_fill_w(); 
  
  // 2. Back-propagation
  
  // 2.0. Add tip at the end of the input (1)
  int N = _x.cols();
  Eigen::MatrixXd x( input_dim + 1, N );
  x.block(0,0, input_dim, N ) = _x;
  x.row( input_dim ) = Eigen::VectorXd::Ones(N).transpose();

  Eigen::MatrixXd dE_w1, dE_w2;
  double nu = 0.002;

  double err = get_error( x, _t );
  std::cout << "Init error: "<< err/(double)N << std::endl;
  
  for( int i = 0; i < 2000; ++i ) {
    if( i % 100 == 0 ) { printf("Iter %d - Err (no div): %f \n", i, err); }
    err = backprop_step( dE_w1, dE_w2, x, _t, N );
    w1 = w1 - nu*dE_w1;
    w2 = w2 - nu*dE_w2;   
  }

  Eigen::MatrixXd z;
  forward_pass( x, _y, z );
  
  // Get error
  std::cout << "Error: "<< get_error( x, _t )/(double)N << std::endl;
    
  return true;
}


  /*
  for( int i = 0; i < input_dim + 1; ++i ) {
    for( int j = 0; j < hidden_dim; ++j ) {
      for( int t = 0; t < N; ++t ) {
	_dE_w1(j,i) += d_j(j,t) * x(i,t);
      }
    }
  } */
  /*
    for( int j = 0; j < hidden_dim + 1; ++j ) {
    for( int k = 0; k < output_dim; ++k ) {
    for( int t = 0; t < N; ++t ) {
    _dE_w2(k,j) += d_k(k,t) * z(j,t);
    }
    }
    }*/
