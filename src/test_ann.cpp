
#include <learning_values/base_ann.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

/****/
double rand_val( double _min_val,
		 double _max_val ) {
  return _min_val + (_max_val - _min_val)*(rand()/(double)RAND_MAX);
}

/**
 * @function gen_train_data
 */
void gen_train_data( Eigen::MatrixXd &_x,
		     Eigen::MatrixXd &_t,
		     int _N ) {

  Eigen::VectorXd xi, ti;
  int dim = 2;
  xi.resize(dim); ti.resize(1);
  
  _x.resize( dim, _N );
  _t.resize( 1, _N );
  
  for( int i = 0; i < _N; ++i ) {
    xi << (double)i*0.02, rand_val(-2,2);
    ti << xi(0)*xi(0)*3 - xi(0)*2 + 5*xi(1)*xi(1)*xi(1);
    /*
    xi << rand_val(-2, 2), rand_val(-2,2), rand_val(-2,2);
    ti << xi(0)*xi(0)*xi(0) + 2*xi(1)*xi(1) - 3*xi(2);
    */
    _x.col(i) = xi;
    _t.col(i) = ti;
  }

  std::cout << "Bef x\n: "<< _x << std::endl;
  
  // Get the mean
  Eigen::VectorXd mean = Eigen::VectorXd::Zero(dim);
  for( int j = 0; j < dim; ++j ) {
    for( int i = 0; i < _N; ++i ) {
      mean(j) += _x(j,i); 
    }
    mean(j) = mean(j)/(double)_N;
  }

  for(int i = 0; i < _N; ++i ) {
    for( int j = 0; j < dim; ++j ) {
      _x(j,i) -= mean(j);
    }
  }

  // Scale
  Eigen::VectorXd stdv = Eigen::VectorXd::Zero(dim);
  for( int j = 0; j < dim; ++j ) {
    for( int i = 0; i < _N; ++i ) {
      stdv(j) += _x(j,i)*_x(j,i); 
    }
    stdv(j) = sqrt(stdv(j)/(double)_N);
  }
  
  for(int i = 0; i < _N; ++i ) {
    for( int j = 0; j < dim; ++j ) {
      _x(j,i) /= stdv(j);
    }
  }
  
  std::cout << "After x\n: "<< _x << std::endl;
  return;
}

int main( int argc, char* argv[] ) {

  srand( time(NULL) );

  int dim = 2;
  
  base_ann test( dim, 20, 1 );

  // Generate training data
  Eigen::MatrixXd x, t, y;
  gen_train_data( x, t, 1000 );

  
  if( test.train( x, t, y ) ) {
    printf("NN learned! \n");
  }

  std::ofstream output( "test.txt", std::ofstream::out );
  for( int i = 0; i < t.cols(); ++i ) {
    output << x(0,i) << " " << t(0,i) << " " << y(0,i) << std::endl;
  }
  output.close();
  
  return 0;
}
