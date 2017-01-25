
#include <Eigen/Core>


class base_ann {

 public:
  base_ann( int _input_dim,
	    int _hidden_dim,
	    int _output_dim);

  bool forward_pass( const Eigen::MatrixXd &_x,
		     Eigen::MatrixXd &_y,
		     Eigen::MatrixXd &_z );
  
  double backprop_step( Eigen::MatrixXd &_dE_w1,
			Eigen::MatrixXd &_dE_w2,
			const Eigen::MatrixXd &x,
			const Eigen::MatrixXd &t,
			int N );

  double get_error( Eigen::MatrixXd _x,
		    Eigen::MatrixXd _t );
  
  bool train( const Eigen::MatrixXd &_x,
	      const Eigen::MatrixXd &_t,
	      Eigen::MatrixXd &_y );
  void random_fill_w();
  double rand_val( double _min_val,
		   double _max_val );
 private:
  Eigen::MatrixXd w1;
  Eigen::MatrixXd w2;

  int input_dim;
  int hidden_dim;
  int output_dim;
  
};
