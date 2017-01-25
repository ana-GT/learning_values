/**
 * @file ChainFkSolverPos_updated.h
 */

#include <kdl/chainfksolverpos_recursive.hpp>


namespace KDL {

  class ChainFkSolverPos_updated : public ChainFkSolverPos_recursive {

  public:    
    ChainFkSolverPos_updated( const Chain&_chain );
    ~ChainFkSolverPos_updated();

    int JntToCart( const JntArray& q_in,
		   std::vector<Frame>& p_out,
		   int segmentNr=-1 );
    //private:
    const Chain chain;
  };
}
