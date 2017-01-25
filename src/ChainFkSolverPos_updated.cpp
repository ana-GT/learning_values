/**
 * @file ChainFkSolverPos_updated.cpp
 */

#include "ChainFkSolverPos_updated.h"

namespace KDL {

  /**
   * @function ChainFkSolverPos_updated
   */
  ChainFkSolverPos_updated::ChainFkSolverPos_updated(const Chain& _chain):
    ChainFkSolverPos_recursive(_chain),
    chain(_chain)
  {

  }

  /**
   * @function JntToCart
   * @brief Forward Kinematics with 
   */
  int ChainFkSolverPos_updated::JntToCart(const JntArray& q_in,
					  std::vector<Frame>& p_out,
					  int seg_nr )
  {
    unsigned int segmentNr;
    if(seg_nr<0) {
      segmentNr=chain.getNrOfSegments();
    } else {
      segmentNr = seg_nr;
    }
    
    if(q_in.rows()!=chain.getNrOfJoints()) {
      return -1;
    } else if(segmentNr>chain.getNrOfSegments()) {
      return -1;
    } else if(p_out.size() != segmentNr) {
      p_out.resize(segmentNr);
      return -1;
    }
    else if(segmentNr == 0) {
      return -1;
    } else {
      int j=0;
      // Initialization
      if(chain.getSegment(0).getJoint().getType()!=Joint::None){
	p_out[0] = chain.getSegment(0).pose(q_in(j));
	j++;
      }else
	p_out[0] = chain.getSegment(0).pose(0.0);

      for(unsigned int i=1;i<segmentNr;i++){
	if(chain.getSegment(i).getJoint().getType()!=Joint::None){
	  p_out[i] = p_out[i-1]*chain.getSegment(i).pose(q_in(j));
	  j++;
	}else{
	  p_out[i] = p_out[i-1]*chain.getSegment(i).pose(0.0);
	}
      }
      return 0;
    }
  }
  

  /** 
   * @function ~ChainFkSolverPos_updated
   * @brief Destructor
   */
  ChainFkSolverPos_updated::~ChainFkSolverPos_updated()
  {
  }
   
  
} // namespace KDL
