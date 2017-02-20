
#include "interaction.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  printf("Initing node...\n");
  ros::init( argc, argv, "interaction" );
  ros::NodeHandle nh;

  printf("Fill robot info...\n");
  AnalyzePoses::fill_robot_info();
  printf("Finished filling robot info...\n");
  Interaction inter(&nh);
  // Load human
  printf("Set human \n");
  if( !inter.setHuman( false ) ) {
    printf("Error setting human model! \n");
    return 0;
  }
  // Load robot
  printf("Set robot \n");
  if( !inter.setRobot( BAXTER, true ) ) {
    printf("Error setting robot model! \n");
    return 0;
  }
  //Publish a bit
  printf("Pub a bit \n");
  inter.pub_a_bit();
  
  // Show all common points
  std::vector<Eigen::VectorXd> human_confs;
  std::vector<Eigen::VectorXd> robot_confs;
  std::vector<geometry_msgs::Point> P_world;
  double dv = 0.10;

  printf("Get common points \n");
  if( !inter.getCommonPoints(human_confs, robot_confs, P_world, dv ) ) {
    printf("There are not common points! \n");
    return 0;
  }

  // View
  printf("View interaction \n");
  double dt = 0.5;
  inter.view_interaction( human_confs,
			  robot_confs,
			  P_world, dt );

  inter.stop();
  
  return 0;
}
