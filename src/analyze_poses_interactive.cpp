/**
 * @file analyze_poses_interactive.cpp
 */
#include "analyze_poses.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

sensor_msgs::JointState gMsg;
std::string gJoint_names[7] = {"right_shoulder_0", "right_shoulder_1", "right_shoulder_2",
			       "right_elbow_0",
			       "right_wrist_0","right_wrist_1", "right_wrist_2" };

void js_cb( const sensor_msgs::JointState::ConstPtr &_msg ) {
  gMsg = *_msg;
}

ros::Publisher mBallPub;
visualization_msgs::Marker mToBall;

/****/
void init_marker() {
  std_msgs::ColorRGBA blue;
  blue.r = 0.0; blue.g = 0.0; blue.b = 1.0; blue.a = 1.0; // Blue

    mToBall.header.frame_id = "human/right_hand";
    mToBall.header.stamp = ros::Time::now();
    mToBall.action = visualization_msgs::Marker::ADD;
    mToBall.type = visualization_msgs::Marker::SPHERE;
    mToBall.scale.x = 0.10;
    mToBall.scale.y = 0.10;
    mToBall.scale.z = 0.10;
    mToBall.pose.position.z = -0.10;
    
    mToBall.color = blue;
    mToBall.lifetime = ros::Duration();
    mToBall.id = 23;

}

/****/
int main( int argc, char* argv[] ) {

  ros::init( argc, argv, "analyze_interactive" );
  ros::NodeHandle nh;

  // Init marker
  init_marker();
  
  // Update msg
  ros::Subscriber sub = nh.subscribe("/human/joint_states", 1, js_cb );
  
  // Generate configurations to test
  AnalyzePoses ap;
  ap.setChain( false );
  std::vector<Eigen::VectorXd> worst, best;
  std::vector<geometry_msgs::Point> target_world;
  printf("Generating sets \n");
  ap.generate_best_worst_voxel( 0.10,
				best, worst,
				target_world );
  
  printf("FInished generating \n");
  // Display them by publishing them in state message for human
  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/human/analyze_joint_states", 1 );
  sensor_msgs::JointState random_config_msg;

  // Publish marker
  mBallPub = nh.advertise<visualization_msgs::Marker>("ballToCatch", 0 );

  
  usleep(1.0*1e6);
  printf("Started... size good: %d \n", best.size() );

  
  for( int i = 0; i < best.size(); ++i ) {
  
    ros::spinOnce();
    
    // Update
    random_config_msg = gMsg;
    random_config_msg.header.frame_id = "analyze_joint_states";
    for( int j = 0; j < 7; ++j ) {
      
      for( int k = 0; k < gMsg.name.size(); ++k ) {
	if( gJoint_names[j].compare( gMsg.name[k] ) == 0 ) {
	  random_config_msg.position[k] = best[i](j); break;
	} // if
	}// for
    }
    
    random_config_msg.header.stamp = ros::Time::now();
    
    std::cout <<"J["<<i<<"]: "<< best[i].transpose() << std::endl;
    pub.publish( random_config_msg );
    mToBall.header.stamp = ros::Time::now();
    mBallPub.publish( mToBall );
    usleep(0.5*1e6);
  }

  return 0;
}
