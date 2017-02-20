/**
 * @file analyze_poses_interactive2.cpp
 * @brief Generate poses for the arm with a ball in the target position
 */
#include "analyze_poses.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

sensor_msgs::JointState gMsg_joints;
ros::Publisher gPub_target;
ros::Publisher gPub_states_view;
visualization_msgs::Marker gMarker_target;

void js_cb( const sensor_msgs::JointState::ConstPtr &_msg );
void init_marker( bool _is_left );

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  ros::init( argc, argv, "analyze_interactive" );
  ros::NodeHandle nh;

  AnalyzePoses::fill_robot_info();
  bool is_left = true;
  unsigned int robot_type = HUMAN_MODEL; //BAXTER; // HUMAN_MODEL; //SCHUNK_LWA4; 
  std::string topic_in, topic_out, name;

  switch( robot_type ) {
  case BAXTER: { name=std::string("baxter"); } break;
  case HUMAN_MODEL: { name=std::string("human"); } break;
  case SCHUNK_LWA4: { name=std::string("schunk"); } break;
  }
  topic_in = "/" + name + "/joint_states";
  topic_out = "/" + name + "/analyze_joint_states";

  // Update msg
  ros::Subscriber sub = nh.subscribe(topic_in, 1, js_cb );  
  
  // Display them by publishing them in state message for human
  gPub_states_view = nh.advertise<sensor_msgs::JointState>(topic_out, 1 );

  // Init marker
  init_marker( is_left );
  
  // Generate configurations to test
  for( int i = 0; i < 1000; ++i ) {
    ros::spinOnce();
    gPub_states_view.publish( gMsg_joints );
    usleep(0.002*1e6);
  }

  AnalyzePoses ap;
  ap.setChain( is_left, robot_type );

 // Generate configurations to test
  for( int i = 0; i < 1000; ++i ) {
    ros::spinOnce();
    gPub_states_view.publish( gMsg_joints );
    usleep(0.002*1e6);
  }

  
  
  std::vector<Eigen::VectorXd> worst, best;
  std::vector<geometry_msgs::Point> target_world;
  ap.generate_best_worst_voxel( 0.10,
				10,
				best, worst,
				target_world );

  sensor_msgs::JointState js_msg;
  
  // Publish marker
  gPub_target = nh.advertise<visualization_msgs::Marker>("target_marker", 0 );
  
  // Publish
  for( int i = 0; i < best.size(); ++i ) {
  
    ros::spinOnce();    
    
    printf("Sending %d / %d \n", i, best.size() );
    std::cout << best[i].transpose() << std::endl;
    js_msg = gMsg_joints;
    js_msg.header.frame_id = "analyze_joint_states";
    for( int j = 0; j < ap.joint_names.size(); ++j ) {      
      for( int k = 0; k < js_msg.name.size(); ++k ) {
	if( ap.joint_names[j].compare( js_msg.name[k] ) == 0 ) {
	  js_msg.position[k] = best[i](j); break;
	} // if
      }// for k
    } // for j
    
    js_msg.header.stamp = ros::Time::now();   
    gPub_states_view.publish( js_msg );
    gMarker_target.header.stamp = ros::Time::now();
    gMarker_target.pose.position = target_world[i];
    gPub_target.publish( gMarker_target );
    usleep(0.25*1e6);
  }
  
  ap.stop();
  return 0;
}


/**
 * @function js_cb
 * @brief Joint state callback for human
 */
void js_cb( const sensor_msgs::JointState::ConstPtr &_msg ) {
  gMsg_joints = *_msg;
}

/**
 * @function init_marker
 */
void init_marker( bool _is_left ) {
  
  std_msgs::ColorRGBA blue;
  blue.r = 0.0; blue.g = 0.0; blue.b = 1.0; blue.a = 1.0; // Blue
  
  gMarker_target.header.frame_id = "world";

  gMarker_target.header.stamp = ros::Time::now();
  gMarker_target.action = visualization_msgs::Marker::ADD;
  gMarker_target.type = visualization_msgs::Marker::SPHERE;
  gMarker_target.scale.x = 0.075;
  gMarker_target.scale.y = 0.075;
  gMarker_target.scale.z = 0.075;
  gMarker_target.pose.position.x = 0.00;
  gMarker_target.pose.position.y = 0.00;
  gMarker_target.pose.position.z = 0.00;
  
  gMarker_target.color = blue;
  gMarker_target.lifetime = ros::Duration();
  gMarker_target.id = 23;  
}
