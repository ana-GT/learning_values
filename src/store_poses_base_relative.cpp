#include <iostream>
#include "analyze_poses.h"
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState gMsg_joints;
ros::Publisher gPub_states_view;

/**
 * @function js_cb
 * @brief Joint state callback for human
 */
void js_cb( const sensor_msgs::JointState::ConstPtr &_msg ) {
  gMsg_joints = *_msg;
}

int main( int argc, char* argv[] ) {

  ros::init( argc, argv, "generating" );
  ros::NodeHandle nh;

  AnalyzePoses::fill_robot_info();
  
  bool is_left = false;
  int robot_type = BAXTER; //BAXTER; // HUMAN_MODEL; //SCHUNK_LWA4; 
  std::string topic_in, topic_out, name; std::string text_file;
  std::string side;

  if( is_left ) { side = "left"; } else { side = "right"; }

  switch( robot_type ) {
  case BAXTER: { name=std::string("baxter"); } break;
  case HUMAN_MODEL: { name=std::string("human"); } break;
  case SCHUNK_LWA4: { name=std::string("schunk"); } break;
  }
  topic_in = "/" + name + "/joint_states";
  topic_out = "/" + name + "/analyze_joint_states";
  text_file = name + "_config_" + side + ".txt";

  // Update msg
  ros::Subscriber sub = nh.subscribe(topic_in, 1, js_cb );  
  
  // Display them by publishing them in state message for human
  gPub_states_view = nh.advertise<sensor_msgs::JointState>(topic_out, 1 );

  // Ap 
  AnalyzePoses ap;
  int n = 10;
  double dv = 0.05;
  if( !ap.setChain( is_left, robot_type ) ) {
    printf("Model was not set. EXIT! \n" );
    return 0;
  }  
  
  // Generate configurations to test
  for( int i = 0; i < 1000; ++i ) {
    ros::spinOnce();
    gPub_states_view.publish( gMsg_joints );
    usleep(0.002*1e6);
  }


  ap.generate_and_store( text_file, dv, n );
  ap.stop();
}
