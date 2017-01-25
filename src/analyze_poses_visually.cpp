
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <tf/transform_broadcaster.h>

#include "analyze_poses.h"

const int num_joints = 3;
std::string joint_frames[num_joints] = {"shoulder", "elbow", "wrist"};
ros::Publisher mJointArrayPub;
visualization_msgs::MarkerArray mJointArrayMsgs;

const int num_links = 4;
std::string link_frames[num_links] = {"shoulder", "elbow", "wrist", "base_link"};
ros::Publisher mLinkArrayPub;
visualization_msgs::MarkerArray mLinkArrayMsgs;
double links_offset_x[num_links] = {0, 0, 0, 0.0 };
double links_offset_y[num_links] = {0, 0, 0, -0.30*0.5 };
double links_offset_z[num_links] = {-0.298*0.5, -0.331*0.5, -0.086*0.5, -0.40*0.5 };
double links_length_x[num_links] = {0.02, 0.02, 0.02, 0.10 };
double links_length_y[num_links] = {0.05, 0.05, 0.05, 0.30 };
double links_length_z[num_links] = {0.298, 0.331, 0.086, 0.40 };


std::vector< std::vector<geometry_msgs::Pose> > mJointGen;
int counter = 0;


/**
 * @function init_viz
 */
void init_viz(  ) {

  std::vector<std_msgs::ColorRGBA> colors(4);
  colors[0].r = 1.0; colors[0].g = 0.0; colors[0].b = 0.0; colors[0].a = 1.0; // Red
  colors[1].r = 0.0; colors[1].g = 1.0; colors[1].b = 0.0; colors[1].a = 1.0; // Green
  colors[2].r = 0.0; colors[2].g = 0.0; colors[2].b = 1.0; colors[2].a = 1.0; // Blue
  colors[3].r = 0.5; colors[3].g = 0.5; colors[3].b = 0.0; colors[3].a = 1.0; // Yellow

  mJointArrayMsgs.markers.resize(num_joints);
  for( int i = 0; i < num_joints; ++i ) {
    mJointArrayMsgs.markers[i].header.frame_id = joint_frames[i];
    mJointArrayMsgs.markers[i].header.stamp = ros::Time::now();
    mJointArrayMsgs.markers[i].action = visualization_msgs::Marker::ADD;
    mJointArrayMsgs.markers[i].type = visualization_msgs::Marker::SPHERE;
    mJointArrayMsgs.markers[i].scale.x = 0.05;
    mJointArrayMsgs.markers[i].scale.y = 0.05;
    mJointArrayMsgs.markers[i].scale.z = 0.05;
    
    mJointArrayMsgs.markers[i].color = colors[i];
    mJointArrayMsgs.markers[i].lifetime = ros::Duration();
    mJointArrayMsgs.markers[i].id = i;
  }

  mLinkArrayMsgs.markers.resize(num_links);
  
  for( int i = 0; i < num_links; ++i ) {
    mLinkArrayMsgs.markers[i].header.frame_id = link_frames[i];
    mLinkArrayMsgs.markers[i].header.stamp = ros::Time::now();
    mLinkArrayMsgs.markers[i].action = visualization_msgs::Marker::ADD;
    mLinkArrayMsgs.markers[i].type = visualization_msgs::Marker::CUBE;
    mLinkArrayMsgs.markers[i].scale.x = links_length_x[i];
    mLinkArrayMsgs.markers[i].scale.y = links_length_y[i];
    mLinkArrayMsgs.markers[i].scale.z = links_length_z[i];
    mLinkArrayMsgs.markers[i].pose.position.x = links_offset_x[i];
    mLinkArrayMsgs.markers[i].pose.position.y = links_offset_y[i];
    mLinkArrayMsgs.markers[i].pose.position.z = links_offset_z[i];
    mLinkArrayMsgs.markers[i].pose.orientation.x = 0;
    mLinkArrayMsgs.markers[i].pose.orientation.y = 0;
    mLinkArrayMsgs.markers[i].pose.orientation.z = 0;
    mLinkArrayMsgs.markers[i].pose.orientation.w = 1;
    
    
    mLinkArrayMsgs.markers[i].color = colors[i];
    mLinkArrayMsgs.markers[i].lifetime = ros::Duration();
    mLinkArrayMsgs.markers[i].id = num_joints + i;
  }

}

/**
 * @function init_samples
 */
void init_samples( int _N,
		   std::vector< std::vector<geometry_msgs::Pose> > &_jointGen ) {

  AnalyzePoses ap;
  ap.generate_random_poses(_N, _jointGen );  
}

/**
 *
 */
void pub_callback( const ros::TimerEvent &_ev ) {

  static tf::TransformBroadcaster br;

  // Publish transforms for shoulder, elbow and wrist
  tf::Transform Tf[num_joints]; //Ts, Te, Tw;
  tf::Quaternion q; tf::Point p;

  for( int i = 0; i < num_joints; ++i ) {
    tf::pointMsgToTF( mJointGen[i][counter].position, p );
    tf::quaternionMsgToTF( mJointGen[i][counter].orientation, q );
    Tf[i].setOrigin(p); Tf[i].setRotation(q);
    br.sendTransform( tf::StampedTransform( Tf[i], ros::Time::now(),
					   "base_link", joint_frames[i] ) );
  }		    
  
  
  std::vector<geometry_msgs::Pose> poses(num_joints);
  
  if( counter >= mJointGen[0].size() ) { counter = 0; }
  
  for( int i = 0; i < num_joints; ++i ) {
    mJointArrayMsgs.markers[i].header.stamp = ros::Time::now();
  }
		    
  for( int i = 0; i < num_links; ++i ) {
    mLinkArrayMsgs.markers[i].header.stamp = ros::Time::now();
  }

  mJointArrayPub.publish( mJointArrayMsgs );
  mLinkArrayPub.publish( mLinkArrayMsgs );
  
  counter++;
}

/**
 *
 */
int main( int argc, char* argv[] ) {

  ros::init( argc, argv, "manipul_viz" );
  ros::NodeHandle n;
  
  init_viz();
  init_samples( 100, mJointGen );
  
  mJointArrayPub = n.advertise<visualization_msgs::MarkerArray>("joints_array", 0 );
  mLinkArrayPub = n.advertise<visualization_msgs::MarkerArray>("links_array", 0 );

  ros::Timer update_timer = n.createTimer( ros::Duration(0.25), pub_callback );
  
  ros::Duration(0.1).sleep();
  ros::spin();
  
}
