
#include <learning_values/legible_motion_2d.h>
#include <ros/ros.h>

LegibleMotion2D::LegibleMotion2D( ros::NodeHandle &_nh ) : nh_(_nh) {
  legible_path_pub_ = nh_.advertise<nav_msgs::Path>("legible_path", 10 );
}


void LegibleMotion2D::generateLegibleTrajectory( int _index,
						 nav_msgs::Path &_path ) {

  // Get default
  int d = 100;

  visualization_msgs::InteractiveMarker p;
  geometry_msgs::Pose start, goal;
  server_->get( start_point_, p );
  start = p.pose;
  server_->get( goals_[_index], p );
  goal = p.pose;

  _path.header.frame_id = "world";
  _path.poses.resize(0);

  double dx, dy, dz;
  dx = (goal.position.x - start.position.x);
  dy = (goal.position.y - start.position.y);
  dz = (goal.position.z - start.position.z);
  
  for( int i = 0; i < d; ++i ) {
    geometry_msgs::PoseStamped pi;
    pi.pose.position.x = start.position.x + (dx/(double)d)*i;
    pi.pose.position.y = start.position.y + (dy/(double)d)*i;
    pi.pose.position.z = start.position.z + (dz/(double)d)*i;
    pi.pose.orientation.w = 1;
    _path.poses.push_back( pi );
  }
  
  legible_path_pub_.publish( _path );
}

/**
 *
 */
visualization_msgs::Marker makeBox( visualization_msgs::InteractiveMarker &msg,
				    double _r, double _g, double _b ) {

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = _r;
  marker.color.g = _g;
  marker.color.b = _b;
  marker.color.a = 1.0;

  return marker;
}

/**
 *
 */
void LegibleMotion2D::make_planar_marker( geometry_msgs::Point _pos,
					  std::string _name,
					  bool _set_menu, 
					  double _r, double _g, double _b ) {
  
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  int_marker.pose.position = _pos;
  int_marker.scale = 1;

  int_marker.name = _name;
  int_marker.description = _name;

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1; control.orientation.x = 0;
  control.orientation.y = 1; control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker, _r, _g, _b) );
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  //server_->setCallback( int_marker.name, boost::bind(&LegibleMotion2D::processFeedback, this, _1) );

  if( _set_menu ) { menu_handler_.apply( *server_, int_marker.name ); }

  // Apply changes
  server_->applyChanges();
}

/**
 *
 */
void LegibleMotion2D::init() {
  server_.reset( new interactive_markers::InteractiveMarkerServer("legible", "", false ) );
  menu_handler_.insert( "Plan Legible", boost::bind(&LegibleMotion2D::processFeedback, this, _1) );

  geometry_msgs::Point p1; p1.x = 0.5; p1.y = 0.8; p1.z = 0;
  std::string g1 = "G1";
  make_planar_marker( p1, g1, true, 1.0, 0.0, 0.0 );
  add_goal(g1);
  
  geometry_msgs::Point p2; p2.x = 0.5; p2.y = -0.8; p2.z = 0;
  std::string g2 = "G2";
  make_planar_marker( p2, g2, true, 0.0, 1.0, 0.0 );
  add_goal(g2);

  geometry_msgs::Point s; s.x = 0.0; s.y = 0.0; s.z = 0;
  std::string start = "Start";
  make_planar_marker( s, start, false, 0.0, 0.0, 1.0 );
  set_start(start);
  
}

/**
 * @function add_goal
 */
void LegibleMotion2D::add_goal( std::string _name ) {
  std::vector<std::string>::iterator it;
  it = std::find( goals_.begin(), goals_.end(), _name);
  if( it == goals_.end() ) {
    goals_.push_back(_name);
  }
}

void LegibleMotion2D::set_start( std::string _name ) {
  start_point_ = _name;
}


/**
 *
 */
void LegibleMotion2D::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &_feedback )
{
  ROS_WARN("Planning trajectory from start %s to goal %s",
	   start_point_.c_str(),
	   _feedback->marker_name.c_str() );
  geometry_msgs::Pose pm = _feedback->pose;
  ROS_WARN("Transform in frame %s: %f %f %f -- %f %f %f %f",
	   _feedback->header.frame_id.c_str(),
	   pm.position.x, pm.position.y, pm.position.z,
	   pm.orientation.x, pm.orientation.y, pm.orientation.z, pm.orientation.w );

  visualization_msgs::InteractiveMarker start;
  server_->get( start_point_, start );
  geometry_msgs::Pose ps = start.pose;
  ROS_WARN("Transform for start in frame %s: %f %f %f -- %f %f %f %f",
	   start.header.frame_id.c_str(),
	   ps.position.x, ps.position.y, ps.position.z,
	   ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w );

  int index = -1;
  for( int i = 0; i < goals_.size(); ++i ) {
    if( _feedback->marker_name == goals_[i] ) { index = i; break; }
  }

  if( index == -1 ) { ROS_ERROR("Is marker a goal? WTH!! EXITING"); }
  nav_msgs::Path path;
  generateLegibleTrajectory( index, path );
}
/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  ros::init( argc, argv, "legible_motion_2d" );
  ros::NodeHandle nh;

  LegibleMotion2D lm( nh );
  lm.init();
  
  while( ros::ok() ) {
    ros::spinOnce();
    lm.update();
    usleep( 0.1*1e6 );
  }
  
}
