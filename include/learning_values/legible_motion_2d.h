/**
 * @function LegibleMotion2D
 * @brief Offer two Interactive Markers for goals and 1 for start pose and allows (through a callback) to generate a legible trajectory
 */
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

class LegibleMotion2D {

public:

LegibleMotion2D( ros::NodeHandle &_nh );

 void add_goal( std::string _name );
 void set_start( std::string _name );
void generateLegibleTrajectory( int _index,
				  nav_msgs::Path &_path );
void init();
 void make_planar_marker( geometry_msgs::Point _pos,
			  std::string _name,
			  bool _set_menu = true, 
			  double _r=0.5,
			  double _g = 0.5,
			  double _b = 0.5 );

 void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &_feedback );
 void update() { server_->applyChanges(); }
 
 protected:
 ros::NodeHandle nh_;
 std::vector<std::string> goals_;
 std::string start_point_;
 geometry_msgs::Pose p0;

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
interactive_markers::MenuHandler menu_handler_;
 ros::Publisher legible_path_pub_;

};
