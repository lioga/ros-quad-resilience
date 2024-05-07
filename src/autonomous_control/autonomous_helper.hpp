#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h> 
#include <time.h>
#include <random>
#include <signal.h>

enum Cyber_Attack_Type{
    NO_ATTACK,
    DOS_ATTACK,
    FDI_ATTACK,
    RAMP_ATTACK,
};

mavros_msgs::State current_state_g;
nav_msgs::Odometry current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g; 

ros::Publisher att_raw_pub;
ros::Publisher att_visualisation;
ros::Publisher local_pos_pub;
ros::Publisher global_lla_pos_pub;
ros::Publisher global_lla_pos_pub_raw;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;


void state_cb(const mavros_msgs::State::ConstPtr& msg);

geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu);

void pose_cb(const nav_msgs::Odometry::ConstPtr& msg);

geometry_msgs::Point get_current_location();

void set_heading(float heading);
void set_destination(float x, float y, float z, float psi);
void set_att_destination(float or_x, float or_y, float or_z, float or_w, float z);
void set_destination_lla(float lat, float lon, float alt, float heading);
void set_destination_lla_raw(float lat, float lon, float alt, float heading);

int wait4connect();
int wait4start();
int initialize_local_frame();

int arm();
int takeoff(float takeoff_alt);
int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01);
int check_waypoint_att_reached(float pos_tolerance=0.3);
int set_mode(std::string mode);
int land();

int set_yaw(float angle, float speed, float dir, float absolute_rel);
int takeoff_global(float lat, float lon, float alt);

int init_publisher_subscriber(ros::NodeHandle controlnode);

void sigint_handler(int sig);