#include "ADC_Navio2.h"
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include "MedianFilter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sharp_range_publish");
  ros::NodeHandle nh;

  ros::Rate rate(100); 
  ros::Publisher range_pub = nh.advertise<sensor_msgs::Range>("/mavros/distance_sensor/distance_sensor_sub", 100);
  ros::Publisher gps_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/fake_gps/mocap/pose", 100);
  
  sensor_msgs::Range range;
  float reading_in_volt;
  int unfiltered_reading = 0;
  range.radiation_type = sensor_msgs::Range::INFRARED;
  
  int count = 1;
  geometry_msgs::PoseStamped gps_pose;  
  gps_pose.header.stamp = ros::Time::now();
  gps_pose.header.seq=count;
  gps_pose.header.frame_id = 1;
  gps_pose.pose.position.x = 0.0; //xy is always 0 because we are on a test-bench
  gps_pose.pose.position.y = 0.0;
  
  gps_pose.pose.orientation.x = 0; //we send 0 for attitude but in the 
  gps_pose.pose.orientation.y = 0; //ardupilot parameters we will tell that 
  gps_pose.pose.orientation.z = 0; //don't use gps data for attitude 
  gps_pose.pose.orientation.w = 0;

  auto medFilt = new MedianFilter(9,1500);
  auto adc = new ADC_Navio2();
  adc->initialize();
  ROS_INFO("GO");

  while (ros::ok())
  {
    rate.sleep();
    range.header.stamp = ros::Time::now();
    unfiltered_reading = adc->read(4);
    unfiltered_reading = medFilt->in(unfiltered_reading);
    reading_in_volt = unfiltered_reading/1000.0;
    range.range = 604.95 * pow(reading_in_volt,-1.1904); //if 604.95 then it is in mm format
    
    gps_pose.pose.position.z = range.range; 
    range_pub.publish(range);
    gps_pose_pub.publish(gps_pose);
    ros::spinOnce();
  }

  return 0;
}



