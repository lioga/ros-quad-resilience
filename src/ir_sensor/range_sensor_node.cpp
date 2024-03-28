//>>> TO:DO
// get adc readings from the ir sensor
// make conversion ==>  m = 0.60495 * pow(volts,-1.1904);
// publish sensor_msgs/Range msg to /mavros/distance_sensor/rangefinder_sub topic

#include "ADC_Navio2.h"
#include <sensor_msgs/Range.h>
#include <ros/ros.h>
#include <cmath>
#include <iostream>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sharp_range_publish");
  ros::NodeHandle nh;

  ros::Rate rate(1.0); //it was 20 before
  ros::Publisher range_pub = nh.advertise<sensor_msgs::Range>("/mavros/distance_sensor/rangefinder_sub", 20);
  
  sensor_msgs::Range range;
  float reading_in_volt;
  
  range.radiation_type = sensor_msgs::Range::INFRARED;

  auto adc = new ADC_Navio2();
  adc->initialize();

  while (ros::ok())
  {
    rate.sleep();
    range.header.stamp = ros::Time::now();
    reading_in_volt = (adc->read(4))/1000.0;
    range.range = 0.60495 * pow(reading_in_volt,-1.1904);
    ROS_INFO("in meters %f",range.range);
    range_pub.publish(range);
    ros::spinOnce();
  }

  return 0;
}