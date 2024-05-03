
#include "autonomous_helper.hpp"

using namespace std;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "autonomous_control_node");
  ros::NodeHandle nh;
  geometry_msgs::PoseStamped cmd_att;
  ros::Rate loop_rate(100);
  int count = 1;
  double theta=0.0;
  double v[3]={1.0, 0.0, 0.0};
  double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  
  mavros_msgs::AttitudeTarget att;

  init_publisher_subscriber(nh);
  wait4connect();
  wait4start();
  initialize_local_frame();
  takeoff(0.2);
  
  while (ros::ok())
  {
   
   //Create attitude command message
   att.header.stamp = ros::Time::now();
   att.header.seq=count;
   att.header.frame_id = 1;
   att.orientation.x=sin(theta/2.0)*v[0]/v_norm;
   att.orientation.y=sin(theta/2.0)*v[1]/v_norm;
   att.orientation.z=sin(theta/2.0)*v[2]/v_norm;
   att.orientation.w=cos(theta/2.0);
   att.type_mask = 7;
   att.thrust=0.55;
   //TODO: write an enum that spesifies the attack type then give it to a switch 
   //statement to choose from, you can also make that variable a ros launch  parameter  
   att_raw_pub.publish(att);

   ros::spinOnce();
   count= count +2;
   theta=0.3*sin(count/300.0);
   loop_rate.sleep();

  }



  //land();




  return 0;
}

