
#include "autonomous_helper.hpp"

using namespace std;


int main(int argc, char** argv)
{
  //init node
  ros::init(argc, argv, "autonomous_control_node");
  ros::NodeHandle nh;
   
  //init cyber attacks
  const double CYBER_ATTACK_THRESHOLD = 0.5;
   //TODO: make cyber_attack_type a ros launch  parameter 
  Cyber_Attack_Type cyber_attack_type = NO_ATTACK;
  random_device rd;
  uniform_real_distribution<double> dist(0.0,1.0);
  double random_num;
  float attack_bias = 0.001; 

  ros::Rate loop_rate(100);
  mavros_msgs::AttitudeTarget att;
  
  int count = 1;
  double theta = 0.0;
  double v[3] = {1.0, 0.0, 0.0};
  double v_norm = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  
  init_publisher_subscriber(nh);
  wait4connect();
  wait4start();
  initialize_local_frame();
  //movement of the motors start here below
  takeoff(0.2);
  ros::Duration(0.3).sleep();
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
   random_num = dist(rd); //use this random number
  
   att_visualisation.publish(att);
    
  switch(cyber_attack_type){
    case NO_ATTACK:
      att_raw_pub.publish(att);
      break;
    case DOS_ATTACK:
      if(random_num > CYBER_ATTACK_THRESHOLD)
        att_raw_pub.publish(att);
      break;
    case FDI_ATTACK:
      att.orientation.x += attack_bias;
      att.orientation.y += attack_bias;
      att.orientation.z += attack_bias;
      att.orientation.w += attack_bias;
      att_raw_pub.publish(att);
      break;
    case RAMP_ATTACK:
      att.orientation.x += attack_bias;
      att.orientation.y += attack_bias;
      att.orientation.z += attack_bias;
      att.orientation.w += attack_bias;
      attack_bias += 0.000001;
      att_raw_pub.publish(att);
      break;
    default:
      ROS_INFO("No cyber-attack mode selected, landing...");
      land();
      ros::Duration(3).sleep();
      ros::shutdown();
  }
   
   //Ctrl+C handle for drone to land
   signal(SIGINT, sigint_handler);
   
   ros::spinOnce();
   count= count + 1;
   theta=0.3*sin(count/400.0); //theta=0.3*sin(count/300.0);
  //  if(count>108000){
  //        count =0;
  //  }
   loop_rate.sleep();
  }

  return 0;
}

