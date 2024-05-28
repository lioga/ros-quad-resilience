#include "autonomous_helper.hpp"

using namespace std;


int main(int argc, char** argv)
{
  //init node
  ros::init(argc, argv, "autonomous_control_node");
  ros::NodeHandle nh;
   
  //init cyber attacks
  const double CYBER_ATTACK_THRESHOLD = 0.7;
   //TODO: make cyber_attack_type a ros launch  parameter 
  Cyber_Attack_Type cyber_attack_type = NO_ATTACK;
  random_device rd;
  uniform_real_distribution<double> dist(0.0,1.0);
  uniform_real_distribution<double> bias_dist(-2,2); //[-4,4] for fdi, [-2,2] for ramp 

  double random_num;
  float attack_bias;

  ros::Rate loop_rate(50);
  mavros_msgs::AttitudeTarget att;
  
  int count = 1;
  double theta = 0.0;
  double v[3] = {1.0, 0.0, 0.0};
  double v_norm = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  double roll, pitch, yaw;

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
   att.orientation.x=sin(theta/2.0)*v[0]/v_norm;//0; don't forget to make inital tuning  
   att.orientation.y=sin(theta/2.0)*v[1]/v_norm;//0; by just hovering
   att.orientation.z=sin(theta/2.0)*v[2]/v_norm;//0;
   att.orientation.w=cos(theta/2.0);//1; 
   att.type_mask = 7; 
   att.thrust=0.55; 
  
   tf::Quaternion q(att.orientation.x, att.orientation.y, att.orientation.z, att.orientation.w);
   tf::Matrix3x3 m(q);
   
   att_visualisation.publish(att);
   
   switch(cyber_attack_type){
    case NO_ATTACK:
      att_raw_pub.publish(att);
      break;
    case DOS_ATTACK:
      random_num = dist(rd); 
      if(random_num > CYBER_ATTACK_THRESHOLD)
        att_raw_pub.publish(att);
      else{
        for (int i = 0; i < 10; ++i){
          loop_rate.sleep();
          count= count + 1;
       } 
      }
      break;
    case FDI_ATTACK:
      attack_bias = bias_dist(rd)*M_PI/180;

      m.getRPY(roll, pitch, yaw);
      roll = roll + attack_bias;
      pitch = pitch + attack_bias;
      yaw = yaw + attack_bias;
      
      q.setRPY(roll,pitch,yaw);
      q = q.normalize();
      
      att.orientation.x = q.getX();
      att.orientation.y = q.getY(); 
      att.orientation.z = q.getZ();
      att.orientation.w = q.getW();
      
      att_raw_pub.publish(att);
      break;
    case RAMP_ATTACK:
      attack_bias = bias_dist(rd)*M_PI/180;
      attack_bias = attack_bias + count*M_PI/180*0.003*((attack_bias>0)-(attack_bias<0));
      
      m.getRPY(roll, pitch, yaw);
      roll = roll + attack_bias;
      pitch = pitch + attack_bias;
      yaw = yaw + attack_bias;
      
      q.setRPY(roll,pitch,yaw);
      q = q.normalize();
      
      att.orientation.x = q.getX();
      att.orientation.y = q.getY(); 
      att.orientation.z = q.getZ();
      att.orientation.w = q.getW();
      
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
   theta=0.2*sin(count/400.0); 

   loop_rate.sleep();
  }
  land();

  return 0;
}