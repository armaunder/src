#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>

using namespace std;
#define N_BUMPER (3)
#define N_CLIFF (3)

// Global variables
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist vel; // ✅ Ensure vel is global
string path_to_sounds; // ✅ Make sound path global
ros::Publisher vel_pub; // ✅ Make publisher global

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t cliff[N_CLIFF] = {kobuki_msgs::CliffEvent::FLOOR, kobuki_msgs::CliffEvent::FLOOR, kobuki_msgs::CliffEvent::FLOOR};

int world_state;

void followerCB(const geometry_msgs::Twist msg) {
    follow_cmd = msg;

    // Check for loss of target
    if (fabs(msg.linear.x) < 0.01 && fabs(msg.angular.z) < 0.01) {
        ROS_WARN("Follower has likely lost the target.");
        world_state = 5; // sad state
    }
}

void cliffCallback(const kobuki_msgs::CliffEvent::ConstPtr& msg)
{
    cliff[msg->sensor] = msg->state;
    if (msg->state == kobuki_msgs::CliffEvent::CLIFF) {
        ROS_WARN("Cliff detected! Sensor: %d", msg->sensor);
        world_state = 2; // scared state
    }
}

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
   bumper[msg->bumper] = msg->state;

   if (leftstate == kobuki_msgs::BumperEvent::PRESSED || frontstate == kobuki_msgs::BumperEvent::PRESSED || rightstate == kobuki_msgs::BumperEvent::PRESSED) {
       world_state = 1;
   }
}

void anger(){
   sc.playWave(path_to_sounds + "chewwie.wav");
   vel.linear.x = -2;
   vel.angular.z = 1;
   vel_pub.publish(vel);
   ros::Duration(1).sleep();
   sc.stopWave(path_to_sounds + "chewwie.wav");
}

void happy(){
   vel.linear.x = 0;
   vel.angular.z = 1;
   vel_pub.publish(vel);
   ros::Duration(0.5).sleep();
   
   vel.angular.z = -1;
   vel_pub.publish(vel);
   ros::Duration(0.5).sleep();
   
   vel.angular.z = 0;
   vel.linear.x = 1;
   vel_pub.publish(vel);
   ros::Duration(0.5).sleep();
   
   vel.linear.x = 0;
   vel_pub.publish(vel);
   ros::Duration(0.5).sleep();

   vel.linear.x = 1;
   vel_pub.publish(vel);
   ros::Duration(0.5).sleep();

   vel.linear.x = 0;
   vel_pub.publish(vel);
}

void surprised(){
}

void scared(){
   sc.playWave(path_to_sounds + "r2d2_scared.wav");
   vel.linear.x = -2;
   vel_pub.publish(vel);
   vel.linear.x = 0;
   vel.angular.z = 1;
   vel_pub.publish(vel);
   sleep(2.0);
   vel.angular.z = 0;
   vel_pub.publish(vel);
}

void sad(){
   sc.playWave(path_to_sounds + "sadnesscry.wav");
   vel.linear.x = -0.5;
   vel.angular.z = 0;
   vel_pub.publish(vel);
   ros::Duration(0.5).sleep();
   vel.angular.z = 1;
   vel_pub.publish(vel);
   ros::Duration(2).sleep();
   sc.stopWave(path_to_sounds + "sadnesscry.wav");
   vel.angular.z = 0;
   vel.linear.x = 0;
   vel_pub.publish(vel);
}

//-------------------------------------------------------------
int main(int argc, char **argv) {
   ros::init(argc, argv, "image_listener");
   ros::NodeHandle nh;

   // ✅ Initialize global variables
   path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
   sound_play::SoundClient sc; // Initialize sound client after NodeHandle
   vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

   // Subscribers
   ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
   ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
   ros::Subscriber cliff_sub = nh.subscribe("mobile_base/events/cliff", 10, &cliffCallback);

   // Contest count down timer
   ros::Rate loop_rate(10);
   std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
   uint64_t secondsElapsed = 0;

   // Image Transport
   imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8);
   imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

   world_state = 0;
   double angular = 0.2;
   double linear = 0.25;

   vel.angular.z = angular;
   vel.linear.x = linear;

   sc.playWave(path_to_sounds + "sound.wav");
   ros::Duration(0.5).sleep();
   ros::Rate loop_rate(10);
   while(ros::ok() && secondsElapsed <= 480){     
       ros::spinOnce();
      
       bool any_bumper_pressed=false;
       for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
           any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
       }

       if (any_bumper_pressed){
           world_state = 1;
       }
       if(world_state == 0){
           vel_pub.publish(follow_cmd);
       } else if(world_state == 1){
           ROS_INFO("Anger"); 
		   anger(); // You just kicked me
       }
       else if(world_state == 2){
		   ROS_INFO("Scared");
           scared(); // I have been picked up
       }
       else if(world_state == 3){
           happy();
       }
       else if(world_state == 4){
           surprised();
       }
       else if(world_state == 5){
		   ROS_INFO("Sad");
           sad(); //lost the person
       }
       secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
       loop_rate.sleep();
   }

   return 0;
}