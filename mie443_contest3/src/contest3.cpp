#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>

using namespace std;
#define N_BUMPER (3)

// Global variables
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist vel;
string path_to_sounds; 
ros::Publisher vel_pub; 

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftstate = bumper[kobuki_msgs::BumperEvent::LEFT];
uint8_t frontstate = bumper[kobuki_msgs::BumperEvent::CENTER];
uint8_t rightstate = bumper[kobuki_msgs::BumperEvent::RIGHT];

//cliff sensors
uint8_t cliff[3] = {kobuki_msgs::CliffEvent::FLOOR, kobuki_msgs::CliffEvent::FLOOR, kobuki_msgs::CliffEvent::FLOOR};

int world_state;

void followerCB(const geometry_msgs::Twist msg) {
    follow_cmd = msg;

	//check for loss of target
	if (fabs(msg.linear.x) < 0.01 && fabs(msg.angular.z) < 0.01) {
       ROS_WARN("Follower has likely lost the target.");
       world_state = 3; // sad state
   	}
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    bumper[msg->bumper] = msg->state;
}

void cliffCB(const kobuki_msgs::CliffEvent::ConstPtr& msg)
{
	cliff[msg->sensor] = msg->state;

	if (cliff[0] == kobuki_msgs::CliffEvent::CLIFF || cliff[1] == kobuki_msgs::CliffEvent::CLIFF || cliff[2] == kobuki_msgs::CliffEvent::CLIFF) {
		world_state = 2;
	}
}

// human gets too close, runs away
void angry(){
	vel.linear.x = 0.5;
	vel.angular.z = 0;
	vel_pub.publish(vel);
	sleep(1.0);
	vel.linear.x = 0;
	vel.angular.z = 0;
	vel_pub.publish(vel);
	sleep(1.0);
	vel.linear.x = 1;
	vel.angular.z = 0;
	vel_pub.publish(vel);
	sleep(1.0);
	vel.linear.x = 0;
	vel.angular.z = 0;
	vel_pub.publish(vel);
	sleep(1.0);
}

// // gets picked up, wheels spin fast while it is in the air
// void happy(){
// 	vel.linear.x = 2;
// 	vel_pub.publish(vel);
// 	vel.linear.x = 0;
// 	vel_pub.publish(vel); 
// }

// hits bumper back up and start spinning
void scared(){
	vel.angular.z = 0;
	vel.linear.x = -1;
	vel_pub.publish(vel);
	sleep(2.0);
	vel.linear.x = 0;
	vel.angular.z = 1;
	vel_pub.publish(vel);
	sleep(2.0);
	vel.angular.z = -1;
	vel_pub.publish(vel); 
	sleep(2.0);
	vel.angular.z = 0;
	vel_pub.publish(vel);
}

// loses human, starts meandering
void sad(){
	vel.linear.x = 0.0;
	for (int i = 0; i < 2; i++){
		if (i%2 == 0){
			vel.angular.z = 1;
		}
		else {
			vel.angular.z = -1;
		}
		vel_pub.publish(vel);
		sleep(2.0);
	}
	vel.angular.z = 0.0;
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
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber cliff_sub = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);

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
	bool timer_started = false;
	uint64_t last_bumper_press_time = 0;
    while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		bool any_bumper_pressed=false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        	any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }
		if (any_bumper_pressed){
			ROS_INFO("Bumper Pressed");

			if (!timer_started){
				last_bumper_press_time = secondsElapsed;
				timer_started = true;
			}
			else if ((secondsElapsed - last_bumper_press_time) <= 1){
				world_state = 4;
				timer_started = false;
			} 
			else {
				last_bumper_press_time = secondsElapsed;
				world_state = 1;
			}
		}


		bool any_cliff = false;
        for (uint32_t c_idx = 0; c_idx < 3; ++c_idx) {
			any_cliff |= (cliff[c_idx] == kobuki_msgs::CliffEvent::CLIFF);
        }

		if (any_cliff){
			world_state = 2;
		}
		
		if(world_state == 0){
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			sc.playWave(path_to_sounds+"scrm.wav");
			ROS_INFO("Scared");
			scared();
			sc.stopWave(path_to_sounds+"scrm.wav");
		} // bot gets raised, happy
		else if(world_state == 2){
			sc.playWave(path_to_sounds+"Yippee.wav");
			ROS_INFO("Cliff detected");
			ROS_INFO("Happy");
			ros::Duration(2.0).sleep();
			sc.stopWave(path_to_sounds+"Yippee.wav");
		}
		else if(world_state == 3){
			sc.playWave(path_to_sounds+"sadnesscry.wav");
			ROS_INFO("Sad");
			sad();
			sc.stopWave(path_to_sounds+"sadnesscry.wav");
		}
		else if(world_state == 4){
			sc.playWave(path_to_sounds+"chewwie.wav");
			ROS_INFO("Angry");
			ros::Duration(2.0).sleep();
			sc.stopWave(path_to_sounds+"chewwie.wav");
			angry();
		}
		world_state = 0;
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}