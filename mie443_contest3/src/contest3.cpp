#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/BumperEvent.h>
//#include <kobuki_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sound_play/SoundRequest.h>
#include <sensor_msgs/Image.h>

using namespace std;

// Global variables
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist vel; // ✅ Ensure vel is global
string path_to_sounds; // ✅ Make sound path global
ros::Publisher vel_pub; // ✅ Make publisher global

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
// uint8_t leftstate = bumper[kobuki_msgs::BumperEvent::LEFT];
// uint8_t frontstate = bumper[kobuki_msgs::BumperEvent::CENTER];
// uint8_t rightstate = bumper[kobuki_msgs::BumperEvent::RIGHT];

int world_state;
float posX = 0.0, posY = 0.0, posZ = 0.0;

void followerCB(const geometry_msgs::Twist msg) {
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    bumper[msg->bumper] = msg->state;
	// uint8_t leftstate = bumper[kobuki_msgs::BumperEvent::LEFT];
	// uint8_t frontstate = bumper[kobuki_msgs::BumperEvent::CENTER];
	// uint8_t rightstate = bumper[kobuki_msgs::BumperEvent::RIGHT];

    if (leftstate == kobuki_msgs::BumperEvent::PRESSED || frontstate == kobuki_msgs::BumperEvent::PRESSED || rightstate == kobuki_msgs::BumperEvent::PRESSED) {
        world_state = 1;
    }
}
// // odometry detects change in position
// void odomCB(const kobuki_msgs::Odometry::ConstPtr& msg) {
// 	posX = msg->pose.pose.position.x;
// 	posY = msg->pose.pose.position.y;
// 	posZ = msg->pose.pose.position.z;
// 	if(posZ > 0.5){
// 		world_state = 3;
// 	} // if returned coordinates are less than -1, then the robot is too close to the human
// 	else if (posX < -1 || posY < -1){
// 		world_state = 2;
// 	} // if returned coordinates are greater than 1, then the robot is too far from the human
// 	else if(posX > 1 || posY > 1){
// 		world_state = 5;
// 	}

// }

// human gets too close, runs away
void scared(){
	vel.linear.x = -2;
	vel.angular.z = 1;
	vel_pub.publish(vel);

}

// gets picked up, wheels spin fast while it is in the air
void happy(){
	vel.linear.x = 2;
	vel_pub.publish(vel);
	vel.linear.x = 0;
	vel_pub.publish(vel); 
}

// finds human, sound of shock
void surprised(){
	// sc.playWave(path_to_sounds+"r2scream.wav"); //change sound
	// sleep(2.0); 
}

// hits bumper back up and start spinning
void anger(){
	// sc.playWave(path_to_sounds+"r2scream.wav"); //change sound
	//sleep(2.0);
	vel.linear.x = -2;
	vel_pub.publish(vel);
	vel.linear.x = 0;
	vel.angular.z = 1;
	vel_pub.publish(vel);
	ros::Duration(2.0).sleep();
	vel.angular.z = 0;
	vel_pub.publish(vel); 
}

// loses human, starts meandering
void sad(){
	vel.linear.x = 0.5;
	vel.angular.z = 0.5;
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

	//ros::Subscriber odom = nh.subscribe("odom", 1, &odomCB);

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

    while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		if(world_state == 0){
			vel_pub.publish(follow_cmd);

		}// bumper hit, anger
		else if(world_state == 1){
			sc.playWave(path_to_sounds+"r2scream.wav");
			anger();
			ROS_INFO("Bumper hit");
			ROS_INFO("Anger");
			sc.stopWave(path_to_sounds+"r2scream.wav");
		} // human gets too close, scared
		else if(world_state == 2){
			scared();
		} // bot picked up, happy
		else if(world_state == 3){
			sc.playWave(path_to_sounds+"r2scream.wav");
			happy();
			ros::Duration(2.0).sleep();
			sc.stopWave(path_to_sounds+"r2scream.wav");
		} // finds human, surprised
		else if(world_state == 4){
			surprised();
		} // loses human, sad
		else if(world_state == 5){
			sad();
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}