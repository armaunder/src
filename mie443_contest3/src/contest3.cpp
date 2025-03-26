#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

// if bumper hit
void bumperCB(const geometry_msgs::Twist msg){
    bumper[msg->bumper] = msg->state;
	if(bumper[0] == 1 || bumper[1] == 1 || bumper[2] == 1){
		world_state = 1;
	}
}

// // if human gets too close
// void humanCB(const geometry_msgs::Twist msg){
// 	if(msg->linear.x < 0.5){
// 		world_state = 2;
// 	}
// }

// // if robot gets picked up
// void pickedUpCB(const geometry_msgs::Twist msg){
// 	if(msg->linear.z > 0.5){
// 		world_state = 3;
// 	}
// }

// // if human is found
// void foundCB(const geometry_msgs::Twist msg){
// 	if(msg->linear.x > 0.5){
// 		world_state = 4;
// 	}
// }

// // if human is lost
// void lostCB(const geometry_msgs::Twist msg){
// 	if(msg->linear.x > 0.5){
// 		world_state = 5;
// 	}
// }



// human gets too close, runs away
void scared(){
	// sc.playWave(path_to_sounds+"r2scream.wav");
	// sleep(2.0);
	// sc.stopWave(path_to_sounds+"r2scream.wav");
	vel.linear.x = -2;
	vel.angular.z = 1;
	vel_pub.publish(vel);

}

// gets picked up, wheels spin fast while it is in the air
void happy(){
	//sc.playWave(path_to_sounds+"r2scream.wav"); //change sound
	// sleep(2.0);
	vel.linear.x = 2;
	vel_pub.publish(vel);
	sleep(2.0);
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
	sleep(2.0);
	vel.angular.x = 0;
	vel_pub.publish(vel); 
}

// loses human, starts meandering
void sad(){
	vel.linear.x = 0.5;
	vel.angular.z = 0.5;
	vel_pub.publish(vel);
	ros::Duration(1.0).sleep();
}


//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.25;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			anger();
		}
		else if(world_state == 2){
			scared();
		}
		else if(world_state == 3){
			happy();
		}
		else if(world_state == 4){
			surprised();
		}
		else if(world_state == 5){
			sad();
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
		}
	}

	return 0;
}
