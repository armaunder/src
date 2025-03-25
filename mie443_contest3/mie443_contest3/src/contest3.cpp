#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftstate = bumper[kobuki_msgs::BumperEvent::LEFT];
uint8_t frontstate = bumper [kobuki_msgs::BumperEvent::CENTER];
uint8_t rightstate = bumper [kobuki_msgs::BumperEvent::RIGHT];

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 10;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
	ROS_INFO(msg);
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
	bumper[msg->bumper] = msg->state;
}

void odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    minLaserDist= std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min)/msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    ROS_INFO("Size of laser scan array: %i and size of offset %i", nLasers, desiredNLasers);

    if(desiredAngle*M_PI/180 < msg->angle_max && -desiredAngle*M_PI/180>msg->angle_min){
        for(uint32_t laser_idx = nLasers/2 - desiredNLasers; laser_idx<nLasers/2+desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
    else{
        for(uint32_t laser_idx = 0; laser_idx<nLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
}


//-------------------------------------------------------------
void scared(){
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	sc.playWave(path_to_sounds + "r2d2scream.wav");
	vel.linear.x = -0.5;
	vel.angular.z = 1;
	vel.publish(vel);
	ros::Duration(1.5).sleep();
	vel.angular.z = -1;
	vel.publish(vel);
	ros::Duration(1.5).sleep();
	vel.angular.z = 0;
	vel.linear.x = 0;
	vel.publish(vel);
}


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
	ros::Subscriber odom = nh.subscribe("odom",1,odomCB);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCB);
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
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	sc.playWave(path_to_sounds + "sound.wav");
	ros::Duration(0.5).sleep();
bool any_bumper_pressed=false;
		for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
			any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
		}
	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		bool any_bumper_pressed=false;
		for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
			any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
		}
		if (any_bumper_pressed) {
			scared();
		}

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			/*
			...
			...
			*/
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
