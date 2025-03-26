#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;

// Global variables
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist vel; // ✅ Ensure vel is global
string path_to_sounds; // ✅ Make sound path global
ros::Publisher vel_pub; // ✅ Make publisher global
sound_play::SoundClient sc; // ✅ Make sc global

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftstate = bumper[kobuki_msgs::BumperEvent::LEFT];
uint8_t frontstate = bumper[kobuki_msgs::BumperEvent::CENTER];
uint8_t rightstate = bumper[kobuki_msgs::BumperEvent::RIGHT];

int world_state;

void followerCB(const geometry_msgs::Twist msg) {
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    bumper[msg->bumper] = msg->state;
    if (bumper[0] == 1 || bumper[1] == 1 || bumper[2] == 1) {
        world_state = 1;
    }
}

//-------------------------------------------------------------
// ✅ Fixed `scared()` function to use global `vel` and `sc`
void scared() {
    vel.linear.x = -2;
    vel.angular.z = 1;
    vel_pub.publish(vel);
}

// gets picked up, wheels spin fast while it is in the air
void happy(){
    vel.linear.x = 2;
    vel_pub.publish(vel);
    sleep(2.0);
    vel.linear.x = 0;
    vel_pub.publish(vel); 
}

// finds human, sound of shock
void surprised(){
}

// hits bumper back up and start spinning
void anger(){
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
}

//-------------------------------------------------------------
int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    ros::WallDuration(1.0).sleep(); // Added delay for ROS initialization

    // ✅ Initialize global variables
    path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    sc = sound_play::SoundClient(nh);

    // Subscribers
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

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
        } else if(world_state == 1){
            sc.playWave(path_to_sounds + "r2scream.wav");
            scared();
            ros::Duration(2.0).sleep();
            sc.stopWave(path_to_sounds + "r2scream.wav");
        } else if(world_state == 2){
            anger();
        } else if(world_state == 3){
            happy();
        } else if(world_state == 4){
            surprised();
        } else if(world_state == 5){
            sad();
        }
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}