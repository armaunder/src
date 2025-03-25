#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;

// Global variables
geometry_msgs::Twist follow_cmd;
geometry_msgs::Twist vel; // ✅ Ensure vel is global
sound_play::SoundClient sc; // ✅ Make sc global
string path_to_sounds; // ✅ Make sound path global
ros::Publisher vel_pub; // ✅ Make publisher global

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftstate = bumper[kobuki_msgs::BumperEvent::LEFT];
uint8_t frontstate = bumper[kobuki_msgs::BumperEvent::CENTER];
uint8_t rightstate = bumper[kobuki_msgs::BumperEvent::RIGHT];

int world_state;

void followerCB(const geometry_msgs::Twist msg) {
    follow_cmd = msg;
}

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
    bumper[msg->bumper] = msg->state;
}

//-------------------------------------------------------------
// ✅ Fixed `scared()` function to use global `vel` and `sc`
void scared() {
    sc.playWave(path_to_sounds + "r2scream.wav");
    sleep(2.0);
    sc.stopWave(path_to_sounds + "r2scream.wav");

    vel.linear.x = -2;
    vel.angular.z = 1;
    vel_pub.publish(vel);
}

// ✅ Happy function (unchanged)
void happy() {
    sc.playWave(path_to_sounds + "r2scream.wav"); // Change sound
    sleep(2.0);
    vel.linear.x = 2;
    vel_pub.publish(vel);
    sleep(2.0);
    vel.linear.x = 0;
    vel_pub.publish(vel);
}

//-------------------------------------------------------------
int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    // ✅ Initialize global variables
    path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    // Subscribers
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);

    // Contest count down timer
    ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Image Transport
    imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8);
    imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

    int world_state = 0;
    double angular = 0.2;
    double linear = 0.25;

    vel.angular.z = angular;
    vel.linear.x = linear;

    sc.playWave(path_to_sounds + "sound.wav");
    ros::Duration(0.5).sleep();

    while (ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        bool any_bumper_pressed = false;

        // ✅ Ensure the correct use of `BumperEvent::PRESSED`
        for (uint32_t b_idx = 0; b_idx < 3; ++b_idx) { 
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }

        if (!any_bumper_pressed) { // ✅ Fixed incorrect `else if`
            scared();
        }

        if (world_state == 0) {
            vel_pub.publish(follow_cmd);
        } else if (world_state == 1) {
            /*
            ...
            */
        }

        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - start
        ).count();
        loop_rate.sleep();
    }

    return 0;
}