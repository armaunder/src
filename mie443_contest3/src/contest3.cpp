int main(int argc, char **argv) {
    // ✅ Ensure `ros::init()` is the FIRST thing that runs
    ros::init(argc, argv, "image_listener");
    
    ros::NodeHandle nh; // ✅ Now safe to create NodeHandle

    // ✅ Initialize global variables
    path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    // Subscribers
    ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);

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