#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h> 

#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <random>

#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

float angular = 0.0;
float linear = 0.0; 
float target_yaw;  // target position

float posX = 0.0, posY = 0.0, yaw = 0.0; // actual location recording 
float max_x = 0.0, min_x = 0.0, max_y = 0.0, min_y = 0.0; // search history tracking 

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED}; 
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle = 20;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	bumper[msg->bumper] = msg-> state; 
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	minLaserDist = std::numeric_limits<float>::infinity(); 
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle * M_PI / (180 * msg->angle_increment);
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    if(desiredAngle * M_PI/180 < msg->angle_max && -desiredAngle * M_PI/180 > msg->angle_min){
        for(uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]); 
        }
    }
    else {
        for(uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]); 
        }
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x; 
    posY = msg->pose.pose.position.y; 
    yaw = tf::getYaw(msg->pose.pose.orientation); 
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw)); 
}

void set_vel(bool bumper_pressed, float min_laser_dist, float *lin_vel, float *ang_vel) {
    if (bumper_pressed || min_laser_dist >= 1e6){
        // Bumpers are pressed or about to hit obstacles 
        desiredAngle = 10; 
        *ang_vel = M_PI / 6.; 
        *lin_vel = -0.1; 
        target_yaw = yaw; 
    }
    else if (min_laser_dist < 0.7) {
        // About to hit obstacles and turn 
        desiredAngle = 10; 
        *ang_vel = M_PI / 6.;
        *lin_vel = 0.0;
        target_yaw = yaw; 
    }
    // else if (min_laser_dist < 1.){
    //     // Getting close to obstacles and decelerate 
    //     *ang_vel = 0.0; 
    //     *lin_vel = 0.25; 
    // }
    else {
        // Nothing in front and move forward at full speed 
        desiredAngle = 20; 
        *lin_vel = 0.25;
        *ang_vel = 0.0;
    }
}

void update_pos_history(){
    if (posX > max_x) {
        max_x = posX; 
    }
    else if (posX < min_x) {
        min_x = posX; 
    }
    if (posY > max_y) {
        max_y = posY; 
    }
    else if (posY < min_y) {
        min_y = posY; 
    }
}

float choose_dir(){
    float closed_dist = 0.7; 
    bool closed_to_up = max_y - posY <= closed_dist || posY > max_y; 
    bool closed_to_bottom = posY - min_y <= closed_dist || posY < min_y; 
    bool closed_to_left = posX - min_x <= closed_dist || posX < min_x; 
    bool closed_to_right = max_x - posX <= closed_dist || posX > max_x; 
    if (closed_to_up){
        if (closed_to_left){
            return -3. / 4. * M_PI; 
        }
        else if (closed_to_right){
            return 3. / 4. * M_PI; 
        }
        else{
            float yaw_change = (float)(std::rand() % 90); 
            if (yaw_change < 45) {
                return 3. / 4. * M_PI + DEG2RAD(yaw_change); 
            }
            else {
                return -5. / 4. * M_PI + DEG2RAD(yaw_change); 
            }
        }
    }
    else if (closed_to_bottom) {
        if (closed_to_left){
            return -M_PI / 4.; 
        }
        else if (closed_to_right){
            return M_PI / 4.; 
        }
        else{
            return M_PI / 4. - DEG2RAD((float)(std::rand() % 90)); 
        }
    }
    else {
        return DEG2RAD((float)(std::rand() % 360)); 
    }
}

void set_dir(float target_yaw, float curr_yaw, float *ang_vel){
    target_yaw = RAD2DEG(target_yaw) + 180.; 
    curr_yaw = RAD2DEG(curr_yaw) + 180.; 
    float diff_yaw = target_yaw - curr_yaw; 
    if (abs(diff_yaw) <= 20.) {
        *ang_vel = M_PI / 12. * (float)((diff_yaw > 0) - (diff_yaw < 0)); 
    }
    else {
        *ang_vel = M_PI / 6. * (float)((diff_yaw > 0) - (diff_yaw < 0)); 
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback); 

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        // Check if any of the bumpers were pressed 
        bool any_bumper_pressed = false; 
        for (uint32_t b_idx=0; b_idx < N_BUMPER; ++b_idx){
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); 
        }

        ROS_INFO("Position: (%f, %f) Orientation: %f degrees Ranges: %f", posX, posY, RAD2DEG(yaw), minLaserDist); 
        
        if (secondsElapsed <= 240) {
            // Stage 1: exterior wall following 
            set_vel(any_bumper_pressed, minLaserDist, &linear, &angular); 
            // Update history tracking 
            update_pos_history(); 
            // Refresh target_yaw for stage 2 
            target_yaw = yaw; 
        }
        else {
            // Stage 2: random walk within the pre-explored boundary 
            if (abs(yaw - target_yaw) <= 0.2) {
                int prob = std::rand() % 10; // random probability between 0 and 9 
                if (prob <= 1) {
                    // Change direction of exploration 
                    angular = 0.; 
                    linear = 0.; 
                    target_yaw = choose_dir(); 
                }
                else {
                    // Keep exploring in current direction 
                    set_vel(any_bumper_pressed, minLaserDist, &linear, &angular); 
                }
                target_yaw = yaw; // refresh for numerical accuracy 
            }
            else {
                // Rotate to target yaw 
                set_dir(target_yaw, yaw, &angular);
                linear = 0.; 
            }
            update_pos_history(); 
        }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}