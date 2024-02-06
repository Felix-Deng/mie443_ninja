#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h> 

#include <stdio.h>
#include <cmath>

#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

float angular = 0.0;
float linear = 0.0;

float posX = 0.0, posY = 0.0, yaw = 0.0; 

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED}; 
float minLaserDist = std::numeric_limits<float>::infinity();
float maxLaserDist =0.0;
float maxDistAngle = 0.0;
float setYaw = 0.0, turnAngle;
int32_t nLasers=0, desiredNLasers=0, desiredAngle = 10;
int32_t maxLaserID;

bool turn;

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

    //finding the minimum laser distance
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

    //finding the max laser distance measured from the array
    for (uint32_t laser_idx=0; laser_idx < nLasers; ++laser_idx)
        {
            if(msg->ranges[laser_idx]>msg->ranges[laser_idx+1])
            {
                maxLaserDist = msg->ranges[laser_idx];
                maxLaserID = laser_idx;
            }
        }
    maxDistAngle = (nLasers/2 - maxLaserID)*(msg->angle_increment); 
    //positive angle is left, negative is right. Value is in Rad
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x; 
    posY = msg->pose.pose.position.y; 
    yaw = tf::getYaw(msg->pose.pose.orientation); 
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw)); 
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

    /*
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        // Check if any of the bumpers were pressed 
        bool any_bumper_pressed = false; 
        for (uint32_t b_idx=0; b_idx < N_BUMPER; ++b_idx){
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); 
        }

        // Control logic after bumpers are being pressed 
        ROS_INFO("Position: (%f, %f) Orientation: %f degrees Ranges: %f", posX, posY, RAD2DEG(yaw), minLaserDist); 
        if(posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed && minLaserDist > 0.7){
            angular = 0.0; 
            linear = 0.2; 
        }
        else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed && minLaserDist > 0.5){
            angular = M_PI / 6; 
            linear = 0.0; 
        }
        else if (minLaserDist > 1. && !any_bumper_pressed){
            linear = 0.1; 
            if(yaw < 17/36 * M_PI || posX > 0.6){
                angular = M_PI / 12.; 
            }
            else if (yaw < 19/36 * M_PI || posX < 0.4){
                angular = -M_PI/12.;
            }
            else {
                angular = 0;
            }
        }
        else {
            angular = 0.0; 
            linear = 0.0; 
        }
    */

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        // Check if any of the bumpers were pressed 
        bool any_bumper_pressed = false; 
        for (uint32_t b_idx=0; b_idx < N_BUMPER; ++b_idx){
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); 
        }

        // Control logic after bumpers are being pressed 
        ROS_INFO("Position: (%f, %f) Orientation: %f degrees Ranges: %f", posX, posY, RAD2DEG(yaw), minLaserDist); 
        
        /*
        // object detected (<0.5), stop and turn (ideally turn to the direction with the minLaserDist, add a delay[s])
        if (any_bumper_pressed){
            ROS_INFO("Bumper is pressed, move back linearly");
            angular = 0.0; 
            linear = -0.1;
        }
        else if (minLaserDist < 0.5) {
            angular = M_PI / 6;
            linear = 0.0;
        }
        */

//if minimum distance less than x, move forward
       if (minLaserDist > 0.5) 
       {
            linear = 0.3;
            angular = 0;
            ROS_INFO("Foward");
       }
//if minimum distance less than x and there is a lager distance available, turn
       else if (minLaserDist <= 0.5 && maxLaserDist > 0.5) 
       {
            linear = 0;
            angular = 0;
            setYaw = yaw;
            turnAngle = maxDistAngle;
            ROS_INFO("Too close, turning, turnAngle:%f", turnAngle);
            turn = true;
            while (turn)
            {
                if (turnAngle < 0)
                {
                    linear = 0;
                    angular = M_PI / 6; //turn left
                    ROS_INFO("Turning left");
                }
                else
                {
                    linear = 0;
                    angular = -(M_PI / 6); //turn right
                    ROS_INFO("Turning right");
                }

                if (abs(setYaw - yaw) >= abs(turnAngle))
                {
                    ROS_INFO("Completed turning");
                    linear = 0;
                    angular = 0;
                    turn = false;
                }
            }
       }
//if minimum distance and max distance both less than 0.5, turn 180 deg
       else if (minLaserDist <= 0.5 && maxLaserDist <= 0.5)
       {
            ROS_INFO("At the corner, turning around");
            linear = 0;
            angular = 0;
            setYaw = yaw;
            turnAngle = MI_PI;
            turn = true;
            while (turn)
            {
                ROS_INFO("Turning around");
                linear = 0;
                angular = M_PI / 6;
                if (abs(setYaw - yaw) >= abs(turnAngle))
                {
                    ROS_INFO("Completed turning");
                    turn = false;
                }
            }
       }
       else
       {
        linear = 0;
        angular = M_PI / 8;
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