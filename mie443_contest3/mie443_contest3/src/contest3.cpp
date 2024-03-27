#include <header.h>
#include <ros/package.h>
#include <kobuki_msgs/BumperEvent.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv; 

geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED}; 

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
    //Fill with code
	bumper[msg->bumper] = msg->state;
}

bool is_stopped(geometry_msgs::Twist speed){
	float linear_acc = speed.linear.x + speed.linear.y + speed.linear.z;
	float angular_acc = speed.angular.x + speed.angular.y + speed.angular.z; 
	return linear_acc == 0 && angular_acc == 0; 
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	string path_to_images = ros::package::getPath("mie443_contest3") + "/images/"; 
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

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

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		
		bool any_bumper_pressed = false; 
        for (int b_idx=0; b_idx < 3; ++b_idx){
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); 
        }
		
		if(any_bumper_pressed){
			world_state = 2; 
		}
		else if(is_stopped(follow_cmd)){
			world_state = 1; 
		}
		 
		if(world_state == 0){ 
			// Normal following mode 
			// vel_pub.publish(vel);
			vel_pub.publish(follow_cmd); 

			// Show positively excited emotion 
			Mat img = imread(path_to_images + "excited.png", IMREAD_COLOR); 
			imshow("Display window", img); 
			ros::Duration(0.5).sleep(); 
		}else if(world_state == 1){
			// Case 1: when the robot loses track of the person it is following 
			// Play ... sound 
			sc.playWave(path_to_sounds + "sound.wav"); 
			ros::Duration(0.5).sleep();
			sc.stopWave(path_to_sounds + "sound.wav"); 
		}else if (world_state == 2){
			// Case 2: when the robot cannot continue to track the person due to a static obstacle in its path 
			sc.playWave(path_to_sounds + "sound.wav");
			ros::Duration(0.5).sleep();
			sc.stopWave(path_to_sounds + "sound.wav"); 
		}else if (world_state == 3){
			// When ... 
			sc.playWave(path_to_sounds + "sound.wav");
			ros::Duration(0.5).sleep();
			sc.stopWave(path_to_sounds + "sound.wav"); 
		}else if (world_state == 4){
			// When ... 
			sc.playWave(path_to_sounds + "sound.wav");
			ros::Duration(0.5).sleep();
			sc.stopWave(path_to_sounds + "sound.wav"); 
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
