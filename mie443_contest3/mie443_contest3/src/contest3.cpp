#include <header.h>
#include <ros/package.h>
#include <kobuki_msgs/BumperEvent.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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
	bool emotion1 = false, emotion2 = false, emotion3 = false; // keep track of if all three emotions have been actuated before ending 

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
		
		bool two_side_pressed = false; 
		if (bumper[0] == kobuki_msgs::BumperEvent::PRESSED && bumper[2] == kobuki_msgs::BumperEvent::PRESSED){
			two_side_pressed = true; 
		}

		if(two_side_pressed){
			// Poked by others --> rage 
			world_state = 3; 
			emotion3 = true; 
		}
		else if(any_bumper_pressed){
			// Hitting obstacles --> fear 
			world_state = 1; 
			emotion1 = true; 
		}
		else if(is_stopped(follow_cmd) && secondsElapsed >= 5){
			if (secondsElapsed >= 240 && emotion1 && emotion2 && emotion3){
				// Task completed --> pride 
				world_state = 4; 
			}
			else {
				// Lost track of target --> anger 
				world_state = 2; 
				emotion2 = true; 
			}
		} 
		else {
			world_state = 0; 
		}
		 
		if(world_state == 0){ 
			// Normal following mode 
			// vel_pub.publish(vel);
			vel_pub.publish(follow_cmd); 
		}else if(world_state == 1){
			// Case 1: when the robot cannot continue to track the person due to a static obstacle in its path 			
			// Display image and play sound to show emotion 
			sc.playWave(path_to_sounds + "fear.wav");
			Mat img = imread(path_to_images + "fear.jpg", IMREAD_COLOR);
			resize(img, img, Size(img.cols / 2, img.rows / 2)); 
			namedWindow("Display window", WINDOW_AUTOSIZE); 
			imshow("Display window", img); 
			waitKey(2000); 

			// Robot moves back as a sign of fear 
			std::chrono::time_point<std::chrono::steady_clock> case1_start = std::chrono::steady_clock::now();
			while(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-case1_start).count() / 1000 <= 500){ 
				vel.linear.x = -0.1;
				vel_pub.publish(vel); 
			}
			vel.linear.x = 0.0; 
			vel_pub.publish(vel); 
			for (float i = 0.0; i < 4.0; i++){
				while(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-case1_start).count() / 1000 <= 600 + i * 100){ 
					if (int(i) % 2 == 0){
						vel.angular.z = 2.0;
					}
					else {
						vel.angular.z = -2.0; 
					}
					vel_pub.publish(vel); 
				}
			}
			vel.angular.z = 0.0; 
			vel_pub.publish(vel);
			while(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-case1_start).count() / 1000 <= 1500){ 
				vel.linear.x = -0.2;
				vel_pub.publish(vel); 
			}
			vel.linear.x = 0.0; 
			vel.angular.z = 0.0; 
			vel_pub.publish(vel); 

			destroyAllWindows(); 
			sc.stopWave(path_to_sounds + "fear.wav"); 
		}else if (world_state == 2){
			// Case 2: when the robot loses track of the person it is following 
			// Display image and play sound to show emotion 
			sc.playWave(path_to_sounds + "angry.wav");
			Mat img = imread(path_to_images + "angry.jpg", IMREAD_COLOR);
			resize(img, img, Size(img.cols / 2, img.rows / 2)); 
			namedWindow("Display window", WINDOW_AUTOSIZE); 
			imshow("Display window", img); 
			waitKey(2000);
			
			// Robot shakes left and right to show anger 
			std::chrono::time_point<std::chrono::system_clock> case2_start = std::chrono::system_clock::now();
			for (float i = 0.0; i < 4.0; i++){
				while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-case2_start).count() <= 1 + i){ 
					if (int(i) % 2 == 0){
						vel.angular.z = 2.0;
					}
					else {
						vel.angular.z = -2.0;
					}
					vel_pub.publish(vel); 
				}	
			}
			vel.angular.z = 0.0; 
			vel_pub.publish(vel); 
			std::this_thread::sleep_for(std::chrono::seconds(2)); 

			destroyAllWindows(); 
			sc.stopWave(path_to_sounds + "angry.wav");
		}else if (world_state == 3){
			// Case 3: when two bumpers of the robot are being pressed at the same time as a sign of intimitation 
			// Display image and play sound to show emotion 
			sc.playWave(path_to_sounds + "rage.wav");
			Mat img = imread(path_to_images + "rage.jpg", IMREAD_COLOR);
			resize(img, img, Size(img.cols / 2, img.rows / 2)); 
			namedWindow("Display window", WINDOW_AUTOSIZE); 
			imshow("Display window", img); 
			waitKey(2000); 
			std::this_thread::sleep_for(std::chrono::seconds(1)); 

			// Robot tries to hit the person while being rage
			std::chrono::time_point<std::chrono::system_clock> case3_start = std::chrono::system_clock::now();
			while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-case3_start).count() <= 1){ 
				vel.linear.x = 2.5;
				vel_pub.publish(vel); 
			}
			vel.linear.x = 0.0; 
			vel_pub.publish(vel); 

			destroyAllWindows(); 
			sc.stopWave(path_to_sounds + "rage.wav");  
		}else if (world_state == 4){
			// Case 4: when the robot stops following after 4 minutes as the task is completed
			// Display image and play sound to show emotion  
			sc.playWave(path_to_sounds + "pride.wav");
			Mat img = imread(path_to_images + "pride.jpg", IMREAD_COLOR);
			resize(img, img, Size(img.cols / 2, img.rows / 2)); 
			namedWindow("Display window", WINDOW_AUTOSIZE); 
			imshow("Display window", img); 
			waitKey(2000); 

			// Robot turns in a circle as a sign of pride 
			std::chrono::time_point<std::chrono::system_clock> case4_start = std::chrono::system_clock::now();
			while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-case4_start).count() <= 1){ 
				vel.linear.x = 0.2;
				vel.angular.z = 1.0; 
				vel_pub.publish(vel); 
			}
			vel.linear.x = 0.0; 
			vel.angular.z = 0.0; 
			vel_pub.publish(vel); 

			destroyAllWindows(); 
			sc.stopWave(path_to_sounds + "pride.wav");  
			return 0; 
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
