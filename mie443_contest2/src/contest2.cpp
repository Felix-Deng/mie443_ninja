#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>

# include <cmath>


void get_target(float box_x, float box_y, float box_phi, float *target_x, float *target_y, float *target_phi) {
    float offset = 0.2; 
    *target_x = box_x + offset * std::cos(box_phi); 
    *target_y = box_y + offset * std::sin(box_phi); 
    *target_phi = box_phi - M_PI * ((box_phi >= 0.0) - (box_phi < 0.0)); 
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    int current_box = 0; 
    float target_x, target_y, target_phi; 

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        
        if (current_box == boxes.coords.size()) {
            break; // task completed 
        }

        // Set targets
        std::cout << "Going to box " << current_box << std::endl; 
        std::cout << "Box location (" << boxes.coords[current_box][0] << ", " << boxes.coords[current_box][1] << ") & phi = " << boxes.coords[current_box][2] << std::endl; 
        get_target(
            boxes.coords[current_box][0], 
            boxes.coords[current_box][1], 
            boxes.coords[current_box][2], 
            &target_x, &target_y, &target_phi
        );
        std::cout << "Target location (" << target_x << ", " << target_y << ") & phi = " << target_phi << std::endl; 
        Navigation::moveToGoal(target_x, target_y, target_phi); 
        std::cout << "Arrived at box " << current_box << std::endl; 
        current_box += 1; 

        // Image recognition @ Olivia 
        imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}