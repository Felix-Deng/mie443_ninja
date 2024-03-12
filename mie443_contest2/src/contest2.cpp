#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <nav_msgs/GetPlan.h>
#include <chrono>

#include <cmath>


float angle_rotation_adj(float posx1, float posy1, float posx2, float posy2, float offset){
    return acos(1 - (pow(posx1 - posx2, 2) + pow(posy1 - posy2, 2)) / (2 * pow(offset, 2)));
}

void get_target(float box_x, float box_y, float box_phi, float *target_x, float *target_y, float *target_phi, ros::NodeHandle n) {
    float offset = 0.2; 
    *target_x = box_x + offset * std::cos(box_phi); 
    *target_y = box_y + offset * std::sin(box_phi); 
    if (box_phi >= 0.0){
        *target_phi = box_phi - M_PI; 
    }
    else {
        *target_phi = box_phi + M_PI; 
    }

    
    // Check if the target path is feasible 
    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    std::cout << check_path << std::endl;
    nav_msgs::GetPlan srv;
    // srv.request.start = start;
    // srv.request.goal = goal;
    check_path.call(srv);
    
    if (srv.response.plan.poses.size() == 0){
        // Path unavailable; a new path is needed 
        float min_x = -2.0, min_y = -2.20, max_x = 2.0, max_y = 2.20; 
        float temp_x = *target_x, temp_y = *target_y; 
        if (*target_x > max_x){
            *target_x = max_x; 
            if (box_phi >= 0.0) {
                *target_y = box_y + pow(pow(offset, 2) - pow(max_x - box_x, 2), 0.5); 
                *target_phi = *target_phi + angle_rotation_adj(*target_x, *target_y, temp_x, temp_y, offset); 
            }
            else {
                *target_y = box_y - pow(pow(offset, 2) - pow(max_x - box_x, 2), 0.5); 
                *target_phi = *target_phi - angle_rotation_adj(*target_x, *target_y, temp_x, temp_y, offset); 
            }
        }
        else if (*target_x < min_x) { 
            *target_x = min_x; 
            if (box_phi >= 0.0) {
                *target_y = box_y - pow(pow(offset, 2) - pow(box_x - min_x, 2), 0.5); 
                *target_phi = *target_phi - angle_rotation_adj(*target_x, *target_y, temp_x, temp_y, offset); 
            }
            else {
                *target_y = box_y + pow(pow(offset, 2) - pow(box_x - min_x, 2), 0.5); 
                *target_phi = *target_phi + angle_rotation_adj(*target_x, *target_y, temp_x, temp_y, offset); 
            }
        }
        else if (*target_y < min_y) { 
            *target_y = min_y; 
            if (box_phi >= -M_PI / 2.0) {
                *target_x = box_x + pow(pow(offset, 2) - pow(min_y - box_y, 2), 0.5); 
                *target_phi = *target_phi + angle_rotation_adj(*target_x, *target_y, temp_x, temp_y, offset); 
            }
            else {
                *target_x = box_x - pow(pow(offset, 2) - pow(min_y - box_y, 2), 0.5); 
                *target_phi = *target_phi - angle_rotation_adj(*target_x, *target_y, temp_x, temp_y, offset); 
            }
        }
        else if (*target_y > max_y) {
            *target_y = max_y; 
            if (box_phi <= M_PI / 2.0) {
                *target_x = box_x + pow(pow(offset, 2) - pow(max_y - box_y, 2), 0.5); 
                *target_phi = *target_phi - angle_rotation_adj(*target_x, *target_y, temp_x, temp_y, offset); 
            }
            else {
                *target_x = box_x - pow(pow(offset, 2) - pow(max_y - box_y, 2), 0.5); 
                *target_phi = *target_phi + angle_rotation_adj(*target_x, *target_y, temp_x, temp_y, offset); 
            }
        }
        
    }
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
        
        /*
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
            &target_x, &target_y, &target_phi, n
        );
        std::cout << "Target location (" << target_x << ", " << target_y << ") & phi = " << target_phi << std::endl; 
        Navigation::moveToGoal(target_x, target_y, target_phi); 
        std::cout << "Arrived at box " << current_box << std::endl; 
        current_box += 1; 
        */

        // Image recognition @ Olivia 
        int result = imagePipeline.getTemplateID(boxes); //returns the most matched template ID
        std::cout << "Picture ID:" << result << std::endl;
        ros::Duration(0.01).sleep();
    }
    return 0;
}
