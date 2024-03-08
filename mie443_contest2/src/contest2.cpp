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

void get_target(
    float box_x, float box_y, float box_phi, float *target_x, float *target_y, float *target_phi, 
    float min_x, float max_x, float min_y, float max_y, RobotPose current_pos, ros::NodeHandle n
) {
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
    nav_msgs::GetPlan srv;
    srv.request.start = current_pos;
    RobotPose target_pos(*target_x, *target_y, *target_phi); 
    srv.request.goal = target_pos;
    check_path.call(srv);
    
    if (srv.response.plan.poses.size() == 0){
        // Path unavailable; a new path is needed 
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
        else {
            // The target position is stepping onto another box 
            return; 
        }
    }
}

void get_boundary(float *min_x, float *max_x, float *min_y, float *max_y, ros::NodeHandle n) {
    // Initialize feature to check if the target path is feasible 
    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    nav_msgs::GetPlan srv;
    RobotPose origin_pos(0.0, 0.0, 0.0); 
    srv.request.start = origin_pos;
    
    bool boundary_found = false; 
    *min_x = -4.0; 
    while(!boundary_found) {
        RobotPose test_pos(*min_x, 0.0, 0.0); 
        srv.request.goal = test_pos;
        check_path.call(srv);
        if (srv.response.plan.poses.size() > 0){
            boundary_found = true; 
        }
        else {
            *min_x += 0.01; 
        }
    }

    boundary_found = false; 
    *max_x = 4.0; 
    while(!boundary_found) {
        RobotPose test_pos(*max_x, 0.0, 0.0); 
        srv.request.goal = test_pos;
        check_path.call(srv);
        if (srv.response.plan.poses.size() > 0){
            boundary_found = true; 
        }
        else {
            *max_x -= 0.01; 
        }
    }

    boundary_found = false; 
    *min_y = -4.0; 
    while(!boundary_found) {
        RobotPose test_pos(0.0, *min_y, 0.0); 
        srv.request.goal = test_pos;
        check_path.call(srv);
        if (srv.response.plan.poses.size() > 0){
            boundary_found = true; 
        }
        else {
            *min_y += 0.01; 
        }
    }

    boundary_found = false; 
    *max_y = 4.0; 
    while(!boundary_found) {
        RobotPose test_pos(0.0, *max_y, 0.0); 
        srv.request.goal = test_pos;
        check_path.call(srv);
        if (srv.response.plan.poses.size() > 0){
            boundary_found = true; 
        }
        else {
            *max_y -= 0.01; 
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

    int current_box = 0; // current target box in the list 
    float min_x, min_y, max_x, max_y; // boundary of the environment 
    float target_x, target_y, target_phi; // target location and orientation for the robot

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        
        if (current_box == 0) {
            // Get environmnet boundary before initial run 
            get_boundary(&min_x, &max_x, &min_y, &max_y, n); 
        }
        else if (current_box == boxes.coords.size()) {
            // Task completed with all boxes scanned 
            break; 
        }

        // Set targets
        std::cout << "Going to box " << current_box << std::endl; 
        std::cout << "Box location (" << boxes.coords[current_box][0] << ", " << boxes.coords[current_box][1] << ") & phi = " << boxes.coords[current_box][2] << std::endl; 
        get_target(
            boxes.coords[current_box][0], 
            boxes.coords[current_box][1], 
            boxes.coords[current_box][2], 
            &target_x, &target_y, &target_phi, 
            min_x, max_x, min_y, max_y, 
            robotPose, n
        );
        std::cout << "Target location (" << target_x << ", " << target_y << ") & phi = " << target_phi << std::endl; 
        Navigation::moveToGoal(target_x, target_y, target_phi); 
        current_box += 1; 

        // Image recognition @ Olivia 
        int result = imagePipeline.getTemplateID(boxes); //returns the most matched template ID
        std::cout << "Picture ID:" << result << std::endl;
        ros::Duration(0.01).sleep();
    }
    return 0;
}
