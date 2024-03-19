#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_datatypes.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <stdlib.h>

#include <cmath>


float angle_rotation_adj(float posx1, float posy1, float posx2, float posy2, float offset){
    return acos(1 - (pow(posx1 - posx2, 2) + pow(posy1 - posy2, 2)) / (2 * pow(offset, 2)));
}

bool check_plan(float start_posx, float start_posy, float start_phi, float goal_posx, float goal_posy, float goal_phi, ros::NodeHandle n){
    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan"); 
    nav_msgs::GetPlan srv; 

    geometry_msgs::Quaternion start_orient = tf::createQuaternionMsgFromYaw(start_phi); 
    srv.request.start.header.frame_id = "map"; 
    srv.request.start.header.stamp = ros::Time::now(); 
    srv.request.start.pose.position.x = start_posx; 
    srv.request.start.pose.position.y = start_posy; 
    srv.request.start.pose.position.z = 0.0; 
    srv.request.start.pose.orientation.x = 0.0; 
    srv.request.start.pose.orientation.y = 0.0; 
    srv.request.start.pose.orientation.z = start_orient.z; 
    srv.request.start.pose.orientation.w = start_orient.w;

    geometry_msgs::Quaternion goal_orient = tf::createQuaternionMsgFromYaw(goal_phi); 
    srv.request.goal.header.frame_id = "map"; 
    srv.request.goal.header.stamp = ros::Time::now(); 
    srv.request.goal.pose.position.x = goal_posx; 
    srv.request.goal.pose.position.y = goal_posy; 
    srv.request.goal.pose.position.z = 0.0; 
    srv.request.goal.pose.orientation.x = 0.0; 
    srv.request.goal.pose.orientation.y = 0.0; 
    srv.request.goal.pose.orientation.z = goal_orient.z; 
    srv.request.goal.pose.orientation.w = goal_orient.w;

    srv.request.tolerance = 0.0; 
    bool callExecuted = check_path.call(srv); 
    if (!callExecuted){
        std::cout << "Call to check plan NOT sent" << std::endl; 
    }
    return srv.response.plan.poses.size() > 0; 
}

void get_target(
    float box_x, float box_y, float box_phi, float *target_x, float *target_y, float *target_phi, 
    float min_x, float max_x, float min_y, float max_y, RobotPose current_pos, ros::NodeHandle n
) {
    float offset = 0.45; 
    *target_x = box_x + offset * std::cos(box_phi); 
    *target_y = box_y + offset * std::sin(box_phi); 
    if (box_phi >= 0.0){
        *target_phi = box_phi - M_PI; 
    }
    else {
        *target_phi = box_phi + M_PI; 
    }
    
    if (check_plan(current_pos.x, current_pos.y, current_pos.phi, *target_x, *target_y, *target_phi, n) == false){
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
            std::cout << "Pass" << std::endl; 
            return; 
        }
    }
}

void get_boundary(float *min_x, float *max_x, float *min_y, float *max_y, ros::NodeHandle n){
    float bound = 5.0, increment = 0.02; 
    bool boundary_found = false; 
    *min_x = -bound; 
    while(!boundary_found) {
        if (check_plan(0.0, 0.0, 0.0, *min_x, 0.0, 0.0, n) == true){
            boundary_found = true; 
        }
        else {
            *min_x += increment; 
        }
    }

    boundary_found = false; 
    *max_x = bound; 
    while(!boundary_found) {
        if (check_plan(0.0, 0.0, 0.0, *max_x, 0.0, 0.0, n) == true){
            boundary_found = true; 
        }
        else {
            *max_x -= increment; 
        }
    }

    boundary_found = false; 
    *min_y = -bound; 
    while(!boundary_found) {
        if (check_plan(0.0, 0.0, 0.0, 0.0, *min_y, 0.0, n) == true){
            boundary_found = true; 
        }
        else {
            *min_y += increment; 
        }
    }

    boundary_found = false; 
    *max_y = bound; 
    while(!boundary_found) {
        if (check_plan(0.0, 0.0, 0.0, 0.0, *max_y, 0.0, n) == true){
            boundary_found = true; 
        }
        else {
            *max_y -= increment; 
        }
    }
}

void write_to_file(int tag_IDs[], Boxes boxes){
    std::ofstream myfile;
    myfile.open("result.txt");

    for (int i = 0; i < boxes.coords.size(); i++){
        myfile << "Coordinate: (" << boxes.coords[i][0] << ", " << boxes.coords[i][1] << ", " << boxes.coords[i][2] << ")\n";
        if (tag_IDs[i] == 4) {
            myfile << "Tag: Empty" << "\n"; 
        }
        else if (tag_IDs[i] == 0) {
            myfile << "Tag: 3" << "\n"; 
        }
        else {
            myfile << "Tag: " << tag_IDs[i] << "\n";
        }
        myfile << "\n";
    }

    myfile.close();
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
    float origin_x, origin_y, origin_phi; // start location and orientation of the robot 
    int pic_IDs [] = {0, 0, 0, 0, 0}; // store the picture IDs scanned 

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        if (current_box == 0) {
            // Store original position of the robot 
            origin_x = robotPose.x; 
            origin_y = robotPose.y; 
            origin_phi = robotPose.phi; 

            // Get environmnet boundary before initial run
            get_boundary(&min_x, &max_x, &min_y, &max_y, n); 
            std::cout << "Min: (" << min_x << ", " << min_y << ") Max: (" << max_x << ", " << max_y << ")" << std::endl; 
        }
        else if (current_box == boxes.coords.size()) {
            // Task completed with all boxes scanned 
            Navigation::moveToGoal(origin_x, origin_y, origin_phi); 
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
        bool reached_dest; 
        reached_dest = Navigation::moveToGoal(target_x, target_y, target_phi); 
        current_box += 1; 
        
        // Image recognition
        int picture_ID; 
        if (reached_dest){
            int temp_ID;
            bool check_ID = true;
            std::chrono::time_point<std::chrono::system_clock> scan_start;
            scan_start = std::chrono::system_clock::now();
            uint64_t scanElapsed = 0;
            while(check_ID) {
                ros::spinOnce();
                scanElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-scan_start).count();
                temp_ID = imagePipeline.getTemplateID(boxes); //returns the most matched template ID
                if(temp_ID != 0) {
                    check_ID = false;
                    picture_ID = temp_ID;
                }
                else if(scanElapsed > 45){
                    check_ID = false;
                    picture_ID = 4;
                }
            }
        }
        else {
            picture_ID = 0; 
        }
        
        //0->invalid, 1->template1, 2->template2, 3->template3, 4->blank page
        std::cout << "Picture ID:" << picture_ID << std::endl;
        pic_IDs[current_box - 1] = picture_ID; 
        
        ros::Duration(0.01).sleep();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); 
    }

    // Save scanned picture IDs to file 
    write_to_file(pic_IDs, boxes); 

    return 0;
}
