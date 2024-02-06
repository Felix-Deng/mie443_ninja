#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h> 

#include <stdio.h>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <algorithm>

#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)
#define POS_TOL (0.01) // Tolerance for positional accuracy 
#define YAW_TOL (0.01) // Tolerance for yaw angle accuracy (in rad)

float angular = 0.0;
float linear = 0.0;

float posX = 0.0, posY = 0.0, yaw = 0.0; 
float targetX = 0.0, targetY = 0.0, targetYaw = 0.0; 

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED}; 
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle = 20;

/*************************** Graph Setup ***************************/ 
// Store gridpoint geometric and relational data in searched world 
struct Node {
    Node *north; 
    Node *west; 
    Node *south; 
    Node *east; 
    char source; 
    float pos_x, pos_y; 
}; 

// Used to reserve storage space for Node data 
std::vector<Node> nodes; 

// Map direction char to numeric position change 
float step_size = 0.5; 
float dir_w[2] = {-step_size, 0.0};
float dir_e[2] = {step_size, 0.0}; 
float dir_n[2] = {0.0, step_size}; 
float dir_s[2] = {0.0, -step_size};
std::map<char, float(*)[2]> DIRECTION = {
    {'w', &dir_w}, 
    {'e', &dir_e}, 
    {'n', &dir_n}, 
    {'s', &dir_s}
}; 

// Map direction char to yaw 
std::map<char, float> DIR2YAW = {
    {'e', 0.0}, 
    {'n', M_PI / 2.}, 
    {'w', M_PI}, 
    {'s', 3. / 2. * M_PI}
}; 

Node* add_node(Node* c_node, char dir){
    // Initiate a node during mapping (before moving to the new node)
    nodes.push_back(Node()); // reserve storage address in std::vector 
    nodes[-1].pos_x = posX + *(DIRECTION[dir][0]); 
    nodes[-1].pos_y = posY + *(DIRECTION[dir][0]+1); 

    if (dir == 'w') {
        nodes[-1].east = c_node; 
        nodes[-1].source = 'e'; 
        c_node->west = &nodes[-1]; 
    }
    else if (dir == 'e') {
        nodes[-1].west = c_node; 
        nodes[-1].source = 'w'; 
        c_node->east = &nodes[-1]; 
    }
    else if (dir == 'n') {
        nodes[-1].south = c_node; 
        nodes[-1].source = 's'; 
        c_node->north = &nodes[-1]; 
    }
    else if (dir == 's') {
        nodes[-1].north = c_node; 
        nodes[-1].source = 'n'; 
        c_node->south = &nodes[-1]; 
    }

    return &nodes[-1]; 
}

bool check_node_filled(Node* c_node, char dir){
    // Check if the dir of c_node is filled with a node yet 
    if (dir == 'w') {
        if (c_node->west != NULL){
            return true; 
        }
        else {
            return false; 
        }
    }
    else if (dir == 'e') {
        if (c_node->east != NULL){
            return true; 
        }
        else {
            return false; 
        }
    }
    else if (dir == 'n') {
        if (c_node->north != NULL){
            return true; 
        }
        else {
            return false; 
        }
    }
    else {
        if (c_node->south != NULL){
            return true; 
        }
        else {
            return false; 
        }
    }
}

/*************************** Decisions ***************************/ 
char recom_dir(Node* c_node) {
    // Recommend direction to be explored from the current node c_node 
    char avail_dirs[8] = {'n', 'e', 's', 'w', 'n', 'e', 's', 'w'}; 
    char *s_dir; 
    if (abs(yaw - M_PI / 2.) <= YAW_TOL){
        s_dir = &avail_dirs[0]; 
    }
    else if (abs(yaw) <= YAW_TOL){
        s_dir = &avail_dirs[1]; 
    }
    else if (abs(yaw - 3. / 2. * M_PI) <= YAW_TOL){
        s_dir = &avail_dirs[2]; 
    }
    else {
        s_dir = &avail_dirs[3]; 
    }

    for (int i = 0; i < 4; i++){
        if (!check_node_filled(c_node, *(s_dir + i))){
            return *(s_dir + i); 
        }
    }
    return 'b'; // go back one step to c_node->source 

}

float ida_start_searched(std::vector<Node*>* path, float g, float bound) {
    // Implemented the iterative deepening A* search (helper function of solve_graph_home)
    Node node = *((*path)[-1]); 
    float f = g + abs(node.pos_x) + abs(node.pos_y); // use Manhattan distance for heuristic cost 
    if (f > bound) {
        // Search bound reached 
        return f; 
    }
    if (node.source == 'X') {
        // Found the source node 
        return 0.; 
    }
    float min_cost = std::numeric_limits<float>::max(); 
    if (node.north != NULL && std::count((*path).begin(), (*path).end(), node.north)){
        path->push_back(node.north); 
        float t = ida_start_searched(path, g + 1., bound); 
        if (t == 0) {
            // Found the source node 
            return 0.; 
        }
        if (t < min_cost) {
            min_cost = t; 
        }
        path->pop_back(); 
    }
    if (node.west != NULL && std::count((*path).begin(), (*path).end(), node.west)){
        path->push_back(node.west); 
        float t = ida_start_searched(path, g + 1., bound); 
        if (t == 0) {
            // Found the source node 
            return 0.; 
        }
        if (t < min_cost) {
            min_cost = t; 
        }
        path->pop_back(); 
    }
    if (node.south != NULL && std::count((*path).begin(), (*path).end(), node.south)){
        path->push_back(node.south); 
        float t = ida_start_searched(path, g + 1., bound); 
        if (t == 0) {
            // Found the source node 
            return 0.; 
        }
        if (t < min_cost) {
            min_cost = t; 
        }
        path->pop_back(); 
    }
    if (node.east != NULL && std::count((*path).begin(), (*path).end(), node.east)){
        path->push_back(node.east); 
        float t = ida_start_searched(path, g + 1., bound); 
        if (t == 0) {
            // Found the source node 
            return 0.; 
        }
        if (t < min_cost) {
            min_cost = t; 
        }
        path->pop_back(); 
    }
    return min_cost; 
}

std::vector<Node*> solve_graph_home(Node *c_node) {
    // Given the current Node, find the shortest path back to the source Node 
    std::vector<Node*> path; 
    
    float bound = abs(c_node->pos_x) + abs(c_node->pos_y); // initial boundary = heuristic of c_node 
    float t; 
    path.push_back(c_node); 
    while (true){
        t = ida_start_searched(&path, 0, bound); 
        if (t == 0) {
            std::reverse(path.begin(), path.end()); 
            return path; // ordered from source Node to current Node c_node
        }
        if (t == std::numeric_limits<float>::max()){
            ROS_INFO("NO PATH FOUND TO RETURN TO SOURCE"); 
            return {}; 
        }
        bound = t; 
    }
}

/*************************** Movements ***************************/ 
float set_angular(float target, float current) {
    // Return the required angular velocity to reach target yaw 
    float diff_yaw = RAD2DEG(target - current); 
    if (diff_yaw > 180) {
        diff_yaw -= 360; 
    }
    else if (diff_yaw < -180) {
        diff_yaw += 360; 
    }

    if (abs(diff_yaw) <= YAW_TOL) {
        return 0.0; 
    }
    else if (abs(diff_yaw) <= 20.) {
        return M_PI / 12. * ((diff_yaw > 0) - (diff_yaw < 0)); 
    }
    else {
        return M_PI / 6. * ((diff_yaw > 0) - (diff_yaw < 0)); 
    }
}

float set_linear(float target, float current) {
    // Return the required linear velocity to reach target position 
    float diff_pos = target - current; 
    if (abs(diff_pos) <= POS_TOL) {
        return 0.; 
    }
    else if (abs(diff_pos) <= 0.05) {
        return 0.1 * ((diff_pos > 0) - (diff_pos < 0)); 
    }
    else {
        return 0.25 * ((diff_pos > 0) - (diff_pos < 0)); 
    }
}

void set_target(Node* c_node, Node* n_node, float *targets) {
    // Set targets given current Node c_node and next Node n_node 
    // float targets[3] = {target_x, target_y, target_yaw}
    *targets = n_node->pos_x; 
    *(targets + 1) = n_node->pos_y; 
    if (abs(*targets - c_node->pos_x) <= POS_TOL){
        if (*(targets + 1) > c_node->pos_y){
            *(targets + 2) = M_PI / 2.; 
        }
        else {
            *(targets + 2) = 3. / 2. * M_PI; 
        }
    }
    else {
        if (*targets > c_node->pos_x){
            *(targets + 2) = 0.; 
        }
        else {
            *(targets + 2) = M_PI; 
        }
    }
}


/*************************** Hardware ***************************/ 
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


/*************************** Main Program ***************************/ 
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

    // Start world mapping with the source node initiated
    nodes.push_back(Node()); 
    Node *source_node = &nodes[-1]; 
    source_node->pos_x = posX; 
    source_node->pos_y = posY; 
    source_node->source = 'X'; // signify source node 

    // Keep track of current node 
    Node *curr_node = source_node; 
    // Store path to the source Node if going home 
    std::vector<Node> path_to_source; 
    // Keep track of if during exploration
    char exploring; 

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        // Check if any of the bumpers were pressed 
        bool any_bumper_pressed = false; 
        for (uint32_t b_idx=0; b_idx < N_BUMPER; ++b_idx){
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); 
        }

        // Control logic after bumpers are being pressed 
        ROS_INFO("Position: (%f, %f) Orientation: %f degrees Ranges: %f", posX, posY, RAD2DEG(yaw), minLaserDist); 

        if (any_bumper_pressed || minLaserDist <= 0.1) {
            // TBD: Pending decisions 
            // TODO: also add laser distance checking to if statement 
            angular = 0.0; 
            linear = -1.0; 
        }
        else {
            if (targetX != posX || targetY != posY || targetYaw != yaw) {
                // Not currently at target position, set to move to target 
                if (targetX == posX && targetY == posY) {
                    // At correct position, but wrong yaw
                    linear = 0.0; 
                    angular = set_angular(targetYaw, yaw); 
                    if (!angular) {
                        targetYaw = yaw; 
                    }
                }
                else {
                    // At wrong position, yaw doesn't matter yet 
                    float diffX = targetX - posX, diffY = targetY - posY; 
                    float temp_target_yaw; 
                    if (diffX) {
                        if (diffX > 0) {
                            temp_target_yaw = 0.0; // go east 
                        }
                        else {
                            temp_target_yaw = M_PI; // go west 
                        }
                    }
                    else {
                        if (diffY > 0) {
                            temp_target_yaw = M_PI / 2.; // go north 
                        }
                        else {
                            temp_target_yaw = 3. / 2. * M_PI; // go south 
                        }
                    }
                    if (abs(temp_target_yaw - yaw) <= YAW_TOL) {
                        // Correct direction, go straight 
                        angular = 0.0; 
                        if (diffX) {
                            linear = set_linear(targetX, posX); 
                            if (!linear) {
                                targetX = posX; 
                            }
                        }
                        else {
                            linear = set_linear(targetY, posY); 
                            if (!linear) {
                                targetY = posY; 
                            }
                        }
                    }
                    else {
                        // Wrong direction, first turn 
                        linear = 0.0; 
                        angular = set_angular(temp_target_yaw, yaw); 
                    }
                }
            }
            else {
                // At target position, make decisions for next move
                if (secondsElapsed <= 120) {
                    // Sufficient time left, keep exploring 
                    if (exploring != '\0'){
                        // An exploration step was just completed 
                        if (minLaserDist <= step_size) {
                            // Obstacles in front, cannot move 
                            add_node(curr_node, exploring); 
                            exploring = {}; 
                        }
                        else {
                            // Can proceed to the exploring direction 
                            if (!check_node_filled(curr_node, exploring)) {
                                // Exploring new Node 
                                curr_node = add_node(curr_node, exploring); 
                            }
                            else {
                                // Going back to source 
                                if (exploring == 'w') {
                                    curr_node = curr_node->west; 
                                }
                                else if (exploring == 'e') {
                                    curr_node = curr_node->east; 
                                }
                                else if (exploring == 'n') {
                                    curr_node = curr_node->north; 
                                }
                                else {
                                    curr_node = curr_node->south; 
                                }
                            }
                            exploring = '\0'; 
                        }
                    }
                    else {
                        // Decide on next position to explore 
                        char next_dir = recom_dir(curr_node); 
                        if (next_dir == 'b') { 
                            // Go back to source 
                            next_dir = curr_node->source; 
                        }
                        targetYaw = DIR2YAW[next_dir]; 
                    }
                }
                else {
                    // Not enough time left, return to start position 
                    if (path_to_source.empty()) {
                        if (curr_node->source == 'X') {
                            // TBD: Finished early in source 
                            break; 
                        }
                        // Solve current graph to find shortest path to source (DFS)
                        path_to_source = solve_graph_home(curr_node); 
                    }
                    // Execute path to home 
                    float targets[3]; 
                    set_target(curr_node, &path_to_source[-1], targets); 
                    targetX = *targets; 
                    targetY = *(targets + 1); 
                    targetYaw = *(targets + 2); 
                    curr_node = &path_to_source[-1]; 
                    path_to_source.pop_back(); 
                }
            }
        }

        // if(posX < 0.5 && yaw < M_PI / 12 && minLaserDist > 0.7){
        //     angular = 0.0; 
        //     linear = 0.2; 
        // }
        // else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed && minLaserDist > 0.5){
        //     angular = M_PI / 6; 
        //     linear = 0.0; 
        // }
        // else if (minLaserDist > 1. && !any_bumper_pressed){
        //     linear = 0.1; 
        //     if(yaw < 17/36 * M_PI || posX > 0.6){
        //         angular = M_PI / 12.; 
        //     }
        //     else if (yaw < 19/36 * M_PI || posX < 0.4){
        //         angular = -M_PI/12.;
        //     }
        //     else {
        //         angular = 0;
        //     }
        // }
    
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
