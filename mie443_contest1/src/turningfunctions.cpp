// Assume the current not positions are saved as the following
// save to an address in memory to avoid being refreshed in while loop
currentNodeX = posX
currentNodeY = posY
currentYaw = Yaw
currentTime = secondsElapsed


// forward, time dependent using secondsElapsed
void forward(node){
    while(!any_bumper_pressed && secondsElapsed <= current time + 10) {
        ROS_INFO("Forward");
        angular = 0 
        linear = 2.5; 
    }
}

// left turn
void leftTurn(node){   // node refers to the assumed to include input variable listed above
    while(!any_bumper_pressed && yaw <= currentYaw + M_PI / 2) {
        ROS_INFO("Left Turn");
        angular = M_PI / 6; 
        linear = 0; 
    }
}

// right turn
void rightTurn(node){   // node refers to the assumed to include input variable listed above
    while(!any_bumper_pressed && yaw >= currentYaw + M_PI / 2) {
        ROS_INFO("Right Turn");
        angular = - M_PI / 6; 
        linear = 0; 
    }
}

// 180 turn
void backTurn(node){   // node refers to the assumed to include input variable listed above
    if currentYaw >= M_PI {
        while(!any_bumper_pressed && yaw <= currentYaw - M_PI) {
        ROS_INFO("180 Turn, CCW");
        angular = M_PI / 6; 
        linear = 0; 
        }    
    }  
    else {
        while(!any_bumper_pressed && yaw >= currentYaw + M_PI) {
            ROS_INFO("180 Turn, CW");
            angular = - M_PI / 6; 
            linear = 0; 
        }
    }
}
