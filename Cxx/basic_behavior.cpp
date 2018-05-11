#include "basic_behavior.h"
#include "Definitions.h"

const double STEP_LENGTH 0.15;
const double STEP_WIDTH 0.08;

#define _BEHAVIOR_FUNC(_name) void behavior_##_name::_execute()

_BEHAVIOR_FUNC(initialize_motors){
    printf("[BEHAVIOR]:initialize motors");
}

_BEHAVIOR_FUNC(nothing){
    printf("[BEHAVIOR]:nothing");
    currentFrame.ball_moved_while_moving = false;
    currentFrame.expected_ball_bearing = currentFrame.ballBearing ;
    currentFrame.first_time_track_ball = true;
}

_BEHAVIOR_FUNC(end_game){
    printf("[BEHAVIOR]:end game");
}

_BEHAVIOR_FUNC(initialize){
    printf("[BEHAVIOR]:initialize");
    isInitializing = false;
}


_BEHAVIOR_FUNC(center_round_clockwise){
    printf("[BEHAVIOR]:turn around a center clockwise");
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(center_round_anticlockwise){
    printf("[BEHAVIOR]:turn around a center anti-clockwise");
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(round_clockwise){
    printf("[BEHAVIOR]:turn around clockwise");
    OUTPUT[0] = 3;
    OUTPUT[3] = currentFrame.target_theta/180*M_PI;
    currentFrame.robot_moved = true;
}


_BEHAVIOR_FUNC(round_anticlockwise){
    printf("[BEHAVIOR]:turn around anti-clockwise");
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(walk){
    printf("[BEHAVIOR]:walk to a specific target range and bearing");
    OUTPUT[0] = 1;
    //currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(stop_walk){
    printf("[BEHAVIOR]:walk to a specific target range and bearing");
    OUTPUT[0] = 1;
    //currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(step_forward){
    printf("[BEHAVIOR]:take a step forward of distance ");
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(step_backward){
    printf("[BEHAVIOR]:take a step backward of distance ");
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(step_left){
    printf("[BEHAVIOR]:take a step left of distance ");
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(step_right){
    printf("[BEHAVIOR]:take a step right of distance ");
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(kick_ball_soft){
    printf("[BEHAVIOR]:kick the ball with low strength ");
    currentFrame.robot_moved = true;
}


_BEHAVIOR_FUNC(kick_ball_mid){
    printf("[BEHAVIOR]:kick the ball with medium strength  ");
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(kick_ball_strong){
    printf("[BEHAVIOR]:kick the ball with high strength ");
    OUTPUT[0] = 6;
    OUTPUT[1] = currentFrame.ballRange * cos(currentFrame.ballBearing);
    OUTPUT[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
    OUTPUT[3] = 0.0; //currentFrame.ballBearing;

//    // Debug
//    OUTPUT[1] = 0.2;
//    OUTPUT[2] = -0.1;
//    OUTPUT[3] = 0;

    ROS_INFO("[BEHAVIOR]:kick the ball with high strength at (%f,%f,%f)",OUTPUT[1],
            OUTPUT[2],OUTPUT[3]);
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(approach_ball){
    printf("[BEHAVIOR]:approach ball  ");
    OUTPUT[0] = 3;
    OUTPUT[1] = currentFrame.ballRange * cos(currentFrame.ballBearing) - 0.3;
    OUTPUT[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
    OUTPUT[3] = 0.0;
//    // Debug
//    OUTPUT[1] = 0.8;
//    OUTPUT[2] = 0.1;
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(approach_pose){
    int data_counter = 0;
    currentFrame.approach_pose = true;
    /// Walk with goal pose
    OUTPUT[0] = 3;
    // Go straight to goal
    double    global_next_goal_point_x = currentFrame.robot_goal_x;
    double    global_next_goal_point_y = currentFrame.robot_goal_y;
    double    global_robot_to_next_goal_x = global_next_goal_point_x - currentFrame.robotLoc_x;
    double    global_robot_to_next_goal_y = global_next_goal_point_y - currentFrame.robotLoc_y;

    OUTPUT[1] = global_robot_to_next_goal_x * cos(currentFrame.robotLoc_theta) + 
        global_robot_to_next_goal_y*sin(currentFrame.robotLoc_theta);
    OUTPUT[2] = -global_robot_to_next_goal_x * sin(currentFrame.robotLoc_theta) + 
        global_robot_to_next_goal_y*cos(currentFrame.robotLoc_theta);
    OUTPUT[3] = currentFrame.robot_goal_theta - currentFrame.robotLoc_theta;

    currentFrame.near_target_pose = true;

    ROS_INFO("Trajectory planning: Sent 1 point (%f,%f,%f)",OUTPUT[1],OUTPUT[2],
            OUTPUT[3]);
}

/*
 * Additions 2018 for opt_go_to_ball
 */

_BEHAVIOR_FUNC(rotate_before_walk){
    printf("[BEHAVIOR]:behavior_rotate_before_walk ");
    OUTPUT[0] = 3;
    OUTPUT[3] =  currentFrame.ballBearing;
    currentFrame.robot_moved = true;

}

_BEHAVIOR_FUNC(during_left_walk){
    printf("[BEHAVIOR]:behavior_during_left_walk  ");
    OUTPUT[0] = 1;
    OUTPUT[1] = 0.2;  //v_x
    OUTPUT[2] = 0;  //v_y
    OUTPUT[3] =  0.2 * currentFrame.ballBearing;  //v_theta
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(during_right_walk){
    printf("[BEHAVIOR]:behavior_during_right_walk  ");
    OUTPUT[0] = 1;
    OUTPUT[1] = 0.2;  //v_x
    OUTPUT[2] = 0;  //v_y
    OUTPUT[3] = 0.2 * currentFrame.ballBearing;  //v_theta
    currentFrame.robot_moved = true;
}

_BEHAVIOR_FUNC(rotate_after_walk){
    printf("[BEHAVIOR]:rotate after walk(2018)  ");
    OUTPUT[0] = 3;
    OUTPUT[1] = currentFrame.ballRange * cos(currentFrame.ballBearing);
    OUTPUT[2] = currentFrame.ballRange * sin(currentFrame.ballBearing);
//    OUTPUT[1] = 0.8;
//    OUTPUT[2] = 0.1;
    currentFrame.robot_moved = true;
}