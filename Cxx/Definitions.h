#pragma once

#include <iostream>
#include <tiff.h>
#include <cmath>
#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include "nav_msgs/Odometry.h"

#include <ros/ros.h>

#include <vision/Goalpost.h>
#include <gamecontroller/gameControl.h>
#include <vision/Ball.h>
#include <vision/Opponents.h>
#include <decision/gyro_euler.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <decision/head_angle.h>     //msg

#include <head_motion/head_pose.h>
#include <decision/SerialReceived.h>
#include <localization/OutputData.h>
#include "math.h"

#define POSE_DESIRED_X_DIST 0.30 //the desired distance in the x-axis of the robot from the desired pose
#define POSE_X_DIST_THRES 0.05 //the threshold from the desired distance in the x-axis
#define POSE_DESIRED_Y_DIST 0.30 //the desired distance in the y-axis of the robot from the desired pose
#define POSE_Y_DIST_THRES 0.08 //the threshold from the desired distance in the x-axis
#define POSE_DESIRED_THETA_DIFF 0.0 //the desired bearing difference from the desired pose bearing
#define POSE_THETA_DIFF_THRES 0.20 //the difference threshold between the robot bearing and the desired pose bearing


#define BALL_DESIRED_X_DIST 0.3 // the desired distance from ball on robot's x-axis to perform the kick ball action
#define BALL_X_DIST_THRES 0.04
#define BALL_Y_DIST_THRES 0.08
#define BALL_BEARING_DIST_THRES 0.1

#define SERIAL_INPUT_BYTE_NUMBER 18
#define SERIAL_OUTPUT_BYTE_NUMBER 16

//#define DEBUG

using namespace std;
using namespace decision;

const string GOAL_RECOG_OUTPUT_TOPIC = "vision/goalpost";
const string GAMECONTROL_OUTPUT_TOPIC = "game_state";
const string BALL_RECOG_OUTPUT_TOPIC = "vision/ball";
const string OPPONENTS_RECOG_OUTPUT_TOPIC = "vision/opponents";
//const string HEAD_GYRO_OUTPUT_TOPIC = "/imu/data";
const string SERIAL_RECEIVED_TOPIC = "decision/serial_receiver";
const string LOCALIZATION_OUTPUT_TOPIC = "localization/output_data";

const int MAIN_NODE_RUNNING_RATE = 10;


enum _orientation
{
    Up = 1,
    Down = 2,
    Left = 3,
    Right = 4,
    Mid = 5,
    UnKnown = 6,
};

enum _headMode
{
    FarLeft = 10,
    FarMid = 11,
    FarRight = 12,
    FarRightBack = 13,
    FarLeftBack = 14,

    CloseLeft = 20,
    CloseMid = 21,		// straight
    CloseRight = 22,
    CloseRightBack = 23,
    CloseLeftBack = 24,

    //Close = 30,

    HorizontalTrack = 41,
    VerticalTrack = 42,
    BothTrack = 43
};

/// ALL angles of XABSL are in degrees
class DataStructure
{
public:
    vision::Goalpost received_goalpost;
    gamecontroller::gameControl received_gamecontrol;
    vision::Ball received_ball;
    vision::Opponents received_opponents;
    localization::OutputData received_localization;

    //Game
    bool isReady;
    bool isSet;
    bool isGameStart;
    bool isGameOver;
    bool Pathplan;
    double timeLeft;
    bool isAttacker;
    bool attacker_changed;
    bool pause;
    double sec_state;

    // STATE2_NORMAL               0
    // STATE2_PENALTYSHOOT         1
    // STATE2_OVERTIME             2
    // STATE2_TIMEOUT              3
    // STATE2_DIRECT_FREEKICK      4
    // STATE2_INDIRECT_FREEKICK    5
    // STATE2_PENALTYKICK          6
    double sec_state_info;
    // STATE2_INFO_PREPARE         0
    // STATE2_INFO_FREEZE          1
    // STATE2_INFO_EXECUTE         2

    bool isShooter;
    bool isPenalized;
    // true, don't move, only search ball
    // false, nothing

    double kickDestTheta; //not used for now
    double state_code;

    //Ball
    bool isBallSeen;
    //double ballCenterInImageX;
    //double ballCenterInImageY;
    double ballBearing;
    double ballBearingCamera;
    double ballRange;
    double ballLoc_world_x;
    double ballLoc_world_y;

    double lostBallDir;

    bool first_time_track_ball;
    bool ball_moved_while_moving;
    double expected_ball_bearing;

    double ballFoundHeadAngleYaw;
    bool cannot_kick;
    double front_distance;
    double side_distance;
    double ball_image_x;
    double ball_image_y;

    bool is_within_tol;         // true if the angle between the body and the direction toward the ball is small enough
    bool is_left;               // true if the body is to the left of the direction toward the ball
    bool is_near_enough;        // true if near enough to ball, and object tracking mode can be enabled
    bool is_ready_to_kick;      // true when near enough to kick


    // Goal
    bool isGoalSeen;
    double goalLocLeft_world_x;
    double goalLocLeft_world_y;
    double goalLocRight_world_x;
    double goalLocRight_world_y;
    double goalCenterRange;
    double goalCenterBearing;

    double goalFoundHeadAngleYaw;

    double robot_goal_bearing_odom;

    //Opponent
    bool isOpponentSeen;
    double opponentCenterBearing;
    double opponentRange;
    double opponentCenter_world_x;
    double opponentCenter_world_y;

    //Robot
    double robotLoc_x;
    double robotLoc_y;
    double robotLoc_theta;
    double locConfidence;
//    double gyroHead;
    double gyroBody;
//    double gyroInitHeadYawTheta;
    double gyroInitBodyYawTheta;
//    double gyro_body_initial;
//    double adjust_theta;
    double odom_orientation;
    double odom_x;
    double odom_y;

    double last_received_loc_time;

    double headAnglePitch;
    double headAngleYaw;

    bool is_robot_moving;
   // bool is_robot_moving_ball_flag;
    bool near_target_pose;

    // Pose
    double robot_goal_x;
    double robot_goal_y;
    double robot_goal_theta;
    double pose_previous_x_dist;
    double pose_previous_y_dist;
    double pose_previous_bearing_diff;
    bool approach_pose;
    bool reached_pose;
    double previous_robot_goal_x;
    double previous_robot_goal_y;
    double previous_robot_goal_theta;
    bool only_turn;

    // Switches
    bool remote_switch_1;
    bool remote_switch_2;
    bool remote_switch_3;
    bool remote_switch_4;

    // Outputs
    double kickangle;
    double kick_leg;
    enum _headMode headMode;
    enum _headMode lastBallDir;
    enum _headMode nextGoalDir;
    double target_range;
    double target_bearing;
    double target_x;
    double target_y;
    double target_theta;
    double kick_speed;
    bool first_kick;
    double turned;
    double robot_goal_bearing;

    double foundBallTime;
    double foundGoalTime;
    bool robot_moved;

    double ball_found_head_angle_yaw;
    double goal_found_head_angle_yaw;

    vector<double> head_angle_yaw_list;

    geometry_msgs::Twist current_cmd_vel;
    geometry_msgs::Twist nav_cmd_vel;

    // Local variables for input/output through serial port
    decision::SerialReceived decision_serial_input_data;
    decision::SerialReceived decision_serial_output_data;

    bool update_command;
    void FlushData();
    void PrintReceivedData(void);
};

void CBOnGoalReceived(const vision::Goalpost::ConstPtr &msg );
void CBOnControllingReceived(const gamecontroller::gameControl::ConstPtr &msg);
void CBOnBallReceived(const vision::Ball::ConstPtr &msg);
void CBOnOpponentsReceived(const vision::Opponents::ConstPtr &msg);
void CBOnLocalizationReceived(const localization::OutputData::ConstPtr &msg);

double dealWithBodyYawTheta(double inputTheta);

bool kick_check(double camera_x , double camera_y);
double kick_check_x(double camera_x , double camera_y);
double kick_check_y(double camera_x , double camera_y);

extern DataStructure currentFrame;
extern DataStructure lastFrame;

extern bool isInitializing;
extern std_msgs::String console_msg;
extern stringstream console_strstream;
