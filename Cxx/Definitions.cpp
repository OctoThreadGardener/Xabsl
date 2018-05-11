#include "Definitions.h"

// Call Back functions: just keep a buffer in most cases
// Callback functions for the received messages: goalpost
void CBOnGoalReceived(const vision::Goalpost::ConstPtr &msg)
{
    currentFrame.received_goalpost = *msg;
    auto _goal_number = currentFrame.received_goalpost.goalpost_number;
    if (_goal_number == 2 || _goal_number == 3)
    {
        currentFrame.isGoalSeen = true;
        currentFrame.foundGoalTime = ros::Time::now().toSec();

        //    ROS_INFO("goalFoundHeadAngleYaw: error check ");
        //    ROS_INFO("goalFoundHeadAngleYaw:%f with delay:%f ",currentFrame.goalFoundHeadAngleYaw,
        //             currentFrame.head_angle_yaw_list.size() - trunc(ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec()));

        if (currentFrame.head_angle_yaw_list.size() - 
            trunc(ros::Time::now().toSec() - 
                currentFrame.received_goalpost.header.stamp.toSec()) >10){
            currentFrame.goalFoundHeadAngleYaw = currentFrame.head_angle_yaw_list.at(
                currentFrame.head_angle_yaw_list.size() -
                trunc(ros::Time::now().toSec() -
                    currentFrame.received_goalpost.header.stamp.toSec()) - 1);
        }

        else{
            currentFrame.goalFoundHeadAngleYaw = 0;
        }
    }
    ROS_INFO("goalFoundHeadAngleYaw:%f with delay:%f ",currentFrame.goalFoundHeadAngleYaw,
             ros::Time::now().toSec()-currentFrame.received_goalpost.header.stamp.toSec());

}

// Callback functions for the received messages: game controller
void CBOnControllingReceived(const gamecontroller::gameControl::ConstPtr &msg)
{
    currentFrame.received_gamecontrol = *msg;
    //    ROS_INFO("Received from Gamecontroler:State:%d,Sec_State:%d,time:%d",
    //             msg->state,msg->secondaryState,msg->secsRemaining);
}

// Callback functions for the received messages: ball
void CBOnBallReceived(const vision::Ball::ConstPtr &msg)
{
    currentFrame.received_ball = *msg;
    currentFrame.isBallSeen = true;
    currentFrame.foundBallTime = ros::Time::now().toSec();
    //ROS_INFO("ballFoundHeadAngleYaw: error check ");
    if (currentFrame.head_angle_yaw_list.size() - 
        trunc(ros::Time::now().toSec() - 
            currentFrame.received_ball.header.stamp.toSec()) >10){

        currentFrame.ballFoundHeadAngleYaw = currentFrame.head_angle_yaw_list.at(
            currentFrame.head_angle_yaw_list.size() -
            trunc(ros::Time::now().toSec() - 
                currentFrame.received_ball.header.stamp.toSec()) - 1);
    }
    else
    {
        currentFrame.ballFoundHeadAngleYaw = 0;
    }

    //    ROS_INFO("ballFoundHeadAngleYaw:%f with delay:%f ",currentFrame.ballFoundHeadAngleYaw,
    //             ros::Time::now().toSec()-currentFrame.received_ball.header.stamp.toSec());
    double ball_distance_x ;
    double ball_distance_y ;

    //currentFrame.cannot_kick = !(kick_check(currentFrame.received_ball.ball_center_x, currentFrame.received_ball.ball_center_y));
    //if (currentFrame.cannot_kick)
    //{
    //    currentFrame.ball_image_x = currentFrame.received_ball.ball_center_x; // ball_x
    //    currentFrame.ball_image_y = currentFrame.received_ball.ball_center_y; // ball_y
    //}

    //currentFrame.front_distance = kick_check_x(currentFrame.received_ball.ball_center_x, currentFrame.received_ball.ball_center_y);
    //currentFrame.side_distance = kick_check_y(currentFrame.received_ball.ball_center_x, currentFrame.received_ball.ball_center_y);
}

// Callback functions for the received messages: opponent
void CBOnOpponentsReceived(const vision::Opponents::ConstPtr &msg)
{
    currentFrame.received_opponents = *msg;
}

// Callback functions for the received messages: localization
void CBOnLocalizationReceived(const localization::OutputData::ConstPtr &msg)
{
    currentFrame.received_localization = *msg;

    currentFrame.last_received_loc_time = msg->header.stamp.toSec();

    ROS_INFO("Received localization: x=%f, y=%f, theta=%f, conf=%f",
             currentFrame.received_localization.robotPose.x,currentFrame.received_localization.robotPose.y,
             currentFrame.received_localization.robotPose.theta/M_PI*180,currentFrame.received_localization.robotPoseConfidence);

}

// Function that refreshes all the data received by the callback functions
void DataStructure::FlushData()
{
    //this->kickDestTheta = lastFrame.kickDestTheta;//keep the value, this value is changed in the service "angle"
    lastFrame = *this;

    ///Game
    timeLeft = static_cast<double>(received_gamecontrol.secsRemaining);

    isReady = received_gamecontrol.state == 1;
    isSet = received_gamecontrol.state == 2;
    isGameStart = received_gamecontrol.state == 3;

    /// need to change this because rules have changed!!!
    isGameOver = received_gamecontrol.state == 4 || timeLeft > 600; 

    // Need to see how to add the secondary states
    attacker_changed = isAttacker xor (static_cast<bool>(received_gamecontrol.kickOffTeam == 45));

    this->isAttacker =  static_cast<bool>(received_gamecontrol.kickOffTeam == 45);
    // STATE2_NORMAL               0
    // STATE2_PENALTYSHOOT         1
    // STATE2_OVERTIME             2
    // STATE2_TIMEOUT              3
    // STATE2_DIRECT_FREEKICK      4
    // STATE2_INDIRECT_FREEKICK    5
    // STATE2_PENALTYKICK          6
    switch (received_gamecontrol.secondaryState){
        case 1:case 3:case 4:case 5:case 6:  pause = true; break;
        default: pause = false;
    }

    this->sec_state = received_gamecontrol.secondaryState;
    this->sec_state_info = received_gamecontrol.secondaryStateInfo;
    this->isShooter = static_cast<bool>(received_gamecontrol.secondaryStateTeam == 45);
    this->isPenalized = static_cast<bool>(!(received_gamecontrol.penalty == 0));

    ///Ball
    if (currentFrame.isBallSeen == true and 
        ros::Time::now().toSec() - currentFrame.foundBallTime > 3)
            this->isBallSeen = false;

    //this->ballBearing = static_cast<double>(received_ball.ball_bearing);
    this->ballBearingCamera = - static_cast<double>(received_ball.ball_bearing);
    this->ballBearing = - static_cast<double>(received_ball.ball_bearing) 
        + currentFrame.ballFoundHeadAngleYaw / 180.0 *M_PI ;
    this->ballRange = static_cast<double>(received_ball.ball_range);  //as agreed with 2D

    this->ballLoc_world_x = static_cast<double>(received_localization.ballCenterOnField.x);
    this->ballLoc_world_y = static_cast<double>(received_localization.ballCenterOnField.y);
    this->ball_image_x = static_cast<double>(received_ball.ball_center_x);
    this->ball_image_y = static_cast<double>(received_ball.ball_center_y);

    cout << "ball_image_x:" << this->ball_image_x 
        << "ballBearingCamera:" << this->ballBearingCamera 
        << " ballBearing:" << this->ballBearing << endl 
        << " ballRange:" << this->ballRange << endl;

    cout << "ballLoc_world_x:" << this->ballLoc_world_x 
        << "ballLoc_world_y:" << this->ballLoc_world_y << endl;

    // true if the angle between the body and the direction toward the ball is small enough
    this->is_within_tol = bool(fabs(this->ballBearing) < 0.1 and this->isBallSeen)

    // true if the body is to the left of the direction toward the ball
    this->is_left = bool((this->ballBearing < 0) and (this->isBallSeen));              
 
    // true if near enough to ball, and object tracking mode can be enabled
    this->is_near_enough = bool((this->ballRange < 1.2) and (this->isBallSeen));   

    is_ready_to_kick = true;      // true when near enough to kick

    //Goal
    if (currentFrame.isGoalSeen == true and 
        (ros::Time::now().toSec() - currentFrame.foundGoalTime > 6) and robot_moved)
        {
            this->isGoalSeen = false;
        }

    auto _goal_number = currentFrame.received_goalpost.goalpost_number;
    if ((_goal_number == 2)||(_goal_number == 3))
    {
        this->goalCenterBearing = (
            static_cast<double>(received_goalpost.goalpost_left_bearing) + 
            static_cast<double>(received_goalpost.goalpost_right_bearing)) / 2;
        this->goalCenterRange = (
            static_cast<double>(received_goalpost.goalpost_left_range) + 
            static_cast<double>(received_goalpost.goalpost_right_range)) / 2;
    }

    // Magic number?
    this->goalLocLeft_world_x = 4.5;
    this->goalLocLeft_world_y = -1.3;
    this->goalLocRight_world_x = 4.5;
    this->goalLocRight_world_y = 1.3;

    //    this->robot_goal_bearing_odom =90- atan2((4.5-_odom_x),(_odom_y-0)) -_odom_orientation;

    //    ROS_INFO("Odom x:%f,y:%f,orient:%f,robot_goal_bearing_odom:%f, test1:%f",_odom_x,_odom_y,_odom_orientation,this->robot_goal_bearing_odom,
    //             test1);

    //Opponents
    // For one opponent now
    isOpponentSeen = static_cast<bool>(received_opponents.opponent_detected);
    if (this->isOpponentSeen)
    {
        opponentCenterBearing = static_cast<double>(received_opponents.opponent_bearing[0]);
        opponentRange = static_cast<double>(received_opponents.opponent_range[0]);
        cout << "opponentBearing:" << opponentCenterBearing << endl 
            << "opponentRange:" << opponentRange << endl;
        auto _temp = received_localization.opponentCenterOnField;
        opponentCenter_world_x = static_cast<double>(_temp.x);
        opponentCenter_world_y = static_cast<double>(_temp.y);
    }

    //Robot
    robotLoc_x = static_cast<double>(received_localization.robotPose.x);
    robotLoc_y = static_cast<double>(received_localization.robotPose.y);
    robotLoc_theta = static_cast<double>(received_localization.robotPose.theta);
    locConfidence = static_cast<double>(received_localization.robotPoseConfidence);
    //#endif
}

// Function that prints out the game state, the robot perceived state, and if the ball and goal were seen
void DataStructure::PrintReceivedData(void)
{
    console_strstream << "game state: ";

    switch (currentFrame.received_gamecontrol.state){
        case 0:
            console_strstream << "UnKnown" << endl;break;
        case 1:
            console_strstream << "Ready" << endl; break;
        case 2:
            console_strstream << "Set" << endl;break;
        case 3:
            console_strstream << "Play" << endl;break;
        case 4:
            console_strstream << "Finish" << endl;break;
        default:
            console_strstream << "UnKnown" << endl;break;
    }

    if(currentFrame.isReady == true)
        console_strstream << "Ready" <<endl;
    else
        console_strstream << "UnKnown" <<endl;

    if(currentFrame.isGameStart == true)
        console_strstream << "Play" <<endl;
    else
        console_strstream << "UnKnown" <<endl;

    if(currentFrame.isGameOver == true)
        console_strstream << "Finish" <<endl;
    else
        console_strstream << "UnKnown" <<endl;

    console_strstream << "time: " << timeLeft << endl;
    console_strstream <<"kickangle:  " << kickangle << endl;
    console_strstream << "isBallSeen: " << (isBallSeen ? "true" : "false") << endl;
    if (isBallSeen)
        console_strstream << "ballRange: " << ballRange 
            << "ballBearing: "  << ballBearing << "ballBearingCamera: " 
            << ballBearingCamera  << "within_tol: " << is_within_tol << endl;

    console_strstream << "isGoalSeen: " << boolalpha << isGoalSeen << endl;

    if (isGoalSeen)
        console_strstream << "goalRange: " << goalCenterRange 
            << "goalBearing: " << goalCenterBearing << endl;

    console_strstream << "isOpponentSeen: " << boolalpha << isOpponentSeen << endl;

    if (isOpponentSeen)
        console_strstream << "opponentRange: " << opponentRange 
        << " opponentBearing: " << opponentCenterBearing << endl;

    console_strstream << "locConfidence: " << locConfidence 
        << " robot_moved: " << robot_moved << endl;

    console_strstream << "isAttacker: " << boolalpha << isAttacker 
        << "kick_angle: " << kickangle << endl;

    console_strstream << "is_robot_moving: " << boolalpha << is_robot_moving 
        << "sec_state: " << sec_state << endl;


    console_strstream << "target_range: " << target_range 
        << " target_bearing: " << target_bearing 
        << " target_theta: " << target_theta << endl;

    console_strstream << "gyroBody: " << gyroBody 
        << " odom_orientation: " << odom_orientation << endl;

    console_strstream << "ballFoundHeadAngleYaw: " << ballFoundHeadAngleYaw 
        << " goalFoundHeadAngleYaw: " << goalFoundHeadAngleYaw << endl;

    console_strstream << "cannot_kick: " << cannot_kick << endl;

    console_strstream << "robotLoc_x: " << currentFrame.robotLoc_x 
        << " robotLoc_y: " << currentFrame.robotLoc_y 
        << " robotLoc_theta: " << (currentFrame.robotLoc_theta/M_PI*180) 
        << " locConfidence: " << currentFrame.locConfidence << endl;

    console_strstream << "odom_x: " << currentFrame.odom_x 
    << " odom_y: " << currentFrame.odom_y << " odom_orientation: "
                      << (currentFrame.odom_orientation/M_PI*180) << endl;

    console_strstream << " state_code: " << currentFrame.state_code 
        << " robot_moving: " << currentFrame.is_robot_moving << endl;
}


//Function that corrects the body gyro angle and just calculates the difference from the initial yaw angle
double dealWithBodyYawTheta(double inputYawTheta)
{
    if (isInitializing)
    {
        return inputYawTheta;
    }
    else
    {
        double result;
        result = inputYawTheta - currentFrame.gyroInitBodyYawTheta;

        result = (result + 180) % 360) - 180;

        return result;
    }
}

DataStructure currentFrame;
DataStructure lastFrame;
double gyroInitYawTheta;

bool _can_kick = true;
double _x_distance, _y_distance;

// DEPRECATED: Deep Dark Fantasy
void _kick_check(double camera_x, double camera_y){

    // camera parameters: magic number
    double r_kick_x = (-0.2142 * camera_y + 112.016)/100.0;
    double r_kick_y = (-0.216 * camera_x + 65.793)/100.0;

    //To save time, if can not kick, _y_distance is the distance of r_kick_y to
    //the nearest y we can kick.
    if(r_kick_y > -0.05 && r_kick_y <= 0) {
        _y_distance = 0.0;
    }
    else if(r_kick_y >= -0.2 && r_kick_y <= -0.05) {
        _y_distance = 0.0;
    }
    else if(r_kick_y < -0.2) {
        _y_distance = -0.2 - r_kick_y;
        _can_kick  = false;
    }
    else if(r_kick_y > 0 && r_kick_y < 0.05) {
        _y_distance = 0.0;
    }
    else if(r_kick_y >= 0.05 && r_kick_y <= 0.2) {
        _y_distance = 0.0;
    }
    else if(r_kick_y > 0.2) {
        _y_distance = 0.2 - r_kick_y;
        _can_kick = false;
    }

    //_x_distance should be >0 ,unless an error occured in vision
    if(r_kick_x < 0){
        _x_distance = r_kick_x - 0.23;
        _can_kick = false;
    }

    //To kick far, if can not kick, _x_distance is the distance of r_kick_x to 0.33
    if(r_kick_x < 0.23 && r_kick_x >= 0) {
        _x_distance = 0.0;
    }
    else if(r_kick_x >= 0.23 && r_kick_x <= 0.38) {
        _x_distance = 0.0;
    }
    else if(r_kick_x > 0.38 && r_kick_x <= 0.43) {
        if(abs(r_kick_y) <= 0.09) {
            _x_distance = 0.0;
        }
        else if(abs(r_kick_y) > 0.09 && abs(r_kick_y) <= 0.2) {
            if(r_kick_x > (-0.455 * abs(r_kick_y) + 0.471)) {
                _x_distance = r_kick_x - (-0.455 * abs(r_kick_y) + 0.471);
                _can_kick = false;
            }
            _x_distance = 0.0;
        }
        else if (abs(r_kick_y) > 0.2){
            _x_distance = r_kick_x - 0.38;
            _can_kick = false;
        }
    }
    else if(r_kick_x > 0.43) {
        if(abs(r_kick_y) <= 0.09){
            _x_distance = r_kick_x - 0.43;
            _can_kick = false;
        }
        else if(abs(r_kick_y) > 0.09 && abs(r_kick_y) <= 0.2){
            _x_distance = r_kick_x - (-0.455 * abs(r_kick_y) + 0.471);
            _can_kick = false;
        }
        else if(abs(r_kick_y) > 0.2){
            _x_distance = r_kick_x - 0.38;
            _can_kick = false;
        }
    }
}
bool kick_check(double camera_x , double camera_y){_kick_check(camera_x,camera_y); return _can_kick);}
double kick_check_x(double camera_x , double camera_y){_kick_check(camera_x,camera_y); return _x_distance);}
double kick_check_y(double camera_x , double camera_y){_kick_check(camera_x,camera_y); return _y_distance);}