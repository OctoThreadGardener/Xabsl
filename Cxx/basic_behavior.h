#pragma once

#include "../Xabsl/XabslEngine/XabslBasicBehavior.h"
#include <iostream>
#include "Definitions.h"


// Headers for the UDP Communication
#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>


#include <stdio.h>
#include <signal.h>
#include <sys/mman.h>
#include <memory.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/rtc.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/stat.h>
#include <arpa/inet.h>

using namespace std;
using namespace xabsl;


/////////////////////////////////////Socket Init /////////////////////////////////////////////
typedef struct {
    char robotID;
    float OutputPacket[11];//the content of this packet
    float InputPacket[10];
    bool ReceiveExecuted;//updated means that receiver can receive information
    bool SendExecuted;//true means that receiver has received the information,and the sender can send;false means the sender is sending
    int outputSocket;
    int inputSocket;
    int port;
    char buffer[256];
    float fPos;
    float fVel;
    float fGoal;
    bool SocketEnable;
}Broadcast;
#define OUTPUT currentFrame.decision_serial_output_data.received_data;
#define __CAT_BEHAVIOR_NAME(_name) behavior_##_name
#define _CAT_BEHAVIOR_NAME(_name) __CAT_BEHAVIOR_NAME(_name)
#define __BEHAVIOR_BUILD_UP(_behavior) class _behavior : public BasicBehavior{\
public:\
    _behavior(xabsl::ErrorHandler &errorHandler)\
        :xabsl::BasicBehavior(#_behavior, errorHandler){}\
    virtual void registerParameters()override{}\
    virtual void _execute();\
    virtual void execute()override{\
        OUTPUT[0] = OUTPUT[1] = OUTPUT[2] = OUTPUT[3] =\
        OUTPUT[4] = OUTPUT[5] = OUTPUT[6] = 0;\
        _execute();\
    }\
}
#define _BEHAVIOR_BUILD_UP(_behavior) __BEHAVIOR_BUILD_UP(_behavior)
#define _BEHAVIOR(_name)  _BEHAVIOR_BUILD_UP(_CAT_BEHAVIOR_NAME(_name))


_BEHAVIOR(initialize_motors);
_BEHAVIOR(nothing);
_BEHAVIOR(end_game);
_BEHAVIOR(initialize);

_BEHAVIOR(center_round_clockwise);
_BEHAVIOR(center_round_anticlockwise);
_BEHAVIOR(round_clockwise);
_BEHAVIOR(round_anticlockwise);

_BEHAVIOR(walk);
_BEHAVIOR(stop_walk);

_BEHAVIOR(step_forward);
_BEHAVIOR(step_backward);
_BEHAVIOR(step_left);
_BEHAVIOR(step_right);

_BEHAVIOR(kick_ball_soft);
_BEHAVIOR(kick_ball_mid);
_BEHAVIOR(kick_ball_strong);

_BEHAVIOR(approach_ball);
_BEHAVIOR(approach_pose);

/*Additions 2018 for opt_go_to_ball*/

_BEHAVIOR(rotate_before_walk)
_BEHAVIOR(during_left_walk);
_BEHAVIOR(during_right_walk);
_BEHAVIOR(rotate_after_walk);