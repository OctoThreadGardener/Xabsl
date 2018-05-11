# All packages, symbols needed for decision

## `OUTPUT` in `basic_behavior`

> type: float [7]
> 
> decision serial output data

- rec_data[0]: Command Type

|0|1|2|3|4|5|6|
|----|----|----|----|----|----|----|
|Nothing|Walk with speed mode|Not defined|Walk with target point mode|Kick left leg|Kick right leg|Kick (Use with ball_x, ball_y)|

- rec_data[1]: $v_x$ for 1, $target_x$ for 3, $ball_x$ for 6
- rec_data[2]: $v_y$ for 1, $target_y$ for 3, $ball_y$ for 6
- rec_data[3]: $v_\theta$ for 1, $target_theta$ for 3
- Other: not defined

## `Output Packet Structure` in `basic_behavior.h` in origin file

> Might be the `OUTPUT` in `basic_behavior`
>
> Type: float [6]
> 
- FLOAT[0]: Command Type

    1. Step straight forward (Use with step_amount, step_length)
    2. Step straight backward (Use with step_amount, step_length)
    3. Step left (Use with step_amount, step_width)
    4. Step right (Use with step_amount, step_width)
    5. Walk - curve (Use with target_x, target_y, target_theta)
    6. Circle Clockwise (Use with circle_radius, circle_theta)
    7. Circle Counter-clockwise (Use with circle_radius, circle_theta)
    8. Kick (Use with kick_leg, kick_strength, kick_angle)
    9. Nothing/Stop

- FLOAT[1]: step_amount for 1-4, target_x for 5, circle_radius for 6-7, kick_leg for 8
- FLOAT[2]: step_length for 1-2, step_width for 3-4, target_y for 5, circle_theta for 6-7, kick_strength for 8
- FLOAT[3]: target_theta for 5, kick_angle for 8
- FLOAT[4]: Head yaw
- FLOAT[5]: Head tilt

## Input Packet Structure in `basic_behavior.h` in origin file

> Problem: where to use it?
>
> Type: float [6]

- FLOAT[0]: Time
- FLOAT[1]: Robot_mti_Yaw
- FLOAT[2]: Robot_mti_Pitch
- FLOAT[3]: Robot_mti_Roll
- FLOAT[4]: Odometry_dx (positive:front)
- FLOAT[5]: Odometry_dy (positive:right)
- FLOAT[6]: Odometry_dtheta (positive:clockwise)
- FLOAT[7]: Is_Robot_Moving(0:not moving, 1 moving)
- FLOAT[8]: Can't Kick(0:Can kick-normal, 1 can't kick)

## Symbols

> Defined in `Definitions.h`, generate in `Definitions.cpp`
>
> I have not check the `Definitions.cpp` yet (*ZeroWeight@20180511*)

### Symbols NOT included in CXX (TODO)

## Todo list (*ZeroWeight@20180511*)

## About to modify the logic

- `Definitions.h`: define symbols
- `basic_behavior.h`: define behavior in Xabsl
- `basic_behavior.cpp`: body of behavior

## Have not checked yet (code format)

- `Definitions.h`
- `Definitions.cpp`
- `main_node.cpp`

## No need to check (too far from decision abstraction layer)

- `serial_receiver.*`
- `tools.h`
- `xabsl-debug-interface.*`