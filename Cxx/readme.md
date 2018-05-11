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

> Symbols Defined in `Definitions.h`, generate in `Definitions.cpp`
>
> Behaviors Defined in `basic_behavior.h`, implement in `basic_behavior.cpp`
>
> I have not check the `Definitions.cpp` yet (*ZeroWeight@20180511*)

### Symbols NOT included in CXX (TODO)

### Behaviors NOT implemented (TODO)

## Todo list (*ZeroWeight@20180511*)

## About to modify the logic

- `Definitions.h`: define symbols
- `basic_behavior.h`: define behavior in Xabsl
- `basic_behavior.cpp`: body of behavior
- `Definitions.cpp`: calculate all of the flags which needs calculate

## No need to check the logic

- `main_node.cpp`: only to remember that the code processing head is here

## No need to check (too far from decision abstraction layer)

- `serial_receiver.*`
- `tools.h`
- `xabsl-debug-interface.*`

## Notes

- `Definitions.*` Callback for gyro: missed?
- `Definitions.cpp` Line 190: magic number?
- `Definitions.cpp` Line 336: Cannot understand
- `Definitions.cpp` Deep ♂ Dark ♂ Fantasy
- `main_node.cpp` What is this code for? Related to decision?
- `main_node.cpp` Head mode code here

## Workflow

1. Write the Xabsl code
2. Foreach behavior defined in Xabsl:
    1. Add to `basic_behavior.h`: `_BEHAVIOR(behavior_defined_in_xabsl);`
    2. Implement the behavior in `basic_behavior.cpp`: `_BEHAVIOR_FUNC(behavior_defined_in_xabsl)`
    3. (Notice to delete the `behavior` prefix)
3. Foreach input/output symbol defined in Xabsl:
    1. Update class `DataStructure` in `Definition.h` and define `SymbolinCXX`
    2. Add the symbol to the `main_node.cpp` and Link the Symbol in CXX and Xabsl:
        - `pEngine->registerDecimalInputSymbol("SymbolinXabsl", &currentFrame.SymbolinCXX);` for Decimal Symbol input to Xabsl
        - `pEngine->registerBooleanInputSymbol("SymbolinXabsl", &currentFrame.SymbolinCXX);` for Bool Symbol input to Xabsl
        - `pEngine->registerEnumeratedInputSymbol("SymbolinXabsl", "TypeinXabsl", (int *)(&currentFrame.heSymbolinCXXadMode));` for Enumerate Symbol input to Xabsl
        - For symbols output to CXX (from Xabsl), using `\*OutputSymbol(\*\*\*)` instead.
    3. For Symbols input to Xabsl:
        1. Update the Symbols in FlushData or the implemention of behavior: (everywhere is possible, but notice to prevent ugly code)
    4. For Symbols output to CXX:
        1. The Symbols might be used in everywhere, notice the timing of the symbols: it is DANGEROUS to use a symbols before it is assigned.
4. Congratulations! Contact [Stasinopoulos Sotirios](mailto:sotstas@gmail.com) to compile the code and win RoboCup 2018

![](http://robocup.org/assets/frontend/robocup_logo_no_bg-4f0c91e523e6f7990464c29850b24a3c.png)
![](http://www.robocup2018.org/assets/img/RoboCup2018_Logo_Horizontal_Red-1.jpg)