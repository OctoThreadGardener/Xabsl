#include <math.h>
#include <iostream>
#include <tiff.h>
#include <cmath>
#include <string>
#include <sstream>
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <fstream>

#include "serial_receiver.h"
#include "decision/SerialReceived.h"

#define MAX_VALUE 1000

decision::SerialReceived serial_output_data;
bool command_update;
double received_data_last_time;

void CBonSerialReceived(const decision::SerialReceived::ConstPtr serial_msg)
{

    serial_output_data = *serial_msg;

    command_update = true;

    ROS_INFO("Serial output received from main decision node: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",serial_output_data.received_data[0],serial_output_data.received_data[1],serial_output_data.received_data[2],
            serial_output_data.received_data[3],serial_output_data.received_data[4],serial_output_data.received_data[5],serial_output_data.received_data[6],serial_output_data.received_data[7],serial_output_data.received_data[8],
            serial_output_data.received_data[9],serial_output_data.received_data[10],serial_output_data.received_data[11],serial_output_data.received_data[12],serial_output_data.received_data[13],
            serial_output_data.received_data[14],serial_output_data.received_data[15],serial_output_data.received_data[16],serial_output_data.received_data[17],serial_output_data.received_data[18],
            serial_output_data.received_data[19]);

    received_data_last_time = ros::Time::now().toSec();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_receiver");
    ros::NodeHandle nodeHandle;

    ros::Publisher publish_serial_receiver = nodeHandle.advertise<decision::SerialReceived>("decision/serial_receiver", 10);
    ros::Subscriber subscriber_serial_receiver = nodeHandle.subscribe("/decision/command_to_serial", 10, CBonSerialReceived);

    // Open the serial port
    SerialPort sp;
    if (sp.OpenPort(sp.m_PortName,sp.m_BaudRate))
    {
        printf("Serial Port opened.\n");
    }
    else
    {
        printf("Serial Port open error.\n");
    }

    ros::Rate rate(10);  // program frequency 0.010sec

    bool log_data = false;
    bool writing_mode = false;
    int log_file_line_counter = 0;
    std::string log_file_name;
    std::string temp_file_name = "/home/sotiris/robocup2018_ws/src/decision/log_data/write_file_.txt";
    std::ostringstream counter_stringstream;

    int message_id_counter=0;

    int input_msg_id_1 = 0;
    int input_msg_id_2 = 0;
    float odometry_x = 0.0;
    float odometry_y = 0.0;
    float odometry_theta = 0.0;
    float gyro_roll = 0.0;
    float gyro_pitch = 0.0;
    float waist_to_front_x = 0.0;
    float waist_to_front_y = 0.0;
    float waist_to_front_theta = 0.0;
    float waist_height = 0.0;
    int robot_moving = 0;
    float switch_1 = 0;
    float switch_2 = 0;
    float switch_3 = 0;
    float switch_4 = 0;


    int read_data_dif = 0;
    int written_data_size = 0;

    QByteArray cmd;  // array for writing

    union floatToByte {
        float fl;
        char ui[4];
    };

    int read_array_1[(int)sp.input_byte_number];
    int read_array_2[(int)sp.input_byte_number];
    double last_writing_time = 0.0;

    while(ros::ok())
    {

        // Publish to the serial port RS232 the values of the command and its parameters


        cmd.resize(30);

        cmd[0] = 0xFF;
        cmd[1] = 0x00;
        if (cmd[1]==0xFF)
            cmd[1]=0x00;
        // update flag becomes true only if we have new data since last time
        if (command_update)
        {
            cmd[2] = 0x01;
            command_update = false;
        }
        else
        {
            cmd[2] = 0x00;
        }

        floatToByte temp_float_to_byte;

        // Check if we have a long time to receive a message, or if we have a very large value, if so, send 0
        if (((ros::Time::now().toSec()-received_data_last_time)> 0.5) || (fabs(serial_output_data.received_data[1]) > MAX_VALUE)
                || (fabs(serial_output_data.received_data[2]) > MAX_VALUE) || (fabs(serial_output_data.received_data[3]) > MAX_VALUE)
                || (fabs(serial_output_data.received_data[4]) > MAX_VALUE) || (fabs(serial_output_data.received_data[5]) > MAX_VALUE)
                || (fabs(serial_output_data.received_data[6]) > MAX_VALUE))
        {
            for(int i=1;i<serial_output_data.received_data.size();i++)
                serial_output_data.received_data[i] = 0;

        }

        //Debug phase
        //        serial_output_data.received_data[1] = 10.01;
        //        serial_output_data.received_data[2] = 20.02;
        //        serial_output_data.received_data[3] = 30.03;
        //        serial_output_data.received_data[4] = 40.04;
        //        serial_output_data.received_data[5] = 50.05;
        //        serial_output_data.received_data[6] = 60.06;

        cmd[3] = serial_output_data.received_data[0]; // command type
        temp_float_to_byte.fl = serial_output_data.received_data[1];
        cmd[4] = temp_float_to_byte.ui[0];
        cmd[5] = temp_float_to_byte.ui[1];
        cmd[6] = temp_float_to_byte.ui[2];
        cmd[7] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[2];
        cmd[8] = temp_float_to_byte.ui[0];
        cmd[9] = temp_float_to_byte.ui[1];
        cmd[10] = temp_float_to_byte.ui[2];
        cmd[11] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[3];
        cmd[12] = temp_float_to_byte.ui[0];
        cmd[13] = temp_float_to_byte.ui[1];
        cmd[14] = temp_float_to_byte.ui[2];
        cmd[15] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[4];
        cmd[16] = temp_float_to_byte.ui[0];
        cmd[17] = temp_float_to_byte.ui[1];
        cmd[18] = temp_float_to_byte.ui[2];
        cmd[19] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[5];
        cmd[20] = temp_float_to_byte.ui[0];
        cmd[21] = temp_float_to_byte.ui[1];
        cmd[22] = temp_float_to_byte.ui[2];
        cmd[23] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[6];
        cmd[24] = temp_float_to_byte.ui[0];
        cmd[25] = temp_float_to_byte.ui[1];
        cmd[26] = temp_float_to_byte.ui[2];
        cmd[27] = temp_float_to_byte.ui[3];


        // Checksum
        cmd[28] = 0;
        for(int i=0;i<28;i++)
            cmd[28] = cmd[28] ^ cmd[i];

        written_data_size = sp.SendWriteCommand(cmd);

        usleep(5000);

        cmd[1] = 0x01;

        temp_float_to_byte.fl = serial_output_data.received_data[7];
        cmd[4] = temp_float_to_byte.ui[0];
        cmd[5] = temp_float_to_byte.ui[1];
        cmd[6] = temp_float_to_byte.ui[2];
        cmd[7] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[8];
        cmd[8] = temp_float_to_byte.ui[0];
        cmd[9] = temp_float_to_byte.ui[1];
        cmd[10] = temp_float_to_byte.ui[2];
        cmd[11] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[9];
        cmd[12] = temp_float_to_byte.ui[0];
        cmd[13] = temp_float_to_byte.ui[1];
        cmd[14] = temp_float_to_byte.ui[2];
        cmd[15] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[10];
        cmd[16] = temp_float_to_byte.ui[0];
        cmd[17] = temp_float_to_byte.ui[1];
        cmd[18] = temp_float_to_byte.ui[2];
        cmd[19] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[11];
        cmd[20] = temp_float_to_byte.ui[0];
        cmd[21] = temp_float_to_byte.ui[1];
        cmd[22] = temp_float_to_byte.ui[2];
        cmd[23] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[12];
        cmd[24] = temp_float_to_byte.ui[0];
        cmd[25] = temp_float_to_byte.ui[1];
        cmd[26] = temp_float_to_byte.ui[2];
        cmd[27] = temp_float_to_byte.ui[3];


        // Checksum
        cmd[28] = 0;
        for(int i=0;i<28;i++)
            cmd[28] = cmd[28] ^ cmd[i];

        written_data_size = sp.SendWriteCommand(cmd);

        usleep(5000);

        cmd[1] = 0x02;

        temp_float_to_byte.fl = serial_output_data.received_data[13];
        cmd[4] = temp_float_to_byte.ui[0];
        cmd[5] = temp_float_to_byte.ui[1];
        cmd[6] = temp_float_to_byte.ui[2];
        cmd[7] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[14];
        cmd[8] = temp_float_to_byte.ui[0];
        cmd[9] = temp_float_to_byte.ui[1];
        cmd[10] = temp_float_to_byte.ui[2];
        cmd[11] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[15];
        cmd[12] = temp_float_to_byte.ui[0];
        cmd[13] = temp_float_to_byte.ui[1];
        cmd[14] = temp_float_to_byte.ui[2];
        cmd[15] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[16];
        cmd[16] = temp_float_to_byte.ui[0];
        cmd[17] = temp_float_to_byte.ui[1];
        cmd[18] = temp_float_to_byte.ui[2];
        cmd[19] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[17];
        cmd[20] = temp_float_to_byte.ui[0];
        cmd[21] = temp_float_to_byte.ui[1];
        cmd[22] = temp_float_to_byte.ui[2];
        cmd[23] = temp_float_to_byte.ui[3];
        temp_float_to_byte.fl = serial_output_data.received_data[18];
        cmd[24] = temp_float_to_byte.ui[0];
        cmd[25] = temp_float_to_byte.ui[1];
        cmd[26] = temp_float_to_byte.ui[2];
        cmd[27] = temp_float_to_byte.ui[3];


        // Checksum
        cmd[28] = 0;
        for(int i=0;i<28;i++)
            cmd[28] = cmd[28] ^ cmd[i];

        written_data_size = sp.SendWriteCommand(cmd);

        //        ROS_INFO("Published serial messsage: length:%d message_count:%d, update:%d, cmd_type:%f, cmd_param_1:%f, cmd_param_2:%f, cmd_param_3:%f, cmd_param_4:%f, cmd_param_5:%f, cmd_param_6:%f",
        //                 (int)written_data_size,(int)(cmd[1]),(int)(cmd[2]),serial_output_data.received_data[0],serial_output_data.received_data[1],serial_output_data.received_data[2],serial_output_data.received_data[3],
        //                serial_output_data.received_data[4],serial_output_data.received_data[5],serial_output_data.received_data[6]);

        /// Read data from the serial port RS232
        int temp_data=0;
        int counter_to_head =0;


        // Read data from serial port
        QByteArray serial_input_data_remainder;
        if (sp.m_Port->bytesAvailable())
        {
            sp.m_Port->read(sp.serial_input_data_pack_1.data(),sp.input_byte_number);
            sp.m_Port->read(sp.serial_input_data_pack_2.data(),sp.input_byte_number);
            sp.data_read_flag = true;
            // look if I want to flush it to avoid delay
            //m_Port->flush();
            serial_input_data_remainder = sp.m_Port->readAll();
        }
        else
        {
            qDebug() << "Ready to read, but no bytes available";
            sp.data_read_flag = false;
        }

        // This may not be necessary

        bool head_found_1 = false, head_found_2 = false;
        bool two_heads_found_1 = false,two_heads_found_2 = false;

        if (sp.data_read_flag)
        {
            read_data_dif = (int)sp.input_byte_number - sp.serial_input_data_pack_1.size();

            // rearrange the array to have the FF,FF as first always
            if ((read_data_dif <= 0))
            {
                for(int i=0;i<sp.serial_input_data_pack_1.size();i++)
                {
                    if (((int)(sp.serial_input_data_pack_1.at(i))==-1) && ((int)(sp.serial_input_data_pack_1.at(i+1))==-1)) //we want FF, add another one
                    {

                        counter_to_head = i;
                        if (head_found_1)
                        {
                            two_heads_found_1 = true;
                            ROS_INFO("Two heads found, ignoring packet 1");
                        }
                        head_found_1 = true;
                    }
                    //ROS_INFO("byte %d:%d",i,(int)sp.serial_input_data_pack_1.at(i));
                }
                //ROS_INFO("data size:%d, remainder:%d, dif:%d, counter to head:%d",sp.serial_input_data_pack_1.size(),serial_input_data_remainder.size(),read_data_dif,counter_to_head);


                for(int i=0;i<sp.serial_input_data_pack_1.size();i++)
                {
                    temp_data = sp.serial_input_data_pack_1.at((i+counter_to_head)%(sp.serial_input_data_pack_1.size()));
                    if (temp_data < 0)
                        temp_data += 256; // because the QByteArray contains numbers 0-127, so if >127 becomes negative

                    if (!two_heads_found_1)
                        read_array_1[i%((int)sp.input_byte_number)] = temp_data;

                }
            }

                read_data_dif = (int)sp.input_byte_number - sp.serial_input_data_pack_2.size();

                // rearrange the array to have the FF,FF as first always
                if ((read_data_dif <= 0))
                {
                    for(int i=0;i<sp.serial_input_data_pack_2.size();i++)
                    {
                        if (((int)(sp.serial_input_data_pack_2.at(i))==-1) && ((int)(sp.serial_input_data_pack_2.at(i+1))==-1)) //we want FF, add another one
                        {

                            counter_to_head = i;
                            if (head_found_2)
                            {
                                two_heads_found_2 = true;
                                ROS_INFO("Two heads found, ignoring packet 2");
                            }
                            head_found_2 = true;
                        }
                        //ROS_INFO("byte %d:%d",i,(int)sp.serial_input_data_pack_2.at(i));
                    }
                    //ROS_INFO("data size:%d, remainder:%d, dif:%d, counter to head:%d",sp.serial_input_data_pack_2.size(),serial_input_data_remainder.size(),read_data_dif,counter_to_head);


                    for(int i=0;i<sp.serial_input_data_pack_2.size();i++)
                    {
                        temp_data = sp.serial_input_data_pack_2.at((i+counter_to_head)%(sp.serial_input_data_pack_2.size()));
                        if (temp_data < 0)
                            temp_data += 256; // because the QByteArray contains numbers 0-127, so if >127 becomes negative

                        if (!two_heads_found_2)
                            read_array_2[i%((int)sp.input_byte_number)] = temp_data;

                    }
                }



            input_msg_id_1 = ((int)read_array_1[2]);

            temp_float_to_byte.ui[0] = read_array_1[3];
            temp_float_to_byte.ui[1] = read_array_1[4];
            temp_float_to_byte.ui[2] = read_array_1[5];
            temp_float_to_byte.ui[3] = read_array_1[6];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               odometry_x = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid odometry_x data:%f",temp_float_to_byte.fl);

            temp_float_to_byte.ui[0] = read_array_1[7];
            temp_float_to_byte.ui[1] = read_array_1[8];
            temp_float_to_byte.ui[2] = read_array_1[9];
            temp_float_to_byte.ui[3] = read_array_1[10];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               odometry_y = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid odometry_y data:%f",temp_float_to_byte.fl);


            temp_float_to_byte.ui[0] = read_array_1[11];
            temp_float_to_byte.ui[1] = read_array_1[12];
            temp_float_to_byte.ui[2] = read_array_1[13];
            temp_float_to_byte.ui[3] = read_array_1[14];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               odometry_theta = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid odometry_theta data:%f",temp_float_to_byte.fl);


            temp_float_to_byte.ui[0] = read_array_1[15];
            temp_float_to_byte.ui[1] = read_array_1[16];
            temp_float_to_byte.ui[2] = read_array_1[17];
            temp_float_to_byte.ui[3] = read_array_1[18];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               gyro_roll = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid gyro_roll data:%f",temp_float_to_byte.fl);


            temp_float_to_byte.ui[0] = read_array_1[19];
            temp_float_to_byte.ui[1] = read_array_1[20];
            temp_float_to_byte.ui[2] = read_array_1[21];
            temp_float_to_byte.ui[3] = read_array_1[22];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               gyro_pitch = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid gyro_pitch data:%f",temp_float_to_byte.fl);


            temp_float_to_byte.ui[0] = read_array_1[23];
            temp_float_to_byte.ui[1] = read_array_1[24];
            temp_float_to_byte.ui[2] = read_array_1[25];
            temp_float_to_byte.ui[3] = read_array_1[26];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               waist_to_front_x = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid waist_to_front_x data:%f",temp_float_to_byte.fl);


            temp_float_to_byte.ui[0] = read_array_1[27];
            temp_float_to_byte.ui[1] = read_array_1[28];
            temp_float_to_byte.ui[2] = read_array_1[29];
            temp_float_to_byte.ui[3] = read_array_1[30];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               waist_to_front_y = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid waist_to_front_y data:%f",temp_float_to_byte.fl);


        input_msg_id_2 = ((int)read_array_2[2]);

            temp_float_to_byte.ui[0] = read_array_2[3];
            temp_float_to_byte.ui[1] = read_array_2[4];
            temp_float_to_byte.ui[2] = read_array_2[5];
            temp_float_to_byte.ui[3] = read_array_2[6];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               waist_to_front_theta = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid waist_to_front_theta data:%f",temp_float_to_byte.fl);


            temp_float_to_byte.ui[0] = read_array_2[7];
            temp_float_to_byte.ui[1] = read_array_2[8];
            temp_float_to_byte.ui[2] = read_array_2[9];
            temp_float_to_byte.ui[3] = read_array_2[10];
            if (fabs(temp_float_to_byte.fl)<10)
            {
               waist_height = temp_float_to_byte.fl;
            }
            else
                ROS_INFO("Invalid waist_height data:%f",temp_float_to_byte.fl);


            robot_moving = ((int)read_array_2[11]);

            switch_1 = ((int)read_array_2[12]);
            switch_2 = ((int)read_array_2[13]);
            switch_3 = ((int)read_array_2[14]);
            switch_4 = ((int)read_array_2[15]);




            if ((waist_height<0.5) || (waist_height>1.2))
            {
                ROS_INFO("Invalid data: %x %x %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",read_array_1[0],
                        read_array_1[1],read_array_1[2],read_array_1[3],read_array_1[4],read_array_1[5],read_array_1[6],read_array_1[7],read_array_1[8],read_array_1[9],read_array_1[10],
                        read_array_1[11],read_array_1[12],read_array_1[13],read_array_1[14],read_array_1[15],read_array_1[16],read_array_1[17],read_array_1[18],read_array_1[19],read_array_1[20],
                        read_array_1[21],read_array_1[22],read_array_1[23],read_array_1[24],read_array_1[25],read_array_1[26],read_array_1[27],read_array_1[28],read_array_1[29],read_array_1[30]
                        );
            }

        }
        else
        {
            odometry_x = 0.0;
            odometry_y = 0.0;
            odometry_theta = 0.0;
            gyro_roll = 0.0;
            gyro_pitch = 0.0;
            waist_to_front_x = 0.0;
            waist_to_front_y = 0.0;
            waist_to_front_theta = 0.0;
            waist_height = 0.0;
            robot_moving = 0;
            switch_1 = 0;
            switch_2 = 0;
            switch_3 = 0;
            switch_4 = 0;


        };

        ROS_INFO("Read serial messsage: message_id:%d, odometry_x:%f, odometry_y:%f, odometry_theta:%f, gyro_roll:%f, gyro_pitch:%f, waist_to_front_x:%f, waist_to_front_y:%f, waist_to_front_theta:%f, waist_height:%f, robot_moving:%d",
                         input_msg_id_1,odometry_x,odometry_y,odometry_theta,gyro_roll,gyro_pitch,waist_to_front_x,waist_to_front_y,waist_to_front_theta,waist_height,robot_moving);

        decision::SerialReceived serial_input_msg;

        serial_input_msg.header.stamp = ros::Time::now();
        serial_input_msg.received_data[0] = odometry_x;
        serial_input_msg.received_data[1] = odometry_y;
        serial_input_msg.received_data[2] = odometry_theta;
        serial_input_msg.received_data[3] = gyro_roll;
        serial_input_msg.received_data[4] = gyro_pitch;
        serial_input_msg.received_data[5] = waist_to_front_x;
        serial_input_msg.received_data[6] = waist_to_front_y;
        serial_input_msg.received_data[7] = waist_to_front_theta;
        serial_input_msg.received_data[8] = waist_height;
        serial_input_msg.received_data[9] = robot_moving;
        serial_input_msg.received_data[10] = switch_1;
        serial_input_msg.received_data[11] = switch_2;
        serial_input_msg.received_data[12] = switch_3;
        serial_input_msg.received_data[13] = switch_4;


        publish_serial_receiver.publish(serial_input_msg);

        message_id_counter++;

        //        ROS_INFO("time difference: %lf",ros::Time::now().toSec()-last_writing_time);
        // Write the collected data to file if the switch is ON,
        // and we have received x bytes with the first being 0xFF
        if ((log_data)
                &&(read_array_1[0]==255))
        {
            if (!writing_mode)
            {
                log_file_name = temp_file_name;
                time_t t = time(0);   // get time now
                struct tm * now = localtime( & t );
                counter_stringstream.str("");
                counter_stringstream.clear();
                counter_stringstream << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-'
                                     <<  now->tm_mday << '_' <<  now->tm_hour << '-' << now->tm_min << '-' << now->tm_sec;
                log_file_name.insert(56,counter_stringstream.str());
                writing_mode = true;
            };
            std::ofstream write_file;
            write_file.open(log_file_name.c_str(),std::ios_base::app);
            write_file << input_msg_id_1 << " " << odometry_x << " " <<  odometry_y << " " <<  odometry_theta;
            write_file << " " << gyro_roll << " " << gyro_pitch;
            write_file << " " << waist_to_front_x << " " << waist_to_front_y << " " << input_msg_id_2;
            write_file << " " << waist_to_front_theta << " " << waist_height << " " << robot_moving;
            write_file << " " << switch_1 << " " << switch_2 << " " << switch_3 << " " << switch_4;
            write_file << " " << ros::Time::now();
            write_file << "\n";
            write_file.close();
            last_writing_time = ros::Time::now().toSec();
            log_file_line_counter++;

            ROS_INFO("Logged robot data from Motion NUC, lines:%d",log_file_line_counter);

        }
        else
        {
            writing_mode = false;
            log_file_line_counter = 0;
        }


        ros::spinOnce();

        rate.sleep();
    }
    sp.ClosePort();

    return 0;

}
