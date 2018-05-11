///////////////////////////////////////////////////////////////////////////////////////////////
//// Name         : serial_receiver.h                                                      ////
//// Description  : Header file for serial_receiver.cpp                                    ////
//// Function     : This header file contains assistive definitions and functions for use  ////
////                in the .cpp file, mainly for handling the serial port interaction      ////
//// Dependencies : QtExtSerialPort, for the communication with the serial port            ////
//// Maintainer   : Stasinopoulos Sotirios     email:sotstas@gmail.com                     ////
//// Last update  : 2017.10.10                                                             ////
///////////////////////////////////////////////////////////////////////////////////////////////
//#ifndef DETECTION_BEHAVIOR_H
//#define DETECTION_BEHAVIOR_H

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sstream>
#include <vector>
#include <time.h>
#include <sensor_msgs/LaserScan.h>
#include "QDebug"
#include "QCoreApplication"
#include <QtExtSerialPort/qextserialport.h>
//using namespace std;

#include "decision/SerialReceived.h"

class SerialPort
{
public:
QextSerialPort *m_Port;
QString m_PortName;
int m_BaudRate;
QByteArray serial_input_data_pack_1;
QByteArray serial_input_data_pack_2;
QByteArray serial_output_data;
char* data;
qint64 input_byte_number;
qint64 output_byte_number;
bool data_read_flag;


SerialPort()
{
    m_PortName = QLatin1String("ttyUSB0");
    m_BaudRate=115200;
    input_byte_number = 31;
    //output_byte_number = 16;
    data_read_flag = false;
    serial_input_data_pack_1.resize(input_byte_number);
    serial_input_data_pack_2.resize(input_byte_number);
    //serial_output_data.resize(output_byte_number);
}

~SerialPort()
{
    delete m_Port;
}


int SetPortBaud(int baud)
{
    BaudRateType brt;
    switch(baud)
    {
    case 9600:brt=BAUD9600;break;
    case 115200:brt=BAUD115200;break;
    default :brt=BAUD1152000;break;
    }
    m_Port->setBaudRate(brt);
    return baud;
}

bool OpenPort(QString port_name, int baud)
{
    m_PortName=port_name;    //QLatin1String->more efficient
    m_Port = new QextSerialPort(m_PortName, QextSerialPort::EventDriven);

//    connect(m_Port, SIGNAL(readyRead()), this, SLOT(onDataAvailable()));
//    m_Port->connect(m_Port, SIGNAL(readyRead()), this, SLOT(onDataAvailable()));
    //set parameter
    SetPortBaud(baud);
    m_Port->setFlowControl(FLOW_OFF);
    m_Port->setParity(PAR_NONE);
    m_Port->setDataBits(DATA_8);
    m_Port->setStopBits(STOP_1);

    //open port
    if (m_Port->open(QIODevice::ReadWrite) == true)
     {
            qDebug() << "listening for data on" << m_Port->portName();
            return true;
     }
    else
     {
            qDebug() << "device failed to open:" << m_Port->errorString();
            return false;
     }

}

void ClosePort()
{
        m_Port->close();
}

int SendWriteCommand(QByteArray cmd)
{
    return  m_Port->write(cmd,cmd.size());
}

//void onDataAvailable()
//{
//    QByteArray serial_input_data_remainder;
//    int a;
//    if (a=m_Port->bytesAvailable())
//    {
//        //serial_input_data.resize(a);
//        serial_input_data.resize(read_byte_number);
//        //serial_input_data = m_Port->readAll();
//        m_Port->read(serial_input_data.data(),read_byte_number);
//        data_read_flag = true;
//        // look if I want to flush it to avoid delay
//        //m_Port->flush();
//        serial_input_data_remainder = m_Port->readAll();
//    }
//    else
//    {
//        qDebug() << "Ready to read, but no bytes available";
//        data_read_flag = false;
//    }
//}
};


