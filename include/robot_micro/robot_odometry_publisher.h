#ifndef ROBOT_MICRO_ROBOT_ODOMETRY_PUBLISHER_H
#define ROBOT_MICRO_ROBOT_ODOMETRY_PUBLISHER_H

// ROS dependencies
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

#include <stdio.h>
#include <pigpio.h>

/* Arduino - wiringPI - Real RPI
D2 	P0  17
D3 	P1  18
D4 	P2  27/22
D5 	P3  22
D6 	P4  23
D7 	P5  24
D8 	P6  25
D9 	P7  4
D10     18
D11 MOSI
D12 MISO
D13 SCL
*/

// Arduino pins equivalent
#define D0 21
#define D1 16
#define D2 17
#define D3 18
#define D4 27
#define D5 22
#define D6 23
#define D7 24
#define D8 25
#define D9 4

#endif //ROBOT_MICRO_ROBOT_ODOMETRY_PUBLISHER_H
