#include "ros/ros.h"
#include "std_msgs/Int16.h"

#include <sstream>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include<wiringPi.h>

#define	EOC	26
#define	Clock	27
#define	DataIn	28
#define	DataOut	29
#define	ChipSelect	30

#define    Wait1us        delayMicroseconds(1);
#define    Wait2us        delayMicroseconds(2);
#define    Wait4us        {Wait2us;Wait2us;}
#define    Wait8us        {Wait4us;Wait4us;}
#define    Wait10us  {Wait8us;Wait2us;}

unsigned int current_left, current_right;

void reset_adc(){
    digitalWrite(Clock, 0);
    digitalWrite(ChipSelect, 1);
    Wait10us;

    digitalWrite(ChipSelect, 0);
    Wait10us;
}

unsigned int ADCSelChannel(unsigned char Channel, unsigned int previous_value)
{
    unsigned int ConvertValue;
    unsigned char i, Chan;
    unsigned char ConvertValueL, ConvertValueH;
    unsigned char delay;

    ConvertValueL = ConvertValueH = 0;
    delay = 0;

    if(digitalRead(EOC))
    {
        digitalWrite(Clock, 0);
        digitalWrite(ChipSelect, 1);
        Wait4us;
        digitalWrite(ChipSelect, 0);
        Wait4us;
        Channel = Channel << 4;
        for (i = 0; i < 4; i ++) //ÊäÈëÐèÒª×ª»»µÄÍšµÀµÄ±àÂë
        {
            Chan = Channel;
            Chan = Chan >> 7;
            digitalWrite(DataIn, Chan & 0x01);
            Wait4us;
            digitalWrite(Clock ,1);
            digitalWrite(Clock ,0);
            Channel = Channel << 1;
        }
        for (i = 0; i < 6;i ++) //ÊäÈë×ª»»Ê±ÖÓ
        {
            digitalWrite(Clock ,1);
            digitalWrite(Clock ,0);
        }
        digitalWrite(ChipSelect ,1);

        while ((!digitalRead(EOC)) && (delay < 20))
        {
            Wait10us;
            delay ++;
        }
        if (delay == 20)
        {
            reset_adc();
            return previous_value; //×ª»»³¬Ê±£¬·µ»ØŽíÎóŽúÂë
        }
        else
        {
            Wait10us;
            digitalWrite(Clock ,0);
            digitalWrite(ChipSelect, 1);
            Wait1us;
            digitalWrite(ChipSelect, 0);
            Wait1us;
            for (i = 0; i < 2; i ++) //¶ÁÈ¡žß¶þÎ»bitÖµ
            {
                digitalWrite(Clock ,1);
                ConvertValueH <<= 1;
                if (digitalRead(DataOut))
                    ConvertValueH |= 0x1;
                digitalWrite(Clock ,0);
                Wait1us;
            }
            for (i = 0; i < 8; i ++) //¶ÁÈ¡µÍ°ËÎ»bitÖµ
            {
                digitalWrite(Clock ,1);
                ConvertValueL <<= 1;
                if (digitalRead(DataOut))
                    ConvertValueL |= 0x1;
                digitalWrite(Clock ,0);
                Wait1us;
            }
            digitalWrite(ChipSelect, 1);
            ConvertValue = ConvertValueH;
            ConvertValue <<= 8;
            ConvertValue |= ConvertValueL;
            return ConvertValue;
        }
    }else
    {
        reset_adc();
        return previous_value;
    }
}

int main(int argc, char **argv){

    delay(1000);

    if (wiringPiSetup() < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return -1 ;
    }

    pinMode (EOC, INPUT);
    pullUpDnControl(EOC, PUD_UP);
    pinMode (DataOut,INPUT);
    pullUpDnControl(DataOut, PUD_UP);

    pinMode (Clock, OUTPUT);
    pinMode (DataIn, OUTPUT);
    pinMode (ChipSelect, OUTPUT);

    reset_adc();

    ros::init(argc, argv, "robot_current_publisher");

    ros::NodeHandle n;

    ros::Publisher current_left_pub = n.advertise<std_msgs::Int16>("/robot/current_left", 1);
    ros::Publisher current_right_pub = n.advertise<std_msgs::Int16>("/robot/current_right", 1);

    ros::Rate loop_rate(100);

    unsigned int max = 800;
    unsigned int min = 300;

    while (ros::ok())
    {
        unsigned int temp_current_left = 0;
        unsigned int temp_current_right = 0;

        temp_current_left = ADCSelChannel(0, current_left);
        delay (15);

        temp_current_right = ADCSelChannel(1, current_right);
        delay (15);

        if(temp_current_left > min && temp_current_left < max)
            current_left = temp_current_left;
        else
            reset_adc();

        if(temp_current_right > min && temp_current_right < max)
            current_right = temp_current_right;
        else
            reset_adc();

        std_msgs::Int16 msg_current_left;
        msg_current_left.data = current_left;

        std_msgs::Int16 msg_current_right;
        msg_current_right.data = current_right;

        // ROS_INFO("current left %d, current right %d", current_left, current_right);

        current_left_pub.publish(msg_current_left);
        current_right_pub.publish(msg_current_right);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}