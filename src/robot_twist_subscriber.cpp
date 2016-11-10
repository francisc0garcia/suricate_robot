#include "robot_micro/robot_twist_subscriber.h"

int max_pwm = 255;
double scaling_pwm = 1;
int pwm_R = 0;
int pwm_L = 0;
int rate = 100;

std::string topic_name = "";


#define M1_DIR 13 //Direction  C
#define M1_PWM 6

#define M2_DIR 24 //Direction
#define M2_PWM 25

//encoders
#define E1_ENABLE 5
#define E2_ENABLE 19

bool enable_motors = true;

void resetPWM(){
    // Reset
    gpioPWM(M1_PWM, 0);
    gpioPWM(M2_PWM, 0);

    gpioWrite(M1_DIR, PI_LOW);
    gpioWrite(M2_DIR, PI_LOW);
    gpioWrite(M1_PWM, PI_LOW);
    gpioWrite(M2_PWM, PI_LOW);
}

// TODO: Rename the Twist command ()subscriber into a Wrench

void velCallback(geometry_msgs::Twist::ConstPtr vel)
{
    if(enable_motors){
        double dx = vel->linear.x;
        double dr = vel->angular.z;
        //double dy = vel->linear.y;

        //double right = 1.0 * dx + dr * scaling_pwm / 2 ;
        //double left = 1.0 * dx - dr * scaling_pwm / 2;
        double radious = 0.16;
        double L_p = 0.6655;

        double right = ((100/15)/(2*radious) ) * (dx + L_p * dr)  ;
        double left = ((100/15)/(2*radious) ) * (dx - L_p * dr)  ;

        if(right > 0){
            pwm_R = (int)(max_pwm * right);
            if(pwm_R > max_pwm) pwm_R = max_pwm;

            gpioWrite(M1_DIR, PI_LOW);
            gpioPWM(M1_PWM, pwm_R);
        }else{
            pwm_R = (int)(-1 * max_pwm * right);
            if(pwm_R > max_pwm) pwm_R = max_pwm;

            gpioWrite(M1_DIR, PI_HIGH);
            gpioPWM(M1_PWM, pwm_R);
        }

        if(left > 0){
            pwm_L = (int)(max_pwm * left);
            if(pwm_L > max_pwm) pwm_L = max_pwm;

            gpioWrite(M2_DIR, PI_LOW);
            gpioPWM(M2_PWM, pwm_R);
        }else{
            pwm_L = (int)(-1 * max_pwm * left);
            if(pwm_L > max_pwm) pwm_L = max_pwm;

            gpioWrite(M2_DIR, PI_HIGH);
            gpioPWM(M2_PWM, pwm_R);
        }
    } else
    {
        ROS_INFO("Motors are disabled, check system state.");
        sleep(1);
    }
}

/*
void arrayCallback(std_msgs::Int16MultiArray::ConstPtr array)
{
    ///List of system variables:
    ///0. Arduino connection
    ///1. IMU

    Arr[0] =  array->data[0];
    Arr[1] =  array->data[1];

    if(Arr[0] == 0 || Arr[1] == 0 ){
        enable_motors = false;
        resetPWM();
    }else{
        enable_motors = true;
    }
}*/

int main(int argc, char **argv)
{
    gpioCfgClock(4, 0, 0);
// todo: RENAME THE twist COMMAND TO A WRENCH COMMAND
    while(gpioInitialise()<0){
        ROS_INFO("Initializing gpio...");
        sleep(1);
    }

    gpioSetPWMfrequency(M1_PWM, 25000);
    gpioSetPWMfrequency(M2_PWM, 25000);

    gpioSetMode(M1_DIR, PI_OUTPUT);
    gpioSetMode(M1_PWM, PI_OUTPUT);
    gpioSetMode(M2_DIR, PI_OUTPUT);
    gpioSetMode(M2_PWM, PI_OUTPUT);

    //Enable encoders
    gpioSetMode(E1_ENABLE, PI_OUTPUT);
    gpioSetMode(E2_ENABLE, PI_OUTPUT);
    gpioWrite(E1_ENABLE, PI_HIGH);
    gpioWrite(E2_ENABLE, PI_HIGH);

    resetPWM();

    ros::init(argc, argv, "robot_twist_suscriber");
    ros::NodeHandle n;
    ros::NodeHandle pnode("~");

    pnode.getParam("max_pwm", max_pwm);
    pnode.getParam("scaling_pwm", scaling_pwm);
    pnode.getParam("topic_name", topic_name);
    pnode.getParam("rate", rate);

    ros::Subscriber sub = n.subscribe(topic_name, 1, velCallback);
    //ros::Subscriber sub1 = n.subscribe("/robot/system_state", 1, arrayCallback);

    ros::Rate r(rate);

    while ( ros::ok() )
    {
        ros::spinOnce(); // refresh once
        r.sleep();
    }

    resetPWM();

    ROS_INFO("Turning off robot_twist_subscriber");

    return 0;
}