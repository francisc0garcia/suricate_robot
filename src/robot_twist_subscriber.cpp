#include "robot_micro/robot_wrench_subscriber.h"

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

// Robot specific
double radius = 0.16;
double L_p = 0.6655;

// Reset motors: put zero all outputs
void resetPWM(){
    // Reset
    gpioPWM(M1_PWM, 0);
    gpioPWM(M2_PWM, 0);

    gpioWrite(M1_DIR, PI_LOW);
    gpioWrite(M2_DIR, PI_LOW);
    gpioWrite(M1_PWM, PI_LOW);
    gpioWrite(M2_PWM, PI_LOW);
}

void wrenchCallback(geometry_msgs::Wrench::ConstPtr wrench_temp)
{
    if(enable_motors){
        double dx = wrench_temp->force.x; // vel->linear.x;
        double dr = wrench_temp->torque.z; // vel->angular.z;

        double right = ((100/15)/(2*radius) ) * (dx + L_p * dr)  ;
        double left = ((100/15)/(2*radius) ) * (dx - L_p * dr)  ;

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

int main(int argc, char **argv)
{
    // Define config for GPIO frequency
    gpioCfgClock(4, 0, 0);

    while(gpioInitialise()<0){
        ROS_INFO("Initializing gpio...");
        sleep(1);
    }

    // Define pins for PWM and digital outputs
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

    //Read parameters from launch file
    pnode.getParam("max_pwm", max_pwm);
    pnode.getParam("scaling_pwm", scaling_pwm);
    pnode.getParam("topic_name", topic_name);
    pnode.getParam("rate", rate);

    ros::Subscriber sub = n.subscribe(topic_name, 1, wrenchCallback);

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