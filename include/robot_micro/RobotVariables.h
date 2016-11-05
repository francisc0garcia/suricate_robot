#ifndef ROBOT_MICRO_ROBOTVARIABLES_H
#define ROBOT_MICRO_ROBOTVARIABLES_H

class RobotVariables {
public:
    double battery_voltage = 0;
    double motor_1_current = 0;
    double motor_2_current = 0;
    double velocity_x = 0;
    double velocity_y = 0;
    double velocity_theta = 0;
    double imu_linear_acceleration_x = 0;
    double imu_linear_acceleration_y = 0;
    double imu_linear_acceleration_z = 0;
    double imu_angular_velocity_x = 0;
    double imu_angular_velocity_y = 0;
    double imu_angular_velocity_z = 0;
    double imu_euler_yaw = 0;
    double imu_euler_roll = 0;
    double imu_euler_pitch = 0;

};

#endif //ROBOT_MICRO_ROBOTVARIABLES_H
