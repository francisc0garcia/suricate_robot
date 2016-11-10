cmake_minimum_required(VERSION 2.8.1)
project(suricate_robot)

# C++14:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#set(CMAKE_CXX_COMPILER "clang++")
#set(CMAKE_C_COMPILER "clang")

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        rospy
        std_msgs
        geometry_msgs
        sensor_msgs
        dynamic_reconfigure
        image_transport
        message_filters
        tf
        nav_msgs
        visualization_msgs
        gazebo_ros
        )

generate_dynamic_reconfigure_options(cfg/controller.cfg)
catkin_package(DEPENDS DEPENDS dynamic_reconfigure)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES robot_micro gazebo_ros_diff_drive
        CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
        DEPENDS  roscpp gazebo_ros gazebo  SDF geometry_msgs
)

find_package( OpenCV REQUIRED)
find_package( PkgConfig REQUIRED)
find_package( Threads REQUIRED)

find_library(wiringPi REQUIRED)
find_library(pigpio REQUIRED)

find_library(WIRINGPI_LIBRARY wiringPi /Documents/software/wiringPi/wiringPi)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(wrench_driver src/gazebo_plugins/wrench_driver.cpp)
target_link_libraries(wrench_driver gazebo_ros_utils ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_utils src/gazebo_plugins/gazebo_ros_utils.cpp)
target_link_libraries(gazebo_ros_utils ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(visual_odometry src/visual_odometry.cpp)
#target_link_libraries(visual_odometry ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(robot_twist_subscriber src/robot_twist_subscriber.cpp)
target_link_libraries(robot_twist_subscriber ${Boost_LIBRARIES}  ${catkin_LIBRARIES}
        pigpio "${CMAKE_THREAD_LIBS_INIT}")

#add_executable(arduino_test src/test/arduino_test.cpp)
#target_link_libraries(arduino_test  ${Boost_LIBRARIES}  ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(kalman_example src/test/kalman_example.cpp)
#target_link_libraries(kalman_example  ${Boost_LIBRARIES}  ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(test_adc src/test/test_adc.cpp)
#target_link_libraries(test_adc  ${Boost_LIBRARIES}  ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})

#add_executable(test_pwm src/test/test_pwm.cpp)
#target_link_libraries(test_pwm pigpio "${CMAKE_THREAD_LIBS_INIT}")

#add_executable(test_i2c src/test/test_i2c.cpp)
#target_link_libraries(test_i2c pigpio "${CMAKE_THREAD_LIBS_INIT}")
