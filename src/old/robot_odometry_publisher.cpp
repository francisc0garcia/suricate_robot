#include "robot_micro/robot_odometry_publisher.h"
#include <stdio.h>

const int pin_ss_left = 24;
const int pin_ss_right = 25;
int g_spi_handle;
int result = 0;

int main(int argc, char **argv){

    if (gpioInitialise()<0) {
        ROS_ERROR("Couldn't initialize gpioInitialise!");
        return -1;
    }

    gpioSetMode(pin_ss_left, PI_OUTPUT);
    gpioSetMode(pin_ss_right, PI_OUTPUT);

    gpioWrite(pin_ss_left, PI_HIGH);
    gpioWrite(pin_ss_right, PI_HIGH);

    unsigned int SPI_CHANNEL = 0;
    unsigned int SPI_SPEED = 5000000;
    unsigned int SPI_MODE = 3;

    char write_cmd[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    char read_data[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };

    g_spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, SPI_MODE);
    printf("SPI Handle: %d\n", g_spi_handle);

    // initialize encoder
    gpioWrite(pin_ss_left, PI_LOW);
    gpioWrite(pin_ss_right, PI_LOW);

    write_cmd[0] = 0x88;
    write_cmd[1] = 0x03;

    result = spiXfer(g_spi_handle, (char*)&write_cmd, (char*)&read_data, 2);

    gpioWrite(pin_ss_left, PI_HIGH);
    gpioWrite(pin_ss_right, PI_HIGH);

    printf("SPI init: %02X %02X result: %d\n", read_data[0], read_data[1], result);

    gpioDelay(500);

    // clear encoder
    /*
    gpioWrite(pin_ss_left, PI_LOW);
    gpioWrite(pin_ss_right, PI_LOW);

    write_cmd[0] = (char)0x98;
    write_cmd[1] = 0x00;
    write_cmd[2] = 0x00;
    write_cmd[3] = 0x00;
    write_cmd[4] = 0x00;

    result = spiXfer(g_spi_handle, (char*)&write_cmd, (char*)&read_data, 5);

    gpioWrite(pin_ss_left, PI_HIGH);
    gpioWrite(pin_ss_right, PI_HIGH);

    gpioDelay(500);

    gpioWrite(pin_ss_left, PI_LOW);
    gpioWrite(pin_ss_right, PI_LOW);

    write_cmd[0] = (char)0xE0;

    result = spiXfer(g_spi_handle, (char*)&write_cmd, (char*)&read_data, 1);

    gpioWrite(pin_ss_left, PI_HIGH);
    gpioWrite(pin_ss_right, PI_HIGH);

    printf("SPI clear: %02X %02X %02X %02X %02X result: %d \n", read_data[0],
           read_data[1], read_data[2], read_data[3], read_data[4], result);

    gpioDelay(1000);
*/
    // Read encoder
    gpioWrite(pin_ss_left, PI_LOW);
    gpioWrite(pin_ss_right, PI_LOW);

    write_cmd[0] = 0x60;
    write_cmd[1] = 0x00;
    write_cmd[2] = 0x00;
    write_cmd[3] = 0x00;
    write_cmd[4] = 0x00;

    result =  spiWrite(g_spi_handle, (char*)&write_cmd, 1);
    result =  spiRead(g_spi_handle, (char*)&read_data, 4);

    //result = spiXfer(g_spi_handle, (char*)&write_cmd, (char*)&read_data, 5);

    gpioWrite(pin_ss_left, PI_HIGH);
    gpioWrite(pin_ss_right, PI_HIGH);

    printf("SPI Read: %02X %02X %02X %02X %02X result: %d \n", read_data[0],
           read_data[1], read_data[2], read_data[3], read_data[4], result);

    usleep(200);

    spiClose(g_spi_handle);

    gpioTerminate();

    return 0;
}