#include <stdio.h>
#include <pigpio.h>

/*

Arduino - wiringPI - Real RPI
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

int main (int argc, char *argv[])
{
    if (gpioInitialise()<0) return 1;

    printf("set mode\n");
    //gpioSetMode(MOTOR1_E, PI_OUTPUT);
    //gpioSetMode(D0, PI_OUTPUT);
    //gpioSetMode(D1, PI_OUTPUT);
    //gpioSetMode(D2, PI_OUTPUT);
    //gpioSetMode(D3, PI_OUTPUT);
    //gpioSetMode(D4, PI_OUTPUT);
    gpioSetMode(D5, PI_OUTPUT);
    gpioSetMode(D6, PI_OUTPUT);
    gpioSetMode(D7, PI_OUTPUT);
    gpioSetMode(D8, PI_OUTPUT);
    //gpioSetMode(D9, PI_OUTPUT);

    gpioWrite(D5, PI_LOW);
    gpioWrite(D6, PI_LOW);
    gpioWrite(D7, PI_LOW);
    gpioWrite(D8, PI_LOW);

    gpioPWM(D5, 200);
    gpioPWM(D7, 200);

    printf("waiting...\n");

    gpioDelay(2000000);

    gpioWrite(D5, PI_LOW);
    gpioWrite(D7, PI_LOW);

    gpioPWM(D6, 200);
    gpioPWM(D8, 200);

    gpioDelay(2000000);

    printf("set off\n");

    //gpioWrite(D0, PI_LOW);
    //gpioWrite(D1, PI_LOW);
    //gpioWrite(D2, PI_LOW);
    //gpioWrite(D3, PI_LOW);
    //gpioWrite(D4, PI_LOW);
    gpioWrite(D5, PI_LOW);
    gpioWrite(D6, PI_LOW);
    gpioWrite(D7, PI_LOW);
    gpioWrite(D8, PI_LOW);
    //gpioWrite(D9, PI_LOW);

    gpioTerminate();
}