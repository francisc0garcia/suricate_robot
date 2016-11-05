#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

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

#define SDA_PIN  17
#define SCL_PIN  18
/*
Name	Cmd & Data	Meaning
End	0	No more commands
Escape	1	Next P is two bytes
Start	2	Start condition
Stop	3	Stop condition
Address	4 P	Set I2C address to P
Flags	5 lsb msb	Set I2C flags to lsb + (msb << 8)
Read	6 P	Read P bytes of data
Write	7 P ...	Write P bytes of data
 */
int main( int argc, char *argv[])
{
    uint16_t framAddr = 100;
    uint8_t value = 62;

    int  rcnt;
    char ReadBuf[256];

    char CmdBuf_write[] = {4, 0x50,  // Chip address
                     2, 7, 3, framAddr >> 8, framAddr & 0xFF, value, 3,
                     0 // EOL
    };

    char CmdBuf_read[] = {4, 0x50,  // Chip address
                           2, 7, 2, framAddr >> 8, framAddr & 0xFF, 3,
                           2, 6, 1, 3,
                           0 // EOL
    };

    if (gpioInitialise() < 0) return 1;

    // Open bit banging I2C on standard I2C pins
    if (bbI2COpen(SDA_PIN, SCL_PIN, 100000)) return 1;

    for (int i = 0; i < 1000; ++i) {
        CmdBuf_write[7] = (char)i;

        bbI2CZip(SDA_PIN, CmdBuf_write, sizeof(CmdBuf_write), ReadBuf, sizeof(ReadBuf));

        rcnt = bbI2CZip(SDA_PIN, CmdBuf_read, sizeof(CmdBuf_read), ReadBuf, sizeof(ReadBuf));
        printf("res: %d  dataR: %d \n", rcnt, (int)(ReadBuf[0]) );
    }

    bbI2CClose(SDA_PIN);

    gpioTerminate();

    return 0;
}