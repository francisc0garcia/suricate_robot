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

unsigned int ADCSelChannel(unsigned char Channel)
{
    unsigned int ConvertValue;
    unsigned char i, Chan;
    unsigned char ConvertValueL, ConvertValueH;
    unsigned char delay;

    ConvertValueL = ConvertValueH = 0; //³õÊŒ»¯×ª»»œá¹û
    delay = 0;
    if(digitalRead(EOC))
    {
        digitalWrite(Clock ,0);
        digitalWrite(ChipSelect ,1);
        Wait2us;
        digitalWrite(ChipSelect ,0);
        Wait2us;
        Channel = Channel << 4;
        for (i = 0; i < 4; i ++) //ÊäÈëÐèÒª×ª»»µÄÍšµÀµÄ±àÂë
        {
            Chan = Channel;
            Chan = Chan >> 7;
            digitalWrite(DataIn,Chan & 0x01);
            Wait2us;
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
        //¿ªÊŒŒì²â×ª»»œáÊø±êÖŸ£¬»òÕß×ª»»³¬Ê±³öŽí
        while ((!digitalRead(EOC)) && (delay < 10))
        {
            Wait10us;
            delay ++;
        }
        if (delay == 10)
        {
            return (0xFFFF); //×ª»»³¬Ê±£¬·µ»ØŽíÎóŽúÂë
        }
        else
        {
            Wait10us;
            digitalWrite(Clock ,0);
            digitalWrite(ChipSelect ,1);
            Wait1us;
            digitalWrite(ChipSelect ,0);
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
            digitalWrite(ChipSelect ,1);
            ConvertValue = ConvertValueH;
            ConvertValue <<= 8;
            ConvertValue |= ConvertValueL;
            return ConvertValue; //·µ»Ø×ª»»œá¹û
        }
    }else
        return 0;
}

void reset_adc(){
    digitalWrite(Clock, 1);
    digitalWrite(DataIn, 1);
    digitalWrite(ChipSelect, 1);

    Wait4us;

    digitalWrite(Clock, 0);
    digitalWrite(DataIn, 0);
    digitalWrite(ChipSelect, 0);

    Wait4us;

    digitalWrite(Clock, 1);
    digitalWrite(DataIn, 1);
    digitalWrite(ChipSelect, 1);

    Wait4us;

    digitalWrite(Clock, 0);
    digitalWrite(DataIn, 0);
    digitalWrite(ChipSelect, 0);

    Wait4us;

    digitalWrite(Clock, 1);
    digitalWrite(DataIn, 1);
    digitalWrite(ChipSelect, 1);

    Wait4us;

    digitalWrite(Clock, 0);
    digitalWrite(DataIn, 0);
    digitalWrite(ChipSelect, 0);

    Wait4us;
}

int main(){
    unsigned int re0, re1;

    if (wiringPiSetup() < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return 1 ;
    }

    pinMode (EOC,INPUT);
    pullUpDnControl(EOC, PUD_UP);
    pinMode (DataOut,INPUT);
    pullUpDnControl(DataOut, PUD_UP);

    pinMode (Clock, OUTPUT);
    pinMode (DataIn, OUTPUT);
    pinMode (ChipSelect, OUTPUT);

    reset_adc();

    while(1)
    {
        printf("read: AN0: %d\n", re0);
        delay (10);

        re0=ADCSelChannel(0);
        delay (10);

        //re1=ADCSelChannel(1);
        //delay (7);
    }
}