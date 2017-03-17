//#include <TimerOne.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x04

// Max data to read from I2C
#define max_data 5
#define A5 5
#define A3 3
const int controlPin = 9;

int input_data[max_data];

bool comIsOk = 0;

// Data to work with Battery. For state: 1 - OK
int BatteryValue = 0;
int BatteryPers = 0;
int BatteryMin = 440;
int BatteryMax = 600;
int BatteryMaxPers = 550;
bool BatteryState = 0;

// Data to work with Current. For state: 1 - OK
int CurrentValue = 0;
int CurrentMin = 0;
int CurrentMax = 2500;
bool CurrentState = 0;

bool IMU_State = 0;

// Data type of communication (from micro to Rpb)
#define INT_SENT 1
int data[INT_SENT];

// To control a power: 0 - power is OFF; 1 - ON
int power_on = 0;

void setup() {
    Wire.begin(SLAVE_ADDRESS);

    pinMode(controlPin, OUTPUT);
    //digitalWrite(controlPin, 1 - power_on);
    digitalWrite(controlPin, power_on);

     // Define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
}

// callback for received data
void receiveData(int byteCount) {
    int x = 0;

    while (Wire.available()) {
        if (x < max_data) {
            input_data[x] = Wire.read();
            x++;
            if (input_data[x] != 0)
                comIsOk = 1;
        }
    }
}

// callback for sending data
void sendData() {
    Wire.write((byte * ) & data, INT_SENT * sizeof(int));
}

// Define state of variables
int stateDefine(int value, int valueMax, int valueMin) {
    if ((value >= valueMin) && (value <= valueMax))
        return 1;  // Status 1 - variable is in the safety region
    else
        return 0;  // Status 0 - variable is out of the safety region
}

int readCurrent() {
    const int analogIn = A3;
    double average = 0;
    int n = 250;
    for (int i = 0; i < n; i++) {
        average = average + (.026393581 * analogRead(analogIn) - 13.513513513);
        delay(1);
    }
    int value = int(1000 * average / n); // in mA
    return value;
}

int readBattery() {
    const int analogIn = A5;
    double average = 0;
    int n = 5;
    for (int i = 0; i < n; i++) {
        average = average + analogRead(analogIn);
        delay(10);
    }
    int value = int(average / n);
    return value;
}

void safetyCheck() {
    // Reading a values from Input pins
    BatteryValue = readBattery(); //read a voltage of the Battery
    CurrentValue = readCurrent(); //read a voltage of the current

    BatteryState = stateDefine(BatteryValue, BatteryMax, BatteryMin);
    CurrentState = stateDefine(CurrentValue, CurrentMax, CurrentMin);

    IMU_State = input_data[1];

    //if ((IMU_State == 1)&&(BatteryState == 1)&&(CurrentState == 1))
    if (IMU_State == 1)
        power_on = 1;
    else
        power_on = 0;

    // Because of the rele signal is inverted
    //digitalWrite(controlPin, 1 - power_on);
    digitalWrite(controlPin, 1);
}

void loop() {
    safetyCheck();
    BatteryPers = (BatteryValue - BatteryMin) * 100 / (BatteryMaxPers - BatteryMin);

    data[0] = BatteryPers;
}
