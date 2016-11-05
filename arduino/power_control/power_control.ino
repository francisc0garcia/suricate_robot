#include <TimerOne.h>
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

#define SLAVE_ADDRESS 0x04

// Max data to read from I2C
#define max_data 5
#define A5 5
#define A3 3
const int controlPin =  9; 

int input_data[max_data];

bool comIsOk = 0;

// Data to work with Battery. For state: 1 - OK
int BatteryValue = 0; 
int BatteryPers = 0; 
int BatteryMin = 446; 
int BatteryMax = 772;
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
  Serial.begin(9600);
  
  Wire.begin(SLAVE_ADDRESS);
  
  // Define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  // LED Matrix
  matrix.begin(0x70);  // pass in the address
  matrix.setBrightness(1); // control a brightness 15 - max, 0 - min
  matrix.setTextSize(1);

  // Initialisation starting
  matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  matrix.setTextColor(LED_ON);
  for (int8_t x=7; x>=-30; x--) {
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print("Init");
    matrix.writeDisplay();
    delay(100);
  }

  pinMode(controlPin, OUTPUT);
  digitalWrite(controlPin, 1 - power_on);
} 

static const uint8_t PROGMEM
  smile_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100 },
  neutral_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10111101,
    B10000001,
    B01000010,
    B00111100 },
  frown_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10011001,
    B10100101,
    B01000010,
    B00111100 };

// callback for received data
void receiveData(int byteCount){
  int x = 0;

  while(Wire.available()) {
    if(x < max_data){
      input_data[x] = Wire.read();  
      x++;
      if (input_data[x] != 0)
        comIsOk = 1;
    }
  }
}

// callback for sending data
void sendData(){
    // Wire.write((byte*) &data, FLOATS_SENT*sizeof(float));
    Wire.write((byte*) &data, INT_SENT*sizeof(int));
}

// Define state of variables

int stateDefine(int value, int valueMax, int valueMin){
  if ((value >= valueMin) && (value <= valueMax))
    return 1;  // Status 1 - variable is in the safty region
  else 
    return 0;  // Status 0 - variable is out of the safty region
}

int readCurrent(){
  const int analogIn = A3;
  double average = 0;
  int n = 250;
  for(int i = 0; i < n; i++) {
    average = average + (.026393581 * analogRead(analogIn) - 13.513513513);
    delay(1);
  }
  int value = int(1000*average/n); // in mA
  return value;
}

int readBattery(){
  const int analogIn = A5;
  double average = 0;
  int n = 10;
  for(int i = 0; i < n; i++) {
    average = average + analogRead(analogIn);
    delay(1);
  }
  int value = int(average/n);
  return value;
}

void safetyCheck(){
  // Reading a values from Input pins
  BatteryValue = readBattery(); //read a voltage of the Battery
  CurrentValue = readCurrent(); //read a voltage of the current
  
  BatteryState = stateDefine(BatteryValue, BatteryMax, BatteryMin);
  CurrentState = stateDefine(CurrentValue, CurrentMax, CurrentMin); 
  
  IMU_State = input_data[1];
  // Serial.print("Bat-y value is      ");
  // Serial.println(BatteryValue);
  
  if ((IMU_State == 1)&&(BatteryState == 1)&&(CurrentState == 1))
    power_on = 1;
  else 
    power_on = 0;

  // Because of the rele signal is inverted
  digitalWrite(controlPin, 1 - power_on);
}

void matrix_commun() {
  if ((IMU_State == 1)&&(BatteryState == 1)&&(CurrentState == 1)&&(comIsOk == 1)){
    safetyCheck();
    matrix.clear();
    matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_ON);
    matrix.writeDisplay();
    delay(200);
  } else {
    safetyCheck();
    matrix.clear();
    matrix.drawBitmap(0, 0, frown_bmp, 8, 8, LED_ON);
    matrix.writeDisplay();
    delay(500);
    if (comIsOk != 1){
      matrix.clear();
      matrix.setCursor(2,0);
      matrix.print("?");
      matrix.writeDisplay();
      safetyCheck();
    } else
      if (IMU_State == 0){
          matrix.clear();
          matrix.setCursor(-1,0);
          matrix.print("im");
          matrix.writeDisplay();
          safetyCheck();
      }
    if (BatteryState == 0){
      for (int8_t x=5; x>=-18; x--) {
        matrix.clear();
        matrix.setCursor(x,0);
        matrix.print("Baty");
        matrix.writeDisplay();
        // delay(5);
        safetyCheck();
      }
    }
    if (CurrentState == 0) {
      for (int8_t x=5; x>=-18; x--) {
        matrix.clear();
        matrix.setCursor(x,0);
        matrix.print("curt");
        matrix.writeDisplay();
        // delay(5);
        safetyCheck();
      }
    }
  }
}

void loop() {  
  safetyCheck();
  BatteryPers = (BatteryValue - BatteryMin)*100/(BatteryMaxPers - BatteryMin);
  Serial.print("Voltage is       ");
  Serial.println(BatteryValue);

  Serial.print("Persentage is      ");
  Serial.println(BatteryPers);
  // Initialisation of sending data
  // data[0] = BatteryPers;
  data[0] = BatteryValue;
  
  matrix_commun();  
}
