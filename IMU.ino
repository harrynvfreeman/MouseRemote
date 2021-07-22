#include <Wire.h>

//conigurations

//accel scale (+-2g)
//float accelCoeff = 2.0 / (1 << 15);

//gyro scale (+-250dps)
//float gyroCoeff = 250.0 / (1 << 15);

int sampleFrequency = 256;

int imuAddress = 104; //1101000
//int imuAddress = 105; //1101001

//Accel values
//---------------------
byte accelXHighAddr = 59;
byte accelXLowAddr = 60;
byte accelYHighAddr = 61;
byte accelYLowAddr = 62;
byte accelZHighAddr = 63;
byte accelZLowAddr = 64;
int numAccelAddresses = 6;

int16_t accelVals[3];
int16_t *accelXVal = accelVals;
int16_t *accelYVal = accelVals + 1;
int16_t *accelZVal = accelVals + 2;

uint8_t *accelXHighVal = ((byte*)accelXVal ) + 1;
uint8_t *accelXLowVal = (byte*)accelXVal;
uint8_t *accelYHighVal = ((byte*)accelYVal ) + 1;
uint8_t *accelYLowVal = (byte*)accelYVal;
uint8_t *accelZHighVal = ((byte*)accelZVal ) + 1;
uint8_t *accelZLowVal = (byte*)accelZVal;
//---------------------

//Temp values
//---------------------
byte tempHighAddr = 65;
byte tempLowAddr = 66;
int numTempAddresses = 2;

int16_t tempVals[1];
int16_t * tempVal = tempVals;

uint8_t *tempHighVal = ((byte*)tempVal ) + 1;
uint8_t *tempLowVal = (byte*)tempVal;
//---------------------

//Gyro values
//---------------------
byte gyroXHighAddr = 67;
byte gyroXLowAddr = 68;
byte gyroYHighAddr = 69;
byte gyroYLowAddr = 70;
byte gyroZHighAddr = 71;
byte gyroZLowAddr = 72;
int numGyroAddresses = 6;

int16_t gyroVals[3];
int16_t *gyroXVal = gyroVals;
int16_t *gyroYVal = gyroVals + 1;
int16_t *gyroZVal = gyroVals + 2;

uint8_t *gryoXHighVal = ((byte*)gyroXVal ) + 1;
uint8_t *gyroXLowVal = (byte*)gyroXVal;
uint8_t *gyroYHighVal = ((byte*)gyroYVal ) + 1;
uint8_t *gyroYLowVal = (byte*)gyroYVal;
uint8_t *gyroZHighVal = ((byte*)gyroZVal ) + 1;
uint8_t *gyroZLowVal = (byte*)gyroZVal;
//---------------------

int numImuAddresses = 14;
byte *imuOutput[14] = {accelXHighVal, accelXLowVal,
                        accelYHighVal, accelYLowVal,
                        accelZHighVal, accelZLowVal,
                        tempHighVal, tempLowVal,
                        gryoXHighVal, gyroXLowVal,
                        gyroYHighVal, gyroYLowVal,
                        gyroZHighVal, gyroZLowVal};

volatile bool i2c = false;
//uint32_t maxSamples = 256*5;
uint32_t maxSamples = 256*60;
uint32_t numSamples = 0;
uint32_t dataA;
uint32_t dataB;
uint32_t dataC;
int main() {
  //init initializes timer counters and 
  //serial communication
  init();
  Serial.begin(115200);
  Wire.begin();  
   
  Serial.println("Starting---------------");
  
  init_timer();
  while(true) {
      if((i2c) && (numSamples < maxSamples)) {
        i2c = false;
        PIND = 0x80;

  
        bool result = readAddresses(accelXHighAddr, numImuAddresses, imuOutput);
        if (!result) {
          Serial.println("ERROR READING I2C!!!");
          return;
        }
        Serial.print(numSamples);
        Serial.print("|");
        Serial.print(*(gyroXVal));
        Serial.print("|");
        Serial.print(*(gyroYVal));
        Serial.print("|");
        Serial.print(*(gyroZVal));
        Serial.print("|");
        Serial.print(*(accelXVal));
        Serial.print("|");
        Serial.print(*(accelYVal));
        Serial.print("|");
        Serial.print(*(accelZVal));
        Serial.print("|");
        Serial.println("END");

        numSamples++;
      }
  }
}

void init_timer() {
  //stop interrupts
  uint8_t store = SREG;
  SREG = 0x00;

  //clear other timers enabled by init
  TCCR0B = 0x00;
  TCCR1B = 0x00;
  
  //stop timer
  TCCR2B = 0x00;
  
  //OC0A, OC0B disconnected
  //WMG02:WGM01 = 11 for fast PWM
  TCCR2A = 0x03;
  
  //Set top
  OCR2A = 0xF4;

  //Set pin7 PD7 as output
  DDRD |= 0x80;
  
  //WGM00 - 1 for fast PWM
  //No clock prescaler
  //Restart clock
  TCCR2B = 0x0E;

  TIMSK2 = 0x01;
  SREG = store | 128;

  PORTD |= 0x80;
}

//0x0020
ISR(TIMER2_OVF_vect) {
  i2c = true;
}

void writeAddress(byte registerAddress, byte data) {
  Wire.beginTransmission(imuAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(true);
}

bool readAddresses(byte registerAddress, int numBytes, byte **output) {
  Wire.beginTransmission(imuAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(imuAddress, numBytes, true);

  for (int i = 0; i < numBytes; i++) {
    if (!Wire.available()) {
      return false;
    }

    *(*(output+i)) = Wire.read();
  }
  
  return true;
 }
