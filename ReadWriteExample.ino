//Example of reading / writing register of MPU9255

#include <Wire.h>

int imuAddress = 104; //1101000
byte SELF_TEST_X_ACCEL = 13;
byte test_val = 170;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  writeAddress(SELF_TEST_X_ACCEL, test_val);

  //Option sleep 5s to reset scope probe
  //delay(5000);
  
  byte output;
  bool result = readAddress(SELF_TEST_X_ACCEL, &output);
  if (result) {
    Serial.print("Output is: ");
    Serial.println(output);
  } else {
    Serial.println("Error");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

void writeAddress(byte registerAddress, byte data) {
  Wire.beginTransmission(imuAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(true);
}

bool readAddress(byte registerAddress, byte *output) {
  Wire.beginTransmission(imuAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(imuAddress, 1, true);

 if (!Wire.available()) {
     return false;
  }

  *(output) = Wire.read();
  
  return true;
 }
