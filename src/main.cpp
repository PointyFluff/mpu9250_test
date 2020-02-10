// DOA test for MPU-9250
// (C) 2016 Samuel E. Bray


#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

/* 
  * Arduino Pro Mini 5.5v 16Mhz (china)
  * MPU-9250 Motion processing unit
  * 3-Axis Gyroscope
  * 3-Axis Accelerometer
  * AK8963 3-Axis magnetometer (Asahi Kasei Micro)
  
  * 3.3v VDD (on-board Vreg)
  * SDA = A4
  * SCL = A5
  * AD0 = LOW (I2C LOW/HIGH ADDR SELECT, 10k PULL_DOWN)
  * I2C ADDRESS: 0b1101000;
  * 
*/

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
byte mpuADDR = 0b1101000;
byte count = 0;
const byte maxADDR = 128;
const byte pinLED = LED_BUILTIN;
const int sampleSize = 20;

/* MPU-9250 Registers we care about  */
/* SEE DOCUMENT from TDK (PDF): RS-MPU-6000A-00  */


const byte config = 0x1a;
const byte gyro_config = 0x1b;
const byte accel_config = 0x1c;
const byte accel_config_2 = 0x1d;

const byte accel_xout_h = 0x3b; // only need the high byte for our regs, and we'll grab 2 bytes always;
const byte accel_yout_h = 0x3d;
const byte accel_zout_h = 0x3f;

const byte temp_out_h = 0x41;

const byte gyro_xout_h = 0x43;
const byte gyro_yout_h = 0x45;
const byte gyro_zout_h = 0x47;

const byte signal_path_reset = 0x68;
const byte mot_detect_ctrl = 0x69;
const byte pwr_mgmt_1 = 0x6b;
const byte who_am_i = 0x75;



boolean state = LOW;
long data = 0;

void wakeup() {
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, HIGH);
  Serial.begin(9600);
  while(!Serial) delay(10);
  Serial.println("OK");
  digitalWrite(pinLED, LOW);
}

void displayInit() {
  display.setFont(u8g2_font_6x10_tf);
  display.setFontRefHeightExtendedText();
  display.setDrawColor(1);
  display.setFontPosTop();
  display.setFontDirection(0);
}

void draw() {
  display.clearBuffer();
  display.drawFrame(0,0, 129, 12);
  display.drawStr(2, 2, "HELLO WORLD");
  display.sendBuffer();
  Serial.println("Drawing: HELLO WORLD");
  delay(1000);
}

void setReg(byte reg, byte data, bool quiet = true) {
  if(!quiet) {
    Serial.print("\nSetting: 0x");
    Serial.print(reg, HEX);
    Serial.print(" To: 0x");
    Serial.println(data, HEX);
  }
  Wire.beginTransmission(mpuADDR);
  Wire.write(byte(reg));
  Wire.write(byte(data));
  Wire.endTransmission();
}

void startWire() {
  // Start i2c interface, and just loop until we get a response. 
  Wire.begin();
  Wire.setClock(400000); // high speed mode
  Wire.beginTransmission(mpuADDR);
  while(Wire.endTransmission() != 0) {
    Serial.print("NO I2C DEVICE FOUND AT: 0x");
    Serial.println(mpuADDR, HEX);
    digitalWrite(pinLED, HIGH);
    delay(1000);
    Wire.beginTransmission(mpuADDR);
  }
  // Ok, response. carry on. 
 
  digitalWrite(pinLED, LOW);

  Wire.beginTransmission(mpuADDR);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(mpuADDR, 1);
  Serial.print("FOUND 0x");
  Serial.print(mpuADDR, HEX);
  Serial.print(" WHO_AM_I: 0x");
  while(Wire.available())
    Serial.println(Wire.read(), HEX);
  
  Serial.print("Setting configuration...");
  bool quiet = false; 
  byte reset = 0x00;
  setReg(config, reset, quiet);
  setReg(accel_config, reset, quiet);
  setReg(accel_config_2, reset, quiet);
  setReg(gyro_config, reset, quiet);
  Serial.println("DONE");

}

void getTemp() {
  const int numBytes = 2; // number of bytes to fetch for temp data. 
  const int size = sampleSize; // sample size

  long tempData = 0; //celcius tempurature data
  long sum = 0;      //sample sum

// get chunk of data from i2c device 
  for (int i = 0; i < size; i++) {
    Wire.beginTransmission(mpuADDR);
    Wire.write(temp_out_h);
    if(Wire.endTransmission() == 0) {
      Wire.requestFrom(mpuADDR, numBytes);
      if(2 <= Wire.available()) {
        tempData = Wire.read(); // read high byte first
        tempData = tempData << 8; // shft over 8 bits
        tempData |= Wire.read(); // read low byte 
      }
    }
    else {
      Serial.println("\t** COMMUNICATION ERROR** ");
    }
    sum += tempData;
    delay(1);
  }
  tempData = long(sum/size); // push the final result to the data, for whatever. 
  // print the temp 
  Serial.print("Temp: "); 
  Serial.print(float(tempData)/100); 
  delay(1);
}

void getGyro() {
  const int numBytes = 6;
  const int size = sampleSize;

  long gX = 0;
  long gY = 0;
  long gZ = 0;
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for(int i = 0; i < size; i++) {
    Wire.beginTransmission(mpuADDR);
    Wire.write(gyro_xout_h);
    Wire.endTransmission();
    Wire.requestFrom(mpuADDR, numBytes);
    if(Wire.available() >= numBytes) {
      gX = Wire.read(); gX = gX << 8; gX |= Wire.read();
      gY = Wire.read(); gY = gY << 8; gY |= Wire.read();
      gZ = Wire.read(); gZ = gZ << 8; gZ |= Wire.read();
    }
    sumX += gX; sumY += gY; sumZ += gZ;
    delay(1);
  }
  gX = long(sumX/size);
  gY = long(sumY/size);
  gZ = long(sumZ/size);
  Serial.print(" gX: "); Serial.print(float(gX)/100);
  Serial.print(" gY: "); Serial.print(float(gY)/100);
  Serial.print(" gZ: "); Serial.print(float(gZ)/100);
  delay(1);
}

void getAccel() {
  const int numBytes = 6;
  const int size = sampleSize;

  long aX = 0;
  long aY = 0;
  long aZ = 0;
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;

  for(int i = 0; i < size; i++) {
    Wire.beginTransmission(mpuADDR);
    Wire.write(accel_xout_h);
    Wire.endTransmission();
    Wire.requestFrom(mpuADDR, numBytes);
    if(Wire.available() >= numBytes) {
      aX = Wire.read(); aX = aX << 8; aX |= Wire.read();
      aY = Wire.read(); aY = aY << 8; aY |= Wire.read();
      aZ = Wire.read(); aZ = aZ << 8; aZ |= Wire.read();
    }
    sumX += aX; sumY += aY; sumZ += aZ;
    delay(1);
  }
  aX = long(sumX/size);
  aY = long(sumY/size);
  aZ = long(sumZ/size);
  Serial.print(" aX: "); Serial.print(float(aX)/100);
  Serial.print(" aY: "); Serial.print(float(aY)/100);
  Serial.print(" aZ: "); Serial.println(float(aZ)/100);
  delay(1);
}

void setup() {
  wakeup();
  startWire();
  draw();
}

void loop() {
  state = !state;
  digitalWrite(pinLED, state);
  //getTemp();
  //getGyro();
  getAccel();
  delay(25);
}
