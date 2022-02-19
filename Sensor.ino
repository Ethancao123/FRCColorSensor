#include <SoftWire.h>

#include <Wire.h>

 
#define I2C_ADDR 0x52
#define SDA2 3
#define SCL2 5
#define OUTPUT A3
 
uint8_t readBuff[9];
uint8_t readBuff2[9];

SoftWire i2c(SDA2, SCL2);

void setup() {
  Wire.begin();
  i2c.begin();
  Serial.begin(9600);
  i2cWrite(0x00,0b0111);  //enable light sensor and activate rgb mode and enable proximity sensor
  i2cWrite(0x04,0b01000000); //set to 16 bit resolution for 25ms response time and set measurement rate to 25ms
  i2cWrite2(0x00,0b0111);  //enable light sensor and activate rgb mode and enable proximity sensor
  i2cWrite2(0x04,0b01000000); //set to 16 bit resolution for 25ms response time and set measurement rate to 25ms
 // i2cWrite(0x01,0b00110111); //set ir led to 60khz and 125mA
}
 
void loop() {
  i2cRead(0x08,readBuff,2);
  i2cRead2(0x08,readBuff2,2);
  Serial.print(readBuff[0] + readBuff[1]);
  Serial.print("  ");
  Serial.print(readBuff2[0] + readBuff2[1]);
  Serial.println(" ");
  delay(25);
  //Serial.println("loop");
}
 
 
void i2cWrite(uint8_t reg, uint8_t val){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void i2cWrite2(uint8_t reg, uint8_t val){
    i2c.beginTransmission(I2C_ADDR);
    i2c.write(reg);
    i2c.write(val);
    i2c.endTransmission();
}

void i2cRead(uint8_t reg,uint8_t *val,uint16_t len){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDR, len);
    int i = 0;
    while(Wire.available()){
      val[i++] = Wire.read();
    }
}

void i2cRead2(uint8_t reg,uint8_t *val,uint16_t len){
    i2c.beginTransmission(I2C_ADDR);
    i2c.write(reg);
    i2c.endTransmission();
    i2c.requestFrom(I2C_ADDR, len);
    int i = 0;
    while(Wire.available()){
      val[i++] = Wire.read();
    }
}