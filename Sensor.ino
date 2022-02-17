#include <Wire.h>
 
//These values are the readings of the whitepoint of the led. Take the back of vinyl sticker and put it againts face of sensor
#define LED_RED 7540.0f
#define LED_GREEN 14470.0f
#define LED_BLUE 7270.0f
 
//Calculate the balancing factors
#define BAL_RED (LED_GREEN/LED_RED)
#define BAL_GREEN (LED_GREEN/LED_GREEN) 
#define BAL_BLUE (LED_GREEN/LED_BLUE)
 
#define I2C_ADDR 0x52
 
uint8_t readBuff[9];
uint16_t red=0;
uint16_t blue=0;
 
void setup() {
  Wire.begin();
  Serial.begin(4800);
  i2cWrite(0x00,0b0111);  //enable light sensor and activate rgb mode and enable proximity sensor
  i2cWrite(0x04,0b01000000); //set to 16 bit resolution for 25ms response time and set measurement rate to 25ms
 // i2cWrite(0x01,0b00110111); //set ir led to 60khz and 125mA
}
 
void loop() {
  i2cRead(0x0A,readBuff,12);

  blue=(readBuff[7]<<8)|readBuff[6];
  red=(readBuff[10]<<8)|readBuff[9];
 
  red*=BAL_RED;
  blue*=BAL_BLUE;
  
  delay(100);

  i2cRead(0x08,readBuff,2);
  Serial.print(readBuff[0] + readBuff[1]);
  Serial.print(" ");
  Serial.print(red);
  Serial.print(" ");
  Serial.print(blue);
  Serial.println(" ");
  delay(100);
  //Serial.println("loop");
}
 
 
void i2cWrite(uint8_t reg, uint8_t val){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
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