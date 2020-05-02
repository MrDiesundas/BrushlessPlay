/*AMS Rotary Sensor AS5047D
 Circuit
 UNO: MOSI pin 11
      MISO pin 12 
      CLK  pin 13
      CSN  pin 10
 */

#include <SPI.h>

//Set Slave Select Pin
//MOSI, MISO, CLK are handeled automatically
int CSN = 10;
int SO = 12;
int SI = 11;
int CLK = 13 ;
float AngOut;
float AngOutOld = 1;
unsigned int angle;
unsigned int t = 0;
float Speed;

void setup() {
  
  Serial.begin(9600);
  Serial.println("muej_AS5047D - 30.3.2020");

  //Set Pin Modes
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN, HIGH);
  //Initialize SPI 
  SPI.begin();
}

void loop() {
  AngOutOld = GetAngle();
  delay(20);
  AngOut = GetAngle();
  if(abs(AngOutOld-AngOut)<=5){
  Speed = (AngOut-AngOutOld)*1000/20;
  }else Speed = 0;
  Serial.print(AngOut);
  Serial.print(" ");
  Serial.println(Speed);
}

float GetAngle(){
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  
  //Send the Command Frame
  digitalWrite(CSN, LOW);
  delayMicroseconds(1);
  SPI.transfer16(0xFFFF);
  digitalWrite(CSN,HIGH);

  //Read data frame
  digitalWrite(CSN, LOW);
  delayMicroseconds(1);
  angle = SPI.transfer16(0xC000);
  digitalWrite(CSN, HIGH);
  SPI.endTransaction;
  angle = (angle & (0x3FFF));
  AngOut = 400*float(angle)/16384;
  return AngOut;
}
