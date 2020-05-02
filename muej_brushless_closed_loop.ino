// Program to run a brushless motor in closed loop mode as a servomotor with proportional control
// Hardware
// - Arduino Uno
// - uses driver L298N
// - AS5047D over SPI

// SOURCES
// OPENLOOP PART FROM BERRYJAM
// Slow and precise BLDC motor driver using SPWM and SVPWM modulation
// Part of code used from http://elabz.com/
// (c) 2015 Ignas Gramba www.berryjam.eu
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
//
// CLOSE LOOP PART FROM Juan Pablo Angulo
// see https://www.youtube.com/watch?v=TnECJG2mN-E
// 
// ANGLE ENCODER AS5047D FROM MatejGomboc
//https://github.com/MatejGomboc/AS5047D-Arduino-Magnetic-Rotary-Encoder/tree/master/AS5047D
//
// MESSY MIXED ALL 3 PARTS BY JOE MUELLER


// Angle AS5047
#include <SPI.h>
const int CSN = 10;
const int SO = 12;
const int SI = 11;
const int CLK = 13 ;

// Motor
const int motorPin1 = 3; //Coil A -> L298N IN1
const int motorPin2 = 5; //Coil B -> L298N IN2
const int motorPin3 = 6; //Coil C -> L298N IN3
const int P = 6; //Motor's number of Poles
int sineArraySize; // number of PWM steps within one Sinus, choose a multiple of 3
const int pwmSin[] = {127, 136, 145, 153, 162, 170, 179, 187, 194, 202, 209, 215, 221, 227, 232, 237, 241, 245, 248, 250, 252, 253, 254, 254, 253, 252, 250, 248, 245, 241, 237, 232, 227, 221, 215, 209, 202, 194, 187, 179, 170, 162, 153, 145, 136, 127, 118, 109, 101, 92, 84, 75, 67, 60, 52, 45, 39, 33, 27, 22, 17, 13, 9, 6, 4, 2, 1, 0, 0, 1, 2, 4, 6, 9, 13, 17, 22, 27, 33, 39, 45, 52, 60, 67, 75, 84, 92, 101, 109, 118, 127};

// Controller
const int K = 5; // Proportional gain for position control
const int G = 95; // Gain 0 ... 100

// HMI
const int potPin = A0;  // pot controls the RPM speed


// Variables
int currentStepA;  //index for lookup table Coil A
int currentStepB; //index for lookup table Coil B
int currentStepC; //index for lookup table Coil C
int pos = 0; //mechanical position of shaft
int epos = 0; //electrical position of shaft
int torque = 75; //output torque
int directn;
int direct = 0; //vector for Magnetic field orientation
int setpoint = 0; //variable for storing desired position
int error = 0; //variable for calculating the error for Proportional Control
int increment = 1;
bool debug = true;
bool openloop = true;

void setup() {
  Serial.begin(9600);

  // shift frequency to ultrasonic
  //TCCR1B = TCCR1B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 9 and 10 (not used)
  TCCR2B = TCCR2B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 11 and 3
  TCCR0B = TCCR0B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 5 and 6
  ICR1 = 255 ; // 8 bit resolution

  // prepare ports
  pinMode(potPin, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);

  // Angle
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  digitalWrite(CSN, HIGH);
  SPI.begin();

  // uncomment if you want generate lookup table by arduino
  //    for(int i=1;i<=sineArraySize;i++){
  //      float tmp = sin(2*3.1416/sineArraySize*i)*255; // calculates every single PWM value in range 0...255 of a period of 360° in rad
  //      pwmSin[i] = (int)tmp;
  //      Serial.println(pwmSin[i]);
  //    }
  //    delay(1000);
  
  sineArraySize = sizeof(pwmSin) / sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0; // rotor alignment
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;
  sineArraySize--; // Convert from array Size to last PWM array number

}

void loop() {
  // 1. uncomment to find rotor alignment -> start serial plotter
  //MoveOpenLoop();
  //PrintDebugInfo();

  // 2. uncomment to control motor in closed loop
  MoveClosedLoop();
  PrintControlInfo();
}

/////////////// FUNCTIONS /////////////////////////////////////

void MoveOpenLoop() {
  int potState = analogRead(potPin);
  potState = map(potState, 0, 1023, 20, 1000);
  pos = GetAngle();
  analogWrite(motorPin1, pwmSin[currentStepA]*torque / 100);
  analogWrite(motorPin2, pwmSin[currentStepB]*torque / 100);
  analogWrite(motorPin3, pwmSin[currentStepC]*torque / 100);

  currentStepA = currentStepA + increment;
  currentStepB = currentStepB + increment;
  currentStepC = currentStepC + increment;

  //Check for lookup table overflow and return to opposite end if necessary
  if (currentStepA > sineArraySize)  currentStepA = 0;
  if (currentStepA < 0)  currentStepA = sineArraySize;

  if (currentStepB > sineArraySize)  currentStepB = 0;
  if (currentStepB < 0)  currentStepB = sineArraySize;

  if (currentStepC > sineArraySize)  currentStepC = 0;
  if (currentStepC < 0) currentStepC = sineArraySize;

  /// Control speed by this delay
  delayMicroseconds(potState);
}

void MoveClosedLoop()
{
  const int offset = 51; //**ADJUST THIS VALUE** OFFSET needed for Syncronization, unique to each motor as it depends on manufacturing.

  setpoint = analogRead(potPin); //read potentiometer
  setpoint = map(setpoint, 0, 1023, 360, 0); //scale to 0-360 degrees
  //setpoint = 60;
  
  float temp = 0;
  for(int i=0;i<10;i++){
    temp = temp + GetAngle();
  }
  pos = temp/10;
  pos = GetAngle();

  //pos=pulseIn(encoder,HIGH);//read encoder pulse
  epos = map(pos, 0, 360, 0, (90 * P)); //scale shaft's encoder position range to electrical positions multiple of 48*Number of poles in motor.
  epos = constrain(epos, 0, 90 * P); //constraint the values
  pos = map(epos, 0, (90 * P), 0, 360); //translate the electrical position to shaft position in degrees (0-360)

  // Proportional control
  error = pos - setpoint; //calculate error
  if (error < 0) {
    directn = 26; //orient magnetic field vector 90 degrees ahead
  }
  if (error > 0) {
    directn = 64; //orient magnetic field vector 90 degrees behind
  }

  torque = abs(error * K); //define the magnitude of the reaction of the motor, or the Proportional Gain. Small torque for small errors, large torque for large errors. Adjust the Value of K to better match the response you want from the motor, try 1,2,5,8,10.
  torque = constrain(torque, 10, G); //limit maximum torque 0-100 %, you can limit the maximum torque you want the motor to have i.e. instead of 100, use 60 for 60% of the maximum torque available, motor runs slower and cooler ;)

  currentStepA = epos + directn + offset;  //directn is used to define the rotation the motor sould turn.
  currentStepB = currentStepA + 30; //add 120 deg of phase to StepA position.
  currentStepC = currentStepA + 60; //add 240 deg of phase to StepA position.

  currentStepA = currentStepA % 90; //I used remainder operation or modulo to "wrap" the values between 0 and 47
  currentStepB = currentStepB % 90;
  currentStepC = currentStepC % 90;

  //write PWM values to ports
  analogWrite(motorPin1, pwmSin[currentStepA]*torque / 100);
  analogWrite(motorPin2, pwmSin[currentStepB]*torque / 100);
  analogWrite(motorPin3, pwmSin[currentStepC]*torque / 100);
}

float GetAngle() {
  unsigned int angle;
  float AngOut;
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

  //Send the Command Frame
  digitalWrite(CSN, LOW);
  delayMicroseconds(1);
  SPI.transfer16(0xFFFF);
  digitalWrite(CSN, HIGH);

  //Read data frame
  digitalWrite(CSN, LOW);
  delayMicroseconds(1);
  angle = SPI.transfer16(0xC000);
  digitalWrite(CSN, HIGH);
  SPI.endTransaction;
  angle = (angle & (0x3FFF));
  AngOut = 360 * float(angle) / 16384;
  return (int)AngOut; // temporary typecast
}

void PrintDebugInfo() {
  Serial.print("DATA,");
  Serial.print(currentStepA);
  Serial.print(",");
  Serial.print(pwmSin[currentStepA]);
  Serial.print(",");
  Serial.print(pwmSin[currentStepB]);
  Serial.print(",");
  Serial.print(pwmSin[currentStepC]);
  Serial.print(",");
  Serial.print(pos);
  Serial.print(",");
  Serial.println(setpoint);
}

void PrintControlInfo() {
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(pos);
  Serial.print(",");
  Serial.println(torque);
}
