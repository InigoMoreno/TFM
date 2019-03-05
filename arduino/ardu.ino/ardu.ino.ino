#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <SPI.h>
#include <Encoder_Buffer.h>

int strafePinRC = 13, drivePinRC = 12, turnPinRC=11;
int strafeSignal15Vpin = 5;
int eStopPin = 7;

float drivePulseLow = 1130, drivePulseHigh=1930;
float turnPulseLow = 1115, turnPulseHigh=1903;
float strafePulseLow = 1118, strafePulseHigh=1918;

float pulseDeadbandWidth = 0;

SoftwareSerial mcSerial(NOT_A_PIN, 6);
Sabertooth ST1(128, mcSerial);
Sabertooth ST2(129, mcSerial);

Encoder_Buffer Encoder1(46);
Encoder_Buffer Encoder2(47);
Encoder_Buffer Encoder3(48);
Encoder_Buffer Encoder4(49);

void setup() {
  // put your setup code here, to run once:
  mcSerial.begin(9600);
  Sabertooth::autobaud(mcSerial);
  delay(500);
  

  pinMode(eStopPin,OUTPUT); digitalWrite(eStopPin, LOW);
  pinMode(strafeSignal15Vpin,OUTPUT); digitalWrite(strafeSignal15Vpin, HIGH);
  
  Serial.begin(9600);
  SPI.begin();
  Encoder1.initEncoder();
  Encoder2.initEncoder();
  Encoder3.initEncoder();
  Encoder4.initEncoder();
}

void loop() {
  unsigned long DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH);
  unsigned long TURN_PULSE_WIDTH = pulseIn(turnPinRC, HIGH);
  unsigned long STRAFE_PULSE_WIDTH = pulseIn(strafePinRC,HIGH);

  if(DRIVE_PULSE_WIDTH < 500 || TURN_PULSE_WIDTH < 500 || STRAFE_PULSE_WIDTH < 500) {
    digitalWrite(eStopPin, LOW);
    return;
  }
  
  
  // otherwise, unthrow estop
  digitalWrite(eStopPin, HIGH);

  float driveVal = convertRCtoFloat(DRIVE_PULSE_WIDTH,drivePulseHigh,drivePulseLow);
  float turnVal = convertRCtoFloat(TURN_PULSE_WIDTH,turnPulseHigh,turnPulseLow);
  float strafeVal = convertRCtoFloat(STRAFE_PULSE_WIDTH,strafePulseHigh,strafePulseLow);

  //*
  //if (driveVal>0) driveVal=.1;

  
  long encoder1Reading =  Encoder1.readEncoder(); //FL
  long encoder2Reading = -Encoder2.readEncoder(); //FR (is negative)
  long encoder3Reading =  Encoder3.readEncoder(); //BL
  long encoder4Reading = -Encoder4.readEncoder(); //BR (is negative)
  
  
  /*
  char motorFR = convertFloatToByte(driveVal);
  Serial.print((int)motorFR);
  Serial.print("\t");
  ST1.motor(2,-motorFR);  // FL (is negative)
  ST1.motor(1,-motorFR);  // FR (is negative)
  ST2.motor(1, motorFR);  // BL
  ST2.motor(2, motorFR);  // BR
  
  */
   //convert the [-1,1] values to bytes in range [-127,127] for sabertooths
  char motorFR = -1*convertFloatToByte(driveVal + turnVal + strafeVal);
  char motorRR = convertFloatToByte(driveVal + turnVal - strafeVal);
  char motorFL = -1*convertFloatToByte(driveVal - turnVal - strafeVal);
  char motorRL = convertFloatToByte(driveVal - turnVal + strafeVal);
  
  // command motors
  ST1.motor(1,motorFL); ST1.motor(2,motorFR);
  ST2.motor(1,motorRR); ST2.motor(2,motorRL);
  
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(driveVal);
  Serial.print("\t");
  Serial.print(encoder1Reading);
  Serial.print("\t");
  Serial.print(encoder2Reading);
  Serial.print("\t");
  Serial.print(encoder3Reading);
  Serial.print("\t");
  Serial.print(encoder4Reading);
  
  /*Serial.print(driveVal);
  Serial.print("\t");
  Serial.print(turnVal);
  Serial.print("\t");
  Serial.print(strafeVal);*/
  
  Serial.print("\n");
}

float convertRCtoFloat(unsigned long pulseWidth,unsigned long pulseHigh, unsigned long pulseLow){
  float W = (float) pulseWidth;
  float H = (float) pulseHigh;
  float L = (float) pulseLow;
  float M = (H+L)/2;
  float DB = (float) pulseDeadbandWidth;
  if (W > (M-DB/2) && W < (M+DB/2)){
    W = M;
  }
  
  float checkVal = (W-L)/(H-L);
  checkVal=2*checkVal-1;
  checkVal = min(max(checkVal,-1),1);
  return checkVal;
}

char convertFloatToByte(float value){
  float checkVal = (float)127*value;
  checkVal=min(max(checkVal,-127),127);
  return (char)(checkVal);
}

void serialEvent(){
//statementsz
}
