#include <stlport.h>
#include <Eigen30.h>


#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <SPI.h>
#include <Encoder_Buffer.h>


//Somehow eigen redefines min and max
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))


int strafePinRC = 13, drivePinRC = 12, turnPinRC=11;
int strafeSignal15Vpin = 5;
int eStopPin = 7;


float pulseDeadbandWidth = 50;

SoftwareSerial mcSerial(NOT_A_PIN, 6);
Sabertooth ST1(128, mcSerial);
Sabertooth ST2(129, mcSerial);

float T=30; //period of circle
float v=5; //speed of circle (cm/s)
float L=16.5*2.54; //Length of robot (same as width)
float r=(6/2-.75/2)*2.54; //radius of wheels minus radius of cyllinders
Eigen::Matrix<float,4,3> K;

void setup() {
  // put your setup code here, to run once:
  mcSerial.begin(9600);
  Sabertooth::autobaud(mcSerial);
  delay(500);

  pinMode(eStopPin,OUTPUT); digitalWrite(eStopPin, HIGH);
  pinMode(strafeSignal15Vpin,OUTPUT); digitalWrite(strafeSignal15Vpin, HIGH);
  
  Serial.begin(9600);
  SPI.begin();

  K << 1, -1, -L,
       1,  1,  L, 
       1,  1, -L, 
       1, -1,  L;
  K=K/r;
}

void loop() {
  /*
  DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH);
  TURN_PULSE_WIDTH = pulseIn(turnPinRC, HIGH);
  STRAFE_PULSE_WIDTH = pulseIn(strafePinRC,HIGH);

  if(DRIVE_PULSE_WIDTH < 500 || TURN_PULSE_WIDTH < 500 || STRAFE_PULSE_WIDTH < 500) {
    digitalWrite(eStopPin, LOW);
    return;
  }
  
  
  // otherwise, unthrow estop
  digitalWrite(eStopPin, HIGH);

  float driveVal = convertRCtoFloat(DRIVE_PULSE_WIDTH,drivePulseHigh,drivePulseLow);
  float turnVal = convertRCtoFloat(TURN_PULSE_WIDTH,turnPulseHigh,turnPulseLow);
  float strafeVal = convertRCtoFloat(STRAFE_PULSE_WIDTH,strafePulseHigh,strafePulseLow);
  */
  
  float Tm=T*1000;
  float t = (millis() % (int)Tm);
  
  Eigen::Vector3f X;
  X<< 20*sin(3*2*PI*t/Tm), 10*cos(10*2*PI*t/Tm), 0;

  Eigen::Vector4f U;
  U=K*X;
  
  U=U*60/(2*PI); //rad/s to rpm
  U=U/122; //rpm to input

  /*
  if (driveVal>0) driveVal=.1;
  char motorFR = convertFloatToByte(driveVal);
  */
  ST1.motor(2,-convertFloatToByte(U(0)));    // FL (is negative)
  ST1.motor(1,-convertFloatToByte(U(1)));  // FR (is negative)
  ST2.motor(1, convertFloatToByte(U(2)));  // BL 
  ST2.motor(2, convertFloatToByte(U(3)));  // BR
  
  
  // convert the [-1,1] values to bytes in range [-127,127] for sabertooths
  /*char motorFR = -1*convertFloatToByte(driveVal + turnVal + strafeVal);
  char motorRR = convertFloatToByte(driveVal + turnVal - strafeVal);
  char motorFL = -1*convertFloatToByte(driveVal - turnVal - strafeVal);
  char motorRL = convertFloatToByte(driveVal - turnVal + strafeVal);
  */
  // command motors
  //ST1.motor(1,motorFL); ST1.motor(2,motorFR);
  //ST2.motor(1,motorRR); ST2.motor(2,motorRL);


  Serial.print("t: ");
  Serial.print(t/Tm);
  Serial.print("\t");
  for(int i=0; i<4; i++){
    Serial.print(U(i));
    Serial.print("\t");
  }
  
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
