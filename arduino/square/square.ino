#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <SPI.h>
#include <Encoder_Buffer.h>


int eStopPin = 7;

unsigned int vin = A0;

SoftwareSerial mcSerial(NOT_A_PIN, 6);
Sabertooth ST1(128, mcSerial);
Sabertooth ST2(129, mcSerial);

Encoder_Buffer Encoder1(46);
Encoder_Buffer Encoder2(47);
Encoder_Buffer Encoder3(48);
Encoder_Buffer Encoder4(49);

float T=1; //period of square
char A=1; //amplitude of square

float Tm=T*1000;

void setup() {
  // put your setup code here, to run once:
  mcSerial.begin(9600);
  Sabertooth::autobaud(mcSerial);
  delay(500);

  pinMode(vin, INPUT);
  pinMode(eStopPin,OUTPUT); digitalWrite(eStopPin, HIGH);
  
  Serial.begin(9600);
  SPI.begin();
  Encoder1.initEncoder();
  Encoder2.initEncoder();
  Encoder3.initEncoder();
  Encoder4.initEncoder();
}
int starting_time=-1;
boolean b_stop=0;

void loop() {
  if (!b_stop){
    if (starting_time == -1){
      starting_time=millis();
    }
    int tr=millis()-starting_time;
    if (tr<5000){
      float t = (tr % (int)Tm);
      char motorFR;
      if (t>(Tm/2)) motorFR=A;
      else motorFR=-A;
      //if (analogRead(vin)<1023)motorFR=0;
      
      //if (driveVal>0) driveVal=.1;
      Serial.print((int)motorFR);
      Serial.print("\t");
    
      long encoder1Reading =  Encoder1.readEncoder(); //FL
      long encoder2Reading = -Encoder2.readEncoder(); //FR
      long encoder3Reading =  Encoder3.readEncoder(); //BL
      long encoder4Reading = -Encoder4.readEncoder(); //BR
      
      char m1=-motorFR;
      char m2= motorFR;
      char m3= motorFR;
      char m4=-motorFR;
    
      ST1.motor(2,-m1);  // FL (is negative)
      ST1.motor(1,-m2);  // FR (is negative)
      ST2.motor(1, m3);   // BL
      ST2.motor(2, m4);   // BR
      
      Serial.print(millis());
      Serial.print("\t");
      Serial.print(0);
      Serial.print("\t");
      Serial.print(encoder1Reading);
      Serial.print("\t");
      Serial.print(encoder2Reading);
      Serial.print("\t");
      Serial.print(encoder3Reading);
      Serial.print("\t");
      Serial.print(encoder4Reading);
      
      Serial.print("\n");
    }
    else {
      ST1.motor(2,0);  // FL (is negative)
      ST1.motor(1,0);  // FR (is negative)
      ST2.motor(1,0);  // BL
      ST2.motor(2,0);  // BR
      A=A+1;
      if (A==128){
        Serial.println("END");
        b_stop=1;
      }
      else
        Serial.println("NEXT");
      
      delay(5000);
      starting_time=millis();
    }
  }
}
