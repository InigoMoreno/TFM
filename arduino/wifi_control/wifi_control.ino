#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <SPI.h>
#include <Encoder_Buffer.h>



bool show_esp=true;

 
int LEDPIN = 13;
int strafeSignal15Vpin = 5;
int eStopPin = 7;


SoftwareSerial mcSerial(NOT_A_PIN, 6);
Sabertooth ST1(128, mcSerial);
Sabertooth ST2(129, mcSerial);

void setup() {
  // put your setup code here, to run once:
  mcSerial.begin(9600);
  Sabertooth::autobaud(mcSerial);
  delay(500);
  
  pinMode(eStopPin,OUTPUT); digitalWrite(eStopPin, HIGH);

  Serial.begin(9600);     // communication with the host computer
  while (!Serial)   { ; }

  // Start the software serial for communication with the ESP8266
  Serial3.begin(9600); 
  while (!Serial3) { ; }
  Serial3.println("AT+CWMODE=2\r");
  delay_show();
  Serial3.println("AT+CWSAP=\"esp_inigo\",\"moreno14\",5,3\r");
  delay_show();
  Serial3.println("AT+CIFSR\r");
  delay_show();
  Serial3.println("AT+CIPMUX=1\r");
  delay_show();
  Serial3.println("AT+CIPSERVER=1,80\r");
  delay_show();
  
  Serial.println("");
  Serial.println("Remember to to set Both NL & CR in the serial monitor.");
  Serial.println("Ready");
  Serial.println("");    
  
}
unsigned long data;
unsigned long check = ((unsigned long int) 'D') | ((unsigned long int) 'P')<<8 | ((unsigned long int) 'I')<<16 | ((unsigned long int) '+')<<24;

void loop() {
  if ( Serial.available() )       {  
    Serial3.write(Serial.read());
  }
  // listen for communication from the ESP8266 and then write it to the serial monitor
  if ( Serial3.available() )   {  
    byte b = Serial3.read();
    data= (data<<8) | b;
    if(show_esp) Serial.write(b); 
    if (data==check){                                         //+IPD |,0,4:1234
      Serial.println("!");
      char trash[10];
      int size_trash = Serial3.readBytes(trash,1);            //,    |0,4:1234
      size_trash = Serial3.readBytesUntil(',',trash,10);      //0,   |4:1234
      int size_data=Serial3.parseInt();                       //4    |:1234
      size_trash = Serial3.readBytes(trash,1);                //:    |1234
      char ab[size_data];
      if (Serial3.readBytes(ab,size_data) == size_data){      //1234
        if (size_data==4){
          digitalWrite(eStopPin, HIGH);
        
          ST1.motor(2,-((int)ab[0]));  // FL (is negative)
          ST1.motor(1,-((int)ab[1]));  // FR (is negative)
          ST2.motor(1, ((int)ab[2]));  // BL
          ST2.motor(2, ((int)ab[3]));  // BR
        }
        for (int i=0; i<size_data; i++){
          Serial.print(i);
          Serial.print(":");
          Serial.println((byte) ab[i]);
        }
      }
    }
  }
}


void delay_show(){
    delay(100); 
    while( Serial3.available() )   { 
      byte b = Serial3.read();
      if(show_esp)Serial.write(b); 
    }
}
