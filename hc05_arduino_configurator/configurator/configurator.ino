#include <SoftwareSerial.h>

const int arduino_rx = 5;
const int arduino_tx = 6;
SoftwareSerial mySerial (arduino_rx, arduino_tx);
 
void setup()  {
  pinMode(arduino_rx, INPUT);
  pinMode(arduino_tx, OUTPUT);

  Serial.begin(9600);       
  mySerial.begin(38400);    
  Serial.println( "<<< Start! >>>");
  mySerial.println("AT");    
}

void loop() {
  if (mySerial.available()) {
    char c = mySerial.read();      
    Serial.print(c);   }           
  if (Serial.available()) {
    char c = Serial.read();        
    mySerial.write(c); }           
}
