#include <Ps3Controller.h>

byte command_digital = 0x10;
byte command_analog = 0x11;
byte command_connect = 0x12;

void notify()
{
     /* byte signal[4] = {command_digital, 0, 0, 0xff}; */
     /* Serial.write(signal, 4); */
     byte signal[4] = {command_digital, 0, 0, 0xff};

    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.cross ) {
      signal[2] = 0x58; // 'X'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.cross ) {
      signal[2] = 0x78; // 'x'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }

    if( Ps3.event.button_down.square ) {
      signal[2] = 0x53; // 'S'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.square ) {
      signal[2] = 0x73; // 's'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }

    if( Ps3.event.button_down.triangle ) {
      signal[2] = 0x54; // 'T'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.triangle ) {
      signal[2] = 0x74; // 't'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.circle ) {
      signal[2] = 0x43; // 'C'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.circle ) {
      signal[2] = 0x63; // 'c'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }   

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up ) {
      signal[2] = 0x55; // 'U'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.up ) {
      signal[2] = 0x75; // 'u'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.right ) {
      signal[2] = 0x52; // 'R'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }
   
    if( Ps3.event.button_up.right ) {
      signal[2] = 0x72; // 'r'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }        

    if( Ps3.event.button_down.down ) {
      signal[2] = 0x44; // 'D'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.down ) {
      signal[2] = 0x64; // 'd'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.left ) {
      signal[2] = 0x4c; // 'L'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.left ) {
      signal[2] = 0x6c; // 'l'
      Serial.write(signal, 4);
      Serial.flush();
      return;
    }   
    
    //---------------- Analog stick value events ---------------
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
       signal[0] = command_analog;
       signal[1] = Ps3.data.analog.stick.lx;
       signal[2] = Ps3.data.analog.stick.ly;
       Serial.write(signal, 4);
       Serial.flush();
       return;
    }

   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
       signal[0] = command_analog;
       signal[1] = Ps3.data.analog.stick.rx;
       signal[2] = Ps3.data.analog.stick.ry;
       Serial.write(signal, 4);
       Serial.flush();
       return;
   }
}

void onConnect(){
    byte signal[4] = {command_connect, 0, 0, 0xff};
    Serial.write(signal, 4);
    Serial.flush();
    Ps3.setPlayer(1);
}

void setup()
{
    Serial.begin(57600);

    Ps3.attach(notify);
    Ps3.attachOnConnect(onConnect);
    Ps3.begin("b0:a7:32:db:3d:f6");
}

void loop()
{
   delay(20000);
}
