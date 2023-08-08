#include <Ps3Controller.h>

byte header_byte = 0x11;
byte command_digital = 0x12;
byte command_analog = 0x13;
byte command_connect = 0x14;
byte signal_to_send[4] = {command_digital, 0, 0, 0xff};

void notify()
{

    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.cross ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x58; // 'X'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.cross ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x78; // 'x'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }

    if( Ps3.event.button_down.square ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x53; // 'S'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.square ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x73; // 's'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }

    if( Ps3.event.button_down.triangle ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x54; // 'T'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.triangle ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x74; // 't'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.circle ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x43; // 'C'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.circle ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x63; // 'c'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }   

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x55; // 'U'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.up ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x75; // 'u'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.right ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x52; // 'R'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }
   
    if( Ps3.event.button_up.right ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x72; // 'r'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }        

    if( Ps3.event.button_down.down ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x44; // 'D'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.down ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x64; // 'd'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.left ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x4c; // 'L'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.left ) {
      signal_to_send[0] = command_digital;
      signal_to_send[1] = 0;
      signal_to_send[2] = 0x6c; // 'l'
      Serial.write(signal_to_send, 4);
      Serial.flush();
      return;
    }   
    
    //---------------- Analog stick value events ---------------
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
       signal_to_send[0] = command_analog;
       signal_to_send[1] = Ps3.data.analog.stick.lx;
       signal_to_send[2] = Ps3.data.analog.stick.ly;
       Serial.write(signal_to_send, 4);
       Serial.flush();
       return;
    }

   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
       signal_to_send[0] = command_analog;
       signal_to_send[1] = Ps3.data.analog.stick.rx;
       signal_to_send[2] = Ps3.data.analog.stick.ry;
       Serial.write(signal_to_send, 4);
       Serial.flush();
       return;
   }

    if(Ps3.event.button_up.start){
       Serial.write('S');
       Serial.flush();
       return;
   }
}

void onConnect(){
    byte signal_to_send[4] = {command_connect, 0, 0, 0xff};
    Serial.write(signal_to_send, 4);
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
