#include <Ps3Controller.h>

byte header_byte = 0x11;
byte command_digital = 0x12;
byte command_analog = 0x13;
byte command_connect = 0x14;
byte left_stick_data = 0x01;
byte right_stick_data = 0x02;
byte end_byte = 0x15;
byte signal_to_send[4] = {command_digital, 0, 0, 0xff};

byte* send_digital_signal(byte data) {
	byte* result = new byte[5];
	result[0] = header_byte;
	result[1] = header_byte;
	result[2] = command_digital;
	result[3] = data;
	result[4] = end_byte;
	return result;
}

byte* send_analog_signal(byte data0, byte data1, byte data2) {
	byte* result = new byte[7];
	result[0] = header_byte;
	result[1] = header_byte;
	result[2] = command_analog;
	result[3] = data0;
	result[4] = data1;
	result[5] = data2;
	result[6] = end_byte;
	return result;
}

void notify()
{

    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.cross ) {
      byte* signal_to_send = send_digital_signal(0x58);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.cross ) {
      byte* signal_to_send = send_digital_signal(0x78);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }

    if( Ps3.event.button_down.square ) {
      byte* signal_to_send = send_digital_signal(0x53);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.square ) {
      byte* signal_to_send = send_digital_signal(0x73);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }

    if( Ps3.event.button_down.triangle ) {
      byte* signal_to_send = send_digital_signal(0x54);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.triangle ) {
      byte* signal_to_send = send_digital_signal(0x74);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.circle ) {
      byte* signal_to_send = send_digital_signal(0x43);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.circle ) {
      byte* signal_to_send = send_digital_signal(0x63);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }   

    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up ) {
      byte* signal_to_send = send_digital_signal(0x55);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.up ) {
      byte* signal_to_send = send_digital_signal(0x75);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.right ) {
      byte* signal_to_send = send_digital_signal(0x52);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }
   
    if( Ps3.event.button_up.right ) {
      byte* signal_to_send = send_digital_signal(0x72);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }        

    if( Ps3.event.button_down.down ) {
      byte* signal_to_send = send_digital_signal(0x44);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.down ) {
      byte* signal_to_send = send_digital_signal(0x64);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }   

    if( Ps3.event.button_down.left ) {
      byte* signal_to_send = send_digital_signal(0x4c);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }
        
    if( Ps3.event.button_up.left ) {
      byte* signal_to_send = send_digital_signal(0x6c);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
    }   
    
    //---------------- Analog stick value events ---------------
   if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 1 ){
	byte* signal_to_send = send_analog_signal(left_stick_data, Ps3.data.analog.stick.lx, Ps3.data.analog.stick.ly);
	Serial.write(signal_to_send, 7);
	Serial.flush();
	return;
    }

   if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 1 ){
	byte* signal_to_send = send_analog_signal(right_stick_data, Ps3.data.analog.stick.rx, Ps3.data.analog.stick.ry);
	Serial.write(signal_to_send, 7);
	Serial.flush();
	return;
   }

    if(Ps3.event.button_down.start){
      byte* signal_to_send = send_digital_signal(0x57);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
   }

    if(Ps3.event.button_down.select){
      byte* signal_to_send = send_digital_signal(0x59);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
   }

    if(Ps3.event.button_down.l1){
      byte* signal_to_send = send_digital_signal(0x60);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
   }

    if(Ps3.event.button_down.l2){
      byte* signal_to_send = send_digital_signal(0x61);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
   }

    if(Ps3.event.button_down.r1){
      byte* signal_to_send = send_digital_signal(0x62);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
   }

    if(Ps3.event.button_down.r2){
      byte* signal_to_send = send_digital_signal(0x65);
      Serial.write(signal_to_send, 5);
      Serial.flush();
      return;
   }
}

void onConnect(){
    byte signal_to_send[4] = {header_byte, header_byte, command_connect, end_byte};
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
