/* USB to Serial - Teensy becomes a USB to Serial converter
   http://dorkbotpdx.org/blog/paul/teensy_as_benito_at_57600_baud

   You must select Serial from the "Tools > USB Type" menu

   This example code is in the public domain.
*/

// set this to the hardware serial port you wish to use
#include "SBUS.h"
#define HWSERIAL Serial1
#define CONTROL_NO_CONTROL 0


unsigned long baud   = 152000;
const int reset_pin  = 4;
const int led_pin    = 13;
int RC_cmd           = 172;
bool RC              = false;
uint8_t rc_cmd[20];
bool zoomin          = false;
bool zoomout         = false;                

SBUS x8r_gimbal(Serial2);
SBUS x8r_camera(Serial3);
uint16_t gimbal[16];
uint16_t camera[18];
bool failSafe;  
bool lostFrame;

void setup()
{
  pinMode(led_pin, OUTPUT);
  pinMode(reset_pin, OUTPUT);
  Serial.begin(baud);  // USB, communication to PC or Mac
  HWSERIAL.begin(baud); // communication to hardware serial
  x8r_gimbal.begin();
  x8r_camera.begin();
  

  for(uint8_t i = 0; i<sizeof(rc_cmd);i++)
    rc_cmd[i] = 0;
  rc_cmd[0]  = 62;
  rc_cmd[1]  = 67;
  rc_cmd[2]  = 15;
  rc_cmd[3]  = 82;

  camera[0]  = 650;
  camera[2]  = 1024;
  camera[8]  = camera[9]  = camera[10] = camera[11] = camera[12] = camera[14] = 1024;
  camera[1]  = camera[3]  = camera[7]  = camera[13] = camera[15] = camera[16] = camera[17] = 0;
  camera[4]  = 1320;//760 - 25 FPS;// 1320 - 50 fps
  camera[5]  = 1200;//1200 - Prores422;//
  camera[6]  = 352;
  x8r_camera.write(&camera[0]);
  Serial3.flush();
 }


long led_on_time=0;
byte buffer[80];
unsigned char prev_dtr = 0;
double push_button = 0;
double duration = 400;
bool out = false;

void loop(){
  unsigned char dtr;
  int rd, wr, n;                                                         // check if any data has arrived on the USB virtual serial port
  rd = Serial.available();
  if (rd > 0) {                                                          // check if the hardware serial port is ready to transmit
    wr = HWSERIAL.availableForWrite();
    if (wr > 0) {
      if (rd > wr) rd = wr;                                              // compute how much data to move, the smallest of rd, wr and the buffer size
      if (rd > 80) rd = 80;
      n = Serial.readBytes((char *)buffer, rd);                          // read data from the USB port
      if (!RC){
        if (buffer[0]== 62)
          HWSERIAL.write(buffer, n);                                     // write it to the hardware serial port
        else {
          for(byte i = 0; i < sizeof(camera)*2/sizeof(camera[0]); i+=2)
           camera[i/2] = buffer[i] << 8 | buffer[i+1];
//          camera[4]  = 1320;//760 - 25 FPS;// 1320 - 50 fps
//          camera[5]  = 1200;//1200 - Prores422;//
//          camera[6]  = 352;
          push_button = millis() + duration;                             //time of actuation of push button
          x8r_camera.write(&camera[0]);
          out = true;
          Serial3.flush();
        }
        
        if(millis() > push_button && out){
          camera[1]  = camera[3]  = camera[7]  = camera[13] = camera[15] = camera[16] = camera[17] = 0;
          camera[2]  = 1024;
          camera[8]  = camera[9]  = camera[10] = camera[11] = camera[12] = camera[14] = 1024;
          x8r_camera.write(&camera[0]);
          out = false;
          Serial3.flush();
        }else if(millis() < push_button && out)
           digitalWrite(led_pin, LOW);
      }
    }
  }
  
  rd = HWSERIAL.available();                      // check if any data has arrived on the hardware serial port
  if (rd > 0) {                                   // check if the USB virtual serial port is ready to transmit 
    wr = Serial.availableForWrite();
    if (wr > 0) {
      if (rd > wr) rd = wr;                       // compute how much data to move, the smallest
      if (rd > 80) rd = 80;                       // of rd, wr and the buffer size      
      n = HWSERIAL.readBytes((char *)buffer, rd); // read data from the hardware serial port
    if (!RC)
        Serial.write(buffer, n);                  // write it to the USB port
    }
  }

  dtr = Serial.dtr();                             // check if the USB virtual serial port has raised DTR
  if (dtr && !prev_dtr) {
    digitalWrite(reset_pin, LOW);
    delayMicroseconds(250);
    digitalWrite(reset_pin, HIGH);
  }
  prev_dtr = dtr;
  
  if (Serial.baud() != baud){                    // check if the USB virtual serial wants a new baud rate
    baud = Serial.baud();
    if (baud == 57600)                           // This ugly hack is necessary for talking to the arduino bootloader, which actually communicates at 58824 baud (+2.1% error). Teensyduino will configure the UART for the closest baud rate, which is 57143 baud (-0.8% error).  Serial communication can tolerate about 2.5% error, so the combined error is too large.  Simply setting the baud rate to the same as arduino's actual baud rate works.
      HWSERIAL.begin(58824);
    else
      HWSERIAL.begin(baud);
  }
  
  Serial1.flush();
  Serial.flush();
  if(x8r_gimbal.read(&gimbal[0], &failSafe, &lostFrame)){
    if(RC_cmd != gimbal[2]&& (gimbal[2] == 172 || gimbal[2] == 1811)){
       RC = !RC;
       RC_cmd = gimbal[2];
       camera[1]  = camera[3]  = camera[7]  = camera[13] = camera[15] = camera[16] = camera[17] = 0;
       camera[2]  = 1024;
       camera[8]  = camera[9]  = camera[10] = camera[11] = camera[12] = camera[14] = 1024;
        
       x8r_camera.write(&camera[0]);
       Serial3.flush();

       if(RC)
        HWSERIAL.write(rc_cmd, (int)sizeof(rc_cmd)); //set SBGC command to RC  
    }
      
    if(RC){
      digitalWrite(led_pin, LOW);
      camera[0] = 0.82*(gimbal[6]-172)+352;
      if(gimbal[4] > 1050 || gimbal[4] < 900)
       camera[2] = 0.82*(gimbal[4]-172)+352;
      else
        camera[2] = 1024;
   
      x8r_camera.write(&camera[0]);   
      Serial3.flush();
      x8r_gimbal.write(&gimbal[0]);
      Serial2.flush();
    }
  }  
  
}
