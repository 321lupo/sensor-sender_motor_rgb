#include <Bounce.h>

#include <autotune.h>
#include <microsmooth.h>

#include <MIDI.h>
#include <Wire.h>
const int channel = 1;

#include "SmoothPin.h"

#define NUM 3
#define MIDI_CHAN 1
#define DEBUG 1

#define GESTURE_START 0
#define GESTURE_N 20
#define FINGER_START 20
#define FINGER_N 7

#define CONTROL_N 28

#define GREEN 3
#define BLUE 4
#define RED 5

//sensor values
int16_t accel_x, accel_y, accel_z;
SmoothPin fingers[FINGER_N];
int ir; //TODO

int bendPins[] = {A1, A2, A3, A6, A7, A8, A9}; // the pins for each of the finger resistors

/*
 * Button setup: define pins for all of the action buttons
 * buttons are pins 6, 7, 10, 11
 */
#define XYPIN 7
#define MODEPIN 6

Bounce XYLockButton = Bounce(XYPIN, 10);
Bounce ModeButton = Bounce(MODEPIN, 10);

uint8_t xy_lock = 0;  // Store whether or not we're in XY lock.
                      // This is so we can connect with Ableton
                      // 0 = send both, 1 = send X, 2 = send y

//control values
uint8_t controls[CONTROL_N];

//accel init
#define accel_module (0x53)    //ACCELEROMETER SETUP
byte values [6];
char output [512];

//vibration motor
const int motorPin = 9;

//Which mode we are in: 0 is gesture
//mode 1 is accel -> note
#define MODES 4
int mode = 0;

// timing details: we want to send midi at a 50hz clock
// that means everz 20 ms
int lastMidiSend = 0;
#define MIDI_INTERVAL 20

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once
  initFingers();
  initAccel();
  initButtons();
  
  pinMode(motorPin, OUTPUT);
  
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(RED, OUTPUT);
  
  Serial.println("Init done");
  
}

void loop() {  
  // Read all sensors
  readAccel();
  readFingers();
  readButtons();
  //Serial.println("loop!");
  if (ModeButton.fallingEdge()){
    mode++;
    mode %= MODES;
    Serial.println("mode switch");
    
    //do mode switch reset stuff here
    xy_lock = 0;
  }
  
  //Do MIDI communication
  int t = millis();
  if (t - lastMidiSend >= MIDI_INTERVAL) {
    sendOutput();
    lastMidiSend = t;
  }  
 
  // clean up: clear midi buffer
  while (usbMIDI.read()) ; // read and discard any incoming MIDI messages
  
}

void sendOutput() {
   // timing code: if it's been more than 20ms since our last
  // send, then send again
  Serial.println("Mode " + (String) mode);
  
  if (mode == 0){ // gesture selects the control channel
    // calibration mode: xy lock
    if (XYLockButton.update() && XYLockButton.fallingEdge()){
      xy_lock++;
      xy_lock %= 3;
    }
    
    // TODO: calibrate accel
    
    
    
    // Which control number we should send.
    // This code will read the states of all of the fingers, and decide
    // which, if any, controls should be sent
    int xcontrol = -1;
    int ycontrol = -1;
    
    int f[FINGER_N];
    Serial.print("{ ");
    for (int i = 0; i < FINGER_N; i++){
      f[i] = fingers[i].value;
      Serial.print(f[i]);
      Serial.print(", ");
    }
    Serial.print("}");
    Serial.println();
    
    
    if(gesture(f, (const int[]){ 295, 569, 871, 720, 804, 409, 649,  }))               {xcontrol = 1; ycontrol=2; } //pistol
    else if( gesture(f,  (const int[])   { 859, 663, 887, 684, 826, 361, 682, }    )) {xcontrol = 3; ycontrol=4;} //fist
    else if( gesture(f,  (const int[])   { 303, 350, 334, 378, 304, 319, 368, }    )) {xcontrol = 5; ycontrol=6;} //flat
    else if( gesture(f,  (const int[])   { 809, 318, 856, 339, 730, 372, 340, }    )) {xcontrol = 7; ycontrol=8;} //tiger
    else if( gesture(f,  (const int[])   { 328, 687, 377, 472, 343, 321, 407, }    )) {xcontrol = 9; ycontrol=10;} //duck
    else if( gesture(f,  (const int[])   { 305, 593, 871, 573, 816, 340, 640, }    )) {xcontrol = 11; ycontrol=12;} //gun
    else if( gesture(f,  (const int[])   { 313, 445, 347, 599, 829, 342, 624, }    )) {xcontrol = 13; ycontrol=14;} // big gun copy this line to add new gestures

    bool in_gesture = (xcontrol != 1 || ycontrol != -1);
    
    if (xcontrol != -1 && xy_lock != 2){
      int xmidi = map (accel_x, 130, -220, 0, 127);
      if (xmidi>=127) xmidi=127;
      if (xmidi<=0) xmidi=0;    
      sendControl(xcontrol, xmidi);
    }
    if (ycontrol != -1 && xy_lock != 1){
      int ymidi = map (accel_y, -100, 245, 0, 127);  
      if (ymidi>=127) ymidi=127;
      if (ymidi<=0) ymidi=0;   
      sendControl(ycontrol, ymidi);
    }
    
    if (in_gesture){
      analogWrite(BLUE, 0);
      analogWrite(GREEN, 255);
      analogWrite(RED, 0);
    } else {
      analogWrite(BLUE, 255);
      analogWrite(GREEN, 0);
      analogWrite(RED, 255);
      digitalWrite(motorPin, LOW);
    }
      
  } else if (mode == 1) { // accel is the note value
      
  } else if (mode == 2) { // midi arpeggio
    
  } else if (mode == 3) { // z val -> drum awesome
  
  } 
}

void sendControl(uint8_t num, uint8_t val) {
  usbMIDI.sendControlChange(num, val, 1); 
  #ifdef DEBUG
  Serial.print("sending ");
  Serial.print(num);
  Serial.print(" ");
  Serial.print(val);
  Serial.println();
  #endif
}
  
void initAccel(){
    Wire.begin();
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(0);
  Wire.endTransmission();
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(16);
  Wire.endTransmission();
  Wire.beginTransmission(accel_module);
  Wire.write(0x2D);
  Wire.write(8);
  Wire.endTransmission();
}


void readAccel() {
   int16_t xyzregister = 0x32;

   Wire.beginTransmission(accel_module);
   Wire.write(xyzregister);
   Wire.endTransmission();
   Wire.beginTransmission(accel_module);
   Wire.write(xyzregister);
   Wire.endTransmission();
   Wire.beginTransmission(accel_module);
   Wire.requestFrom(accel_module, 6);
   int16_t i = 0;
   while(Wire.available()){
     values[i] = Wire.read();
     i++;
   }
   Wire.endTransmission();
   
   accel_x = (((int16_t)values[1]) << 8) | values [0];
   accel_y = (((int16_t)values[3]) << 8) | values [2];
   accel_z = (((int16_t)values[5]) << 8) | values [4];
}

void initFingers(){
    for(int i = 0; i< FINGER_N; i++){
      fingers[i].init(bendPins[i], true);
    }
}

void initButtons() {
  //XYLockButton = Bounce(7, 10);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
}

void readButtons(){
  //XYLockButton.update();
  ModeButton.update();
}


void readFingers(){
  for(int i = 0; i< FINGER_N; i++){
    fingers[i].read();
  }
}

#define DIFF_THRESH 100
bool gesture(int fv[], const int def[]){
  for(int i = 0; i < FINGER_N; i++){
    if (abs(fv[i] - def[i]) > DIFF_THRESH)
      return false;
  }
  return true;
}
