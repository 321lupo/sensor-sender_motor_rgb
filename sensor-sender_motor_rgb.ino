#include <Bounce.h>

#include <autotune.h>
#include <microsmooth.h>

#include <MIDI.h>
#include <EEPROM.h>
#include <Wire.h>
const int channel = 1;

#include "SmoothPin.h"

#define NUM 3
#define MIDI_CHAN 1
#define DEBUG 1


// control number offsets
#define MODE0_CONTROL 0
#define MODE1_CONTROL 20
#define MODE2_CONTROL 30

#define GREEN 3
#define BLUE 4
#define RED 5

//sensor values
#define FINGER_N 8
int16_t accel_x, accel_y, accel_z;
SmoothPin finger_pins[FINGER_N];
int fingers[FINGER_N];
int ir; //TODO

int bendPins[] = {A14, A0, A1, A2, A3, A6, A7, A8}; // the pins for each of the finger resistors

uint16_t fingerMin[FINGER_N];
uint16_t fingerMax[FINGER_N];
// the eeprom address for the calibration
#define CALIB_ADDR 16


/*
 * Button setup: define pins for all of the action buttons
 * buttons are pins 6, 7, 10, 11
 */
#define FINGERPIN2 2
#define FINGERPIN1 1
#define BUTTON3 11
#define BUTTON4 12

Bounce XYLockButton = Bounce(BUTTON3, 10);
Bounce ModeButton = Bounce(FINGERPIN1, 10);
Bounce CalibButton = Bounce(BUTTON4, 10);

uint8_t xy_lock = 0;  // Store whether or not we're in XY lock.
                      // This is so we can connect with Ableton
                      // 0 = send both, 1 = send X, 2 = send y

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

bool in_calibration_mode = false;
int calib_on_time = -1;

/*
 * mode 1 state
 */
int mode1note = -1;


/*
 * mode 2 state
 */
int mode2note = -1;

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

  readCalibration();
  
  pinMode(motorPin, OUTPUT);
  
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(RED, OUTPUT);

  mode = EEPROM.read(1);
  
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

    //mode 0
    xy_lock = 0;

    // mode 1
    if (mode1note != -1){
      usbMIDI.sendNoteOff(mode1note, 99, MIDI_CHAN);
      mode1note = -1;
    }
    EEPROM.write(1, mode);
  }

  /*
   * Calibration: check to see if button is pushed
   */
  if (!in_calibration_mode && CalibButton.fallingEdge()){
    Serial.println("Calib?");
    calib_on_time = millis();
  } else if (CalibButton.risingEdge()){
    calib_on_time = -1;
  }

  if (!in_calibration_mode && calib_on_time > -1 && millis() - calib_on_time > 3000){
    Serial.println("enter calib");
    in_calibration_mode = true;
    resetCalibration();
  }
  
  if (in_calibration_mode){
    Serial.println("In Calibration");
    if (CalibButton.fallingEdge()){
      in_calibration_mode = false;
      writeCalibration();
      return;
    }
    updateCalibration();
    return;
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

  if (XYLockButton.update() && XYLockButton.fallingEdge()){
      xy_lock++;
  }
  
  if (mode == 0){ // gesture selects the control channel
    // calibration mode: xy lock
    xy_lock %= 3;
    
    // TODO: calibrate accel
    
    
    
    // Which control number we should send.
    // This code will read the states of all of the fingers, and decide
    // which, if any, controls should be sent
    int xcontrol = -1;
    int ycontrol = -1;
    Serial.print("{ ");
    for (int i = 0; i < FINGER_N; i++){
      Serial.print(fingers[i]);
      Serial.print(", ");
    }
    Serial.print("}");
    Serial.println();
    
    
    if(gesture(fingers, (const int[]){ 430, 529, 883, 840, 791, 629, 575, 250, }))               {xcontrol = 1; ycontrol=2; } //pistol
    else if( gesture(fingers,  (const int[])  { 291, 450, 376, 472, 421, 478, 467, 487, }  )) {xcontrol = 3; ycontrol=4;} //fist
    else if( gesture(fingers,  (const int[])   { 334, 361, 343, 391, 311, 339, 338, 148, }    )) {xcontrol = 5; ycontrol=6;} //flat
    else if( gesture(fingers,  (const int[])   { 372, 345, 832, 409, 741, 333, 588, 228, }   )) {xcontrol = 7; ycontrol=8;} //tiger
    else if( gesture(fingers,  (const int[])   { 473, 737, 361, 810, 333, 491, 454, 218, }    )) {xcontrol = 9; ycontrol=10;} //duck
    else if( gesture(fingers,  (const int[])   { 440, 423, 893, 834, 777, 568, 423, 155, }  )) {xcontrol = 11; ycontrol=12;} //metalhead
    else if( gesture(fingers,  (const int[])   { 403, 429, 341, 551, 778, 635, 599, 263, }    )) {xcontrol = 13; ycontrol=14;} // big gun copy this line to add new gestures

    bool in_gesture = (xcontrol != 1 || ycontrol != -1);
    
    if (xcontrol != -1 && xy_lock != 2){
      int xmidi = map (accel_x, -130, 220, 0, 127);    
      sendControl(xcontrol + MODE0_CONTROL, xmidi);
    }
    if (ycontrol != -1 && xy_lock != 1){
      int ymidi = map (accel_y, 100, -245, 0, 127);    
      sendControl(ycontrol + MODE0_CONTROL, ymidi);
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
      
  } 
  /*
   * Accel X is the note value.
   * the fingers are fixed control channels
   * The "key press" is the thumb
   */
  else if (mode == 1) {
    const int thumb_level = 280;
    xy_lock %= (FINGER_N);
    
    int note = map (accel_x, 130, -220, 0, 127);
    if (note>=127) note=127;
    if (note<=0) note=0;

    //Check if thumb is over threshold
    if( fingers[0] >= thumb_level && mode1note != note){
      if (mode1note != -1){
        usbMIDI.sendNoteOff(mode1note, 99, MIDI_CHAN);
      }
      #ifdef DEBUG
      Serial.print("Sending note on ");
      Serial.println(note);
      #endif
      
      usbMIDI.sendNoteOn(note, 99, MIDI_CHAN);
      mode1note = note;
    }
    
    else if (fingers[0] < thumb_level && mode1note != -1){
      // send note up
      usbMIDI.sendNoteOff(mode1note, 99, MIDI_CHAN);
      mode1note = -1;
    }

    // Send control channels
    if (xy_lock != 0){
      int val = map(fingers[xy_lock], 100, 400, 0, 127);
      sendControl(xy_lock + MODE1_CONTROL, val);
    } else {
      for(int i = 1; i< FINGER_N; i++){
        int val = map(fingers[i], 100, 400, 0, 127);
        sendControl(i + MODE1_CONTROL, val);
      }
    }
  } else if (mode == 2) { // midi arpeggio
    
  } else if (mode == 3) { // z val -> drum awesome
  
  } 
}

void sendControl(int num, int val) {
  if (val < 0){
    val = 0;
  } else if (val > 127){
    val = 127;
  }
  usbMIDI.sendControlChange(num, val, MIDI_CHAN); 
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
      finger_pins[i].init(bendPins[i], true);
    }
}

void initButtons() {
  //XYLockButton = Bounce(7, 10);
  pinMode(FINGERPIN1, INPUT_PULLUP);
  pinMode(FINGERPIN2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);
}

void readButtons(){
  //XYLockButton.update();
  ModeButton.update();
  CalibButton.update();
}


void readFingers(){
  for(int i = 0; i< FINGER_N; i++){
    SmoothPin pin = finger_pins[i];
    pin.read();
    fingers[i] = map(pin.value, fingerMin[i], fingerMax[i], 0, 500);
    //fingers[i] = pin.value;
    //Serial.println(pin.value);
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


void resetCalibration() {
  for (int i = 0; i<FINGER_N; i++){
    fingerMin[i] = 32767;
    fingerMax[i] = 0;
  }
}

void updateCalibration() {
  for (int i = 0; i < FINGER_N; i++){
    SmoothPin pin = finger_pins[i];
    int val = pin.read();
    if (val > fingerMax[i]){
      fingerMax[i] = val;
    }
    if (val < fingerMin[i]){
      fingerMin[i] = val;
    }
    char buf[30];
    sprintf(buf, "%d (%d): %d, %d ", i, val, fingerMin[i], fingerMax[i]);
    Serial.println(buf);
  }
}

/*
 * read the calibration values from eeprom
 */
void readCalibration() {
  Serial.println("Read calibraiton");
  for(int i=0; i< FINGER_N; i++){
    fingerMin[i] = readShort(CALIB_ADDR + 2*i);
    fingerMax[i] = readShort(CALIB_ADDR + 2*FINGER_N + 2*i);
     char buf[30];
    sprintf(buf, "%d: %d, %d ", i, fingerMin[i], fingerMax[i]);
    Serial.println(buf);
  }
}

uint16_t readShort(int addr){
  uint16_t val = EEPROM.read(addr);
  val *= 256;
  val += EEPROM.read(addr+1);
  return val;
}

void writeShort(int addr, uint16_t val){
  EEPROM.write(addr, val / 256);
  EEPROM.write(addr + 1, val % 256);
}

void writeCalibration() {
  for(int i=0; i< FINGER_N; i++){
    writeShort(CALIB_ADDR + 2*i, fingerMin[i]);
    writeShort(CALIB_ADDR + 2*FINGER_N + 2*i, fingerMax[i]);
  }
}

