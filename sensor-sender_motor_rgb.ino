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
#define MODE1_CONTROL MODE0_CONTROL + 20
#define MODE2_CONTROL MODE1_CONTROL + 1

#define BLUE1 10
#define GREEN1 6
#define RED1 9
#define BLUE2 5
#define GREEN2 3
#define RED2 4

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
Bounce MuteButton = Bounce(FINGERPIN2, 10);

uint8_t xy_lock = 0;  // Store whether or not we're in XY lock.
                      // This is so we can connect with Ableton
                      // 0 = send both, 1 = send X, 2 = send y

//accel init
#define accel_module (0x53)    //ACCELEROMETER SETUP
byte values [6];
char output [512];

//vibration motor
const int motorPin = 23;

//Which mode we are in: 0 is gesture
//mode 1 is accel -> note
#define MODES 3
int mode = 0;

bool in_calibration_mode = false;
int calib_on_time = -1; // You have to hold down the calibration buton. Track that.
bool muted = false;

/*
 * current note
 */
int current_note = -1;


// timing details: we want to send midi at a 100hz clock
// that means every 10 ms
int lastMidiSend = 0;
#define MIDI_INTERVAL 20

/*
 * Arpeggiator!
 * The number of midi clock ticks we've received. There
 * are 24 per quarter note.
 */
int midi_clock = 1;
bool send_arpegg = false;
uint8_t arpegg_interval = 0; // number of ticks between on and off (and on, and off...)

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once
  initFingers();
  initAccel();
  initButtons();

  usbMIDI.setHandleRealTimeSystem(handleMidiClock);

  readCalibration();
  
  pinMode(motorPin, OUTPUT);
  
  pinMode(BLUE1, OUTPUT);
  pinMode(GREEN1, OUTPUT);
  pinMode(BLUE1, OUTPUT);
  pinMode(BLUE2, OUTPUT);
  pinMode(GREEN2, OUTPUT);
  pinMode(BLUE2, OUTPUT);

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
    Serial.print("mode switch ");
    Serial.println(mode);

    /*
     * We've switched modes - clean some stuff up.
     */
    xy_lock = 0;
    clearOutput();

    //Save the current mode to EEPROM
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
    clearOutput();
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

  //Check for mute mode
  if (MuteButton.fallingEdge()){
    muted = !muted;
    if(muted){
      clearOutput();
    }
  }
  
  //Do MIDI communication
  int t = millis();
  if (t - lastMidiSend >= MIDI_INTERVAL) {
    if(!muted){
      sendOutput();
    }
    lastMidiSend = t;
  }  
 
  // clean up: clear midi buffer
  while (usbMIDI.read()) ; // read and discard any incoming MIDI messages
}

void clearOutput() {
  send_arpegg = false;
  if (current_note != -1){
    usbMIDI.sendNoteOff(current_note, 99, MIDI_CHAN);
    current_note = -1;
  }

  analogWrite(BLUE1, 0);
  analogWrite(BLUE2, 0);
  analogWrite(RED1, 0);
  analogWrite(RED2, 0);
  analogWrite(GREEN1, 0);
  analogWrite(GREEN2, 0);
}

int gestureDebounce = 0;
int lastxcontrol = -1;
int lastycontrol = -1;
void sendOutput() {
   // timing code: if it's been more than 20ms since our last
  // send, then send again
  Serial.println("Mode " + (String) mode + (String) xy_lock);

  if (XYLockButton.update() && XYLockButton.fallingEdge()){
      xy_lock++;
  }


  /*
   * If there are no gestures, send nothing.
   * If there is a thumb gesture, send a note.
   * If there are other gestures, send control changes
   */
  if (mode == 0){
    // calibration mode: xy lock
    xy_lock %= 3;
    
   
    // Which control number we should send.
    // This code will read the states of all of the fingers, and decide
    // which, if any, controls should be sent
    int xcontrol = -1;
    int ycontrol = -1;
    bool send_note = false;

    Serial.print("{ ");
    for (int i = 0; i < FINGER_N; i++){
      Serial.print(fingers[i]);
      Serial.print(", ");
    }
    Serial.print("}");
    Serial.println();
    
    /*
     * Todo: debounce this somehow! ouch.
     */
    if(gesture(fingers, (const int[])          { 465, 89, 250, 461, 467, 450, 404, 414,  }     )) {xcontrol = 1; ycontrol=2; } //pistol
    else if( gesture(fingers,  (const int[])   { 250, 414, 474, 457, 438, 408, 341, 386, }     )) {xcontrol = 3; ycontrol=4;} //fist
    else if( gesture(fingers,  (const int[])   { 89, 50, -5, 30, 90, 31, 56, 52, }             )) {send_note = true; ycontrol=5; } //flat
    else if( gesture(fingers,  (const int[])   { 179, 266, -23, 361, 95, 348, 108, 150, }      )) {xcontrol = 7; ycontrol=8;} //tiger
    else if( gesture(fingers,  (const int[])   { 528, 42, 402, 47, 420, 43, 394, 176, }        )) {xcontrol = 9; ycontrol=10;} //duck
    else if( gesture(fingers,  (const int[])   { 485, 36, 260, 456, 456, 411, 303, 62, }       )) {xcontrol = 11; ycontrol=12;} //metalhead
    else if( gesture(fingers,  (const int[])   { 416, 31, 158, 24, 500, 403, 408, 418, }       )) {xcontrol = 13; ycontrol=14;} // big gun copy this line to add new gestures

    bool in_gesture = (xcontrol != -1 || ycontrol != -1);

    if (xcontrol != lastxcontrol  || ycontrol != lastycontrol) {
      if (gestureDebounce == 0){
        gestureDebounce = millis();
        return;
      }

      if (millis() - gestureDebounce < 40){
        return;
      }
    }

    gestureDebounce = 0;
    lastxcontrol = xcontrol;
    lastycontrol = ycontrol;
    

    if (send_note && xy_lock != 2) {
      noteOn();
      analogWrite(RED1, 255);
      analogWrite(RED2, 255);
    } else {
      noteOff();
      analogWrite(RED1, 0);
      analogWrite(RED2, 0);
    }
    
    if (in_gesture){
      Serial.println("In gesture");
      int xmidi = 0;
      int ymidi = 0;
      if (xcontrol != -1 && xy_lock != 2){
        xmidi = map (accel_x, -130, 220, 0, 127);    
        sendControl(xcontrol + MODE0_CONTROL, xmidi);
      }
      if (ycontrol != -1 && xy_lock != 1){
        ymidi = map (accel_y, 150, -245, 0, 127);    
        sendControl(ycontrol + MODE0_CONTROL, ymidi);
      }

      /*
       * Send a motor hit
       */
      digitalWrite(motorPin, HIGH);
      delay(4);
      digitalWrite(motorPin, LOW);
      delay(6);

      /*
       * Show the LED: calibrate the midi values to a LED value
       */
      int ledval = map(ymidi, 0, 127, 90, 200);
      analogWrite(BLUE1, ledval);
      analogWrite(BLUE2, ledval);

      ledval = map(xmidi, 0, 127, 90, 200);
      analogWrite(GREEN1, ledval);
      analogWrite(GREEN2, ledval);
      
    }
    else {
      analogWrite(BLUE1, 0);
      analogWrite(BLUE2, 0);
      analogWrite(GREEN1, 0);
      analogWrite(GREEN2, 0);
    }
  }
   
  /*
   * Accel X is the note value.
   * Accel Y is the speed
   * If the fingers are in a fist, then send a note. 
   */
  else if (mode == 1 ) {
    bool in_fist = true;
    for (int i = 1; i < FINGER_N && in_fist; i++){
      if (fingers[i] < 330){
        in_fist = false;
      }
    }

    if( in_fist) {
      send_arpegg = true;
    } else {
      send_arpegg = false;
      noteOff();
    }

    /*
     * Map the Y accel to the midi interval
     */
    arpegg_interval =  map (accel_y, 250, -250, 1, 4) * 2;
    if( arpegg_interval < 1){ arpegg_interval = 1; }
    //Serial.print("Arpeggio interval: ");
    //Serial.println(arpegg_interval);
  }

  /*
   * All of the fingers (except thumb) are controls
   * Fingers are 21-29, X is 29, and Y is 30
   */
  else if (mode == 2) {
    // calibration mode: xy lock
    xy_lock %= (FINGER_N + 2);

    /* Program mode - obey xy_lock */
    if (xy_lock != 0){
      int val = 0;
      if (xy_lock < FINGER_N - 1){
        val = map(fingers[xy_lock], 600, 0, 0, 127);
      } else if (xy_lock == FINGER_N - 1){
        val = map (accel_x, -130, 220, 0, 127); 
      } else if (xy_lock == FINGER_N ){
        val = map (accel_y, 100, -245, 0, 127);
      }
      sendControl(xy_lock + MODE2_CONTROL, val);    
    }
    else {
      for(int i = 0; i < FINGER_N - 1; i++){
        int val = map(fingers[i + 1], 600, 0, 0, 127);
        sendControl(i + MODE2_CONTROL, val);
      }
      int val = map (accel_x, -130, 220, 0, 127);
      sendControl(FINGER_N - 1 + MODE2_CONTROL, val);
      val = map (accel_y, 100, -245, 0, 127);
      sendControl(FINGER_N + MODE2_CONTROL, val);
    }
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


/*
 * Send a note, based on accel_x, that is mapped
 * to a one-octave C-major scale.
 * 
 * This needs to be debounced
 */
int noteDebounce = 0;
void noteOn(){
#define SCALE_SIZE 8
  int note = map (accel_x, -130, 220, 0, SCALE_SIZE);  
  if (note >= SCALE_SIZE ) note = SCALE_SIZE - 1;
  if (note<=0) note=0;

  int scale[] = {60, 62, 64, 65, 67, 69, 71, 72};
  note = scale[note];

  if (current_note == note){
    noteDebounce = 0;
    return;
  }
  
  if (current_note != -1){
    if (noteDebounce == 0){
      noteDebounce = millis();
      return;
    }
    if ((millis() - noteDebounce) < 50){
      return;
    }
    usbMIDI.sendNoteOff(current_note, 99, MIDI_CHAN);
  }

  noteDebounce = 0;
  usbMIDI.sendNoteOn(note, 99, MIDI_CHAN);
  current_note = note;
}

void noteOff(){
  if (current_note != -1){
    // send note up
    usbMIDI.sendNoteOff(current_note, 99, MIDI_CHAN);
    current_note = -1;
  }
}

/**
 * Called every time there's a midi tick
 */
void handleMidiTick() { 
  // We're just using this for arpeggiator
  if (! send_arpegg) {
    return;
  }
  int val = midi_clock % (arpegg_interval * 2);

  if (val == 0){
    noteOn();
    analogWrite(RED1, 100);
    analogWrite(RED2, 100);
    analogWrite(GREEN1, 100);
    analogWrite(GREEN2, 100);
  } else if (val == arpegg_interval) {
    noteOff();
    analogWrite(RED1, 0);
    analogWrite(RED2, 0);
    analogWrite(GREEN1, 0);
    analogWrite(GREEN2, 0);
  }
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
  MuteButton.update();
}


void readFingers(){
  for(int i = 0; i< FINGER_N; i++){
    SmoothPin pin = finger_pins[i];
    pin.read();

    // Apply the calibration values to the raw finger result
    fingers[i] = map(pin.value, fingerMin[i], fingerMax[i], 0, 500);
    //fingers[i] = pin.value;
    //Serial.println(pin.value);
  }
}


#define DIFF_THRESH 150
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
    char buf[128];
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

/*
 * Called by the midi library, whenever a tick comes in. 
 */
void handleMidiClock(byte message){
  if (message == 248){ // tick
    midi_clock++;
    handleMidiTick();
  }

  else if (message == 250 || message == 251){
    //MIDI START / MIDI CONTINUE
    midi_clock = 0;
  }
  else if (message == 252){
    // MIDI STOP
    if (send_arpegg){
      analogWrite(RED1, 0);
      analogWrite(RED2, 0);
      analogWrite(GREEN1, 0);
      analogWrite(GREEN2, 0);
      noteOff();
    }
  }
}

