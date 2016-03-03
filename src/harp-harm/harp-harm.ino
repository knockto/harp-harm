#include <Wire.h>

// Libraries are located in repository
#include <Adafruit_PWMServoDriver.h>
#include <MIDI.h>
#include <SoftwareSerial.h>

// Based on Arduino Mega + hardware modified MIDI shield
// Serial pins are removed and serial is connected to pins 50+51.
SoftwareSerial softSerial(51,50);

MIDI_CREATE_INSTANCE(SoftwareSerial, softSerial, MIDI); 
 
// 16 Channel I2C PWM driver
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x40);

// Minium and maximum angle impulse length for servi
#define ANGLE_MIN  220
#define ANGLE_MAX  440

// RGB Led (informational) is connected to pins 10, 11, 12 without resistor.
#define RED 12 // Red led is blinking when unknown (out of range #[36..36+16]) NOTE_ON signals are received
#define GREEN 11 // Green led is blinking when known NOTE_ON signals are received
#define BLUE 10 // Blue is faded alone when Dimmer value is changing

// PWM maximum brightness of RGB led
#define LED_BRIGHTNESS 50

// Variable resistor is connected to A5 port and can reduce gap
// between minimum and maximum servo angles.

#define DIMMER_PIN A5

// Used servos
#define SERVO_COUNT 16

int dimmerValue = 0;
bool servoStatus[16];
bool blinkValue = false;
bool blinkValueRed = false;
bool blinkValueGreen = false;
bool blinkValueBlue = false;

void setupServo() {
  servo.begin();
  servo.setPWMFreq(60);
  
  for(int i = 0; i < SERVO_COUNT; i++) {
    servoStatus[i] = false;
  }  
 
  // Move all servos to one side at start
  for(int i = 0; i < SERVO_COUNT; i++) {
    toggleServo(i);
  }
  
}

void setupMidi() {
  MIDI.setHandleNoteOn(midiNoteOn); 
  MIDI.begin();
  delay(500);
}

void setupDimmer() {
  // dimmmer values 0..20
  updateDimmer();
}

void updateDimmer() {
  int dimmerSensorValue = (1023-analogRead(DIMMER_PIN)) / 50;
  if (dimmerSensorValue != dimmerValue) {
    dimmerValue = dimmerSensorValue;    
    blinkerBrightness(BLUE, dimmerValue * 2.5);
    Serial.print("DIMMER: ");
    Serial.println(dimmerValue);
  }

}

void midiNoteOn(byte channel, byte pitch, byte velocity)
{
  int servoId = pitch - 36;
  Serial.print("MIDI #");
  Serial.print(pitch);
  if (servoId >= 0 && servoId < SERVO_COUNT) {
    Serial.print(" -> ");
    Serial.println(servoId);
    blinkerColor(GREEN);    
    toggleServo(servoId);
  } else {
    blinkerColor(RED);
    Serial.println(" out of range!");
  }
}

void setup() {
  setupDimmer();
  setupServo();
  setupMidi();
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  Serial.println("HarpHarm\n\nAndrew G Kuznetsov\ncav@cav.ru https://github.com/knockto/harp-harm" );
} 

void loop() {
  MIDI.read();
  updateDimmer();
}

void toggleServo(int servoId) {
  int value;
  
  servoStatus[servoId] = !servoStatus[servoId];
  if (servoStatus[servoId]) {
    value = ANGLE_MIN + dimmerValue * 5;
  } else {
    value = ANGLE_MAX - dimmerValue * 5;
  }
  
  servo.setPWM(servoId, 0, value);  
}

void blinker() {
  int value;
  blinkValue = !blinkValue;
  if (blinkValue) {
    value = HIGH;
  } else {
    value = LOW;
  }
  digitalWrite(13, value);
}

void blinkerColor(int color) {
  if ( color != RED) analogWrite(RED, 0);
  if ( color != GREEN) analogWrite(GREEN, 0);  
  if ( color != BLUE) analogWrite(BLUE, 0); 
  bool result;
  
  if (color == RED) {
    blinkValueRed = !blinkValueRed;
    result = blinkValueRed;
  }
  
  if (color == GREEN) {
    blinkValueGreen = !blinkValueGreen;
    result = blinkValueGreen;
  }  
  
  if (color == BLUE) {
    blinkValueBlue = !blinkValueBlue;
    result = blinkValueBlue;
  }    
  
  analogWrite(color, result ? LED_BRIGHTNESS : 0);
}


void blinkerBrightness(int color, int value) {
  if ( color != RED) analogWrite(RED, 0);
  if ( color != GREEN) analogWrite(GREEN, 0);  
  if ( color != BLUE) analogWrite(BLUE, 0); 
  analogWrite(color, value);
}



