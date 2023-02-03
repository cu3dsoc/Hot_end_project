// WG Created: 13/5/22 Modified: 3/2/23
// Combined code to control the hot end test rig (filament encoder, stepper motor and heater)

#include <TMC2208Stepper.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define MAX31855_CS   5
#define MAX31855_DO   6
#define MAX31855_CLK  7
#define RX_pin  11  // SoftwareSerial receiver pin
#define TX_pin  10  // SoftwareSerial transmitter pin
#define R_SENSE 0.11f // SilentStepStick series use 0.11
TMC2208Stepper driver(RX_pin, TX_pin, R_SENSE);   

// Define pins
const int PWM_pin = 3;
// Not sure if pin 11 needs to be used to the heater as well??
const int button_1 = 4; // jetting cycle
const int stepper_enable_pin =12; // stepper driver enable pin
const int stepper_step_pin = 11; // Pin to make motor take a step
const int stepper_dir_pin =10; // Stepper direction pin: HIGH = retract, LOW = prime
const int filament_encoder_emitterPin = 13; // This is the yellow wire
const int filament_encoder_signalPin =  9; // This is the blue wire

// General Constants
const float pi = 3.1415;

// Filament Encoder Constants:
const float wheelDiameter = 15; // mm
const float filamentDiameter = 2.85; // mm
const int numTeeth = 26;
const float wheelCircumference = wheelDiameter*pi; // will be in mm
const float distPerTooth = wheelCircumference/numTeeth; // will be in mm

// Stepper Motor Constants
const int micro_steps = 64; //maximum of 256
const float pitch = 2*pi*6;
const float motor_deg = 1.8;
const uint32_t steps_per_mm = round((360/motor_deg)*(double(micro_steps)/pitch)); // motor steps per translational mm (=170)
const uint32_t steps_per_rev = round((360/motor_deg)*micro_steps); // motor steps per full rotation

// PID heater control constants
const int kp = 9.1;     const int ki = 0.3;     const int kd = 1.8;
const int PID_p = 0;    const int PID_i = 0;    const int PID_d = 0;

// variables:
int sensorState = 0;                      // variable for reading the sensor state
int lastState = 0;                        // records the previous state of the sensor
int counter = 0;                          // counts how many times the teeth pass
float distTravelled = 0;                  // variable to count how many revolutions of the wheel have occured since initiation
long int previousTimeStamp = millis();    // records time of previous passing tooth
long int newTimeStamp = millis();         // records time of passing tooth
long int timeSinceCounterChange = 0;      // records time since previous passing tooth in ms
int button_1_state;                       // variable to store the state of button 1

float temperature_read = 0.0;
float set_temperature = 100;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;


void move_steps(uint32_t steps, float v, int direct) {
  // Function to move the stepper motor the input number steps at the input speed and in the given direction
  // Direction controlled by inputing direct as HIGH or LOW to switch between the two directions
  double time_per_step;
  time_per_step = 1/(steps_per_mm*v); // convert mm/s into s/step to create a delay between steps that results in the desired speed
  time_per_step = time_per_step*1e6; // change from seconds to microseconds
  
  for(uint32_t i=0; i < steps; i++) {
    digitalWrite(stepper_dir_pin, direct);
    digitalWrite(stepper_step_pin, HIGH); 
    delayMicroseconds(time_per_step/2); // The delay between high pulses results in the stepper speed
    digitalWrite(stepper_step_pin, LOW);
    delayMicroseconds(time_per_step/2);
  }
}

float calculate_flow_rate(int timeSinceCounterChange){
  float flowRate = (60*distPerTooth*filamentDiameter*filamentDiameter*pi)/(4*timeSinceCounterChange);
  return flowRate;
}

double readThermocouple() {
  // Function to read the thermocouple giving the measured temperature of the heater block
  uint16_t v;
  pinMode(MAX31855_CS, OUTPUT);
  pinMode(MAX31855_DO, INPUT);
  pinMode(MAX31855_CLK, OUTPUT);
  
  digitalWrite(MAX31855_CS, LOW);
  delay(1); // ms

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v = shiftIn(MAX31855_DO, MAX31855_CLK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX31855_DO, MAX31855_CLK, MSBFIRST);
  
  digitalWrite(MAX31855_CS, HIGH);
  if (v & 0x4) {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v*0.25;
}


void setup() {
  Serial.begin(9600); //Alows communication with a PC for debugging
  pinMode(filament_encoder_emitterPin, OUTPUT);
  pinMode(filament_encoder_signalPin, INPUT);
  digitalWrite(filament_encoder_emitterPin, HIGH);

  driver.beginSerial(115200); //Default serial rate for stepper driver
  driver.push(); // Push at the start of setting up the driver resets the register to default

  //Stepper driver 
  pinMode(stepper_enable_pin, OUTPUT); //Enable stepper driver
  pinMode(stepper_step_pin, OUTPUT);  //Pin to set high to make motor take a step
  pinMode(stepper_dir_pin, OUTPUT);  //Sets motor direction
  pinMode(button_1, INPUT_PULLUP); //button for testing stepper
  digitalWrite(stepper_enable_pin, HIGH);
  driver.pdn_disable(true); 
  driver.I_scale_analog(false); // Use internal voltage reference
  driver.rms_current(750);      // Set motor RMS current Typical range 500-800 (1200 is the speced data sheet max)
  driver.mstep_reg_select(1);  // Needed to set microsteps over uart!
  driver.microsteps(micro_steps); // Set microsteps
  driver.en_spreadCycle(true);  // Change from default stealthChop to spreadcycle which has more power
  //driver.pwm_autoscale(true);  // Needed for stealthChop

  pinMode(PWM_pin, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 and 11 PWM frequency of 980.39 Hz
  Time = millis(); 
}




void loop() {

  // Code to calculate the filament flow rate
  sensorState = digitalRead(filament_encoder_signalPin);
  if (sensorState > lastState) {
    newTimeStamp = millis();
    timeSinceCounterChange = newTimeStamp - previousTimeStamp;
    previousTimeStamp = newTimeStamp;
    counter += 1;
    distTravelled = counter * distPerTooth;
    float flowRate = calculate_flow_rate(timeSinceCounterChange);
  }
  lastState = sensorState;
  String outputText = "Sensor: " + String(sensorState) + " Counter: " + counter + " Distance Travelled: " + distTravelled + " Flow Rate: " + flowRate;
  Serial.println(outputText);
  
  // Code to move the stepper motor forward by 10 mm if the button is pressed
  button_1_state=digitalRead(button_1);
  if(button_1_state == LOW) {
    digitalWrite(stepper_enable_pin, LOW);
    move_steps(steps_per_mm*10, 15, LOW); // move stepper to move slider 10mm-use this to check calcalated steps/mm and microstepping is working correctly
    delay(200);
    digitalWrite(stepper_enable_pin, HIGH);
  }

  // Code to run the PID loop for controlling the heater
  temperature_read = readThermocouple();
  if (temperature_read == 0){
    Serial.println("Thermocouple not wokring");
  }
  else {
    PID_error = set_temperature - temperature_read; // Calculate the error between the setpoint and the real value
    // Calculate the P value and I value in a range on ±3
    PID_p = kp*PID_error;
    if(-3 < PID_error < 3) {
      PID_i = PID_i + (ki*PID_error);
    }

    // For derivative we need real time to calculate speed change rate
    timePrev = Time;
    Time = millis();
    elapsedTime = (Time - timePrev) / 1000; 
    // Now we can calculate the D calue
    PID_d = kd*(PID_error - previous_error)/elapsedTime;
    // Final total PID value is the sum of P + I + D
    PID_value = PID_p + PID_i + PID_d;

    // We define PWM range between 0 and 255
    if(PID_value < 0) {
      PID_value = 0;
    }
    else if(PID_value > 255) {
      PID_value = 255;
    }
    // Now we can write the PWM signal to the mosfet on digital pin D3
    analogWrite(PWM_pin, PID_value);
    previous_error = PID_error; // Remember to store the previous error for next loop.

    delay(300);

    String outputText = "Set temperature: " + String(set_temperature) + " Current temperature: " + String(temperature_read);
    Serial.println(outputText);
  }

  
  delay(1);
}