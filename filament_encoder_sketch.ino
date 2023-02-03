
// WG Created: 13/5/22 Modified: 3/10/22
// Code to control the filament flow rate meter for the CU3D hot-end project

// constants:
const float pi = 3.1415;
const int emitterPin = 13; // This is the yellow wire
const int signalPin =  9; // This is the blue wire
const float wheelDiameter = 15; // mm
const float filamentDiameter = 2.85; // mm
const int numTeeth = 26; //
const float wheelCircumference = wheelDiameter*pi; // will be in mm
const float distPerTooth = wheelCircumference/numTeeth; // will be in mm

// variables:
int sensorState = 0;                      // variable for reading the sensor state
int lastState = 0;                        // records the previous state of the sensor
int counter = 0;                          // counts how many times the teeth pass
float distTravelled = 0;                  // variable to count how many revolutions of the wheel have occured since initiation
long int previousTimeStamp = millis();    // records time of previous passing tooth
long int newTimeStamp = millis();         // records time of passing tooth
long int timeSinceCounterChange = 0;      // records time since previous passing tooth
float flowRate = 0;                       // variable to record the final flow rate in mm^3/s


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(emitterPin, OUTPUT);
  pinMode(signalPin, INPUT);
  digitalWrite(emitterPin, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorState = digitalRead(signalPin);
  if (sensorState > lastState) {
    newTimeStamp = millis();
    timeSinceCounterChange = newTimeStamp - previousTimeStamp;
    previousTimeStamp = newTimeStamp;
    counter += 1;
    distTravelled = counter * distPerTooth;
    flowRate = (60*distPerTooth*filamentDiameter*filamentDiameter*pi)/(4*timeSinceCounterChange);
  }
  lastState = sensorState;
  String outputText = "Sensor: " + String(sensorState) + " Counter: " + counter + " Distance Travelled: " + distTravelled + " Flow Rate: " + flowRate;
  Serial.println(outputText);
  delay(1);
}
