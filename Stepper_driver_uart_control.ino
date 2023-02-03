//---------Libraries

#include <TMC2208Stepper.h>



//----------Declare pins

//button for testing 
const int button_1 = 4; //jetting cycle
int button_1_state;  //variable to store the state of button 1

// Stepper Motor control pins
const int enable_pin =8; //stepper driver enable pin
const int step_pin = 12; //Pin to make motor take a step
const int dir_pin =13; // Stepper direction pin: HIGH = retract, LOW = prime



//----------Stepper driver software serial

#define RX_pin  11  // SoftwareSerial pins
#define TX_pin  10  //
#define R_SENSE 0.11f // SilentStepStick series use 0.11

TMC2208Stepper driver(RX_pin, TX_pin, R_SENSE);   



//----------Stepper motor physical constants

int micro_steps = 64; //maximum of 256
float pitch = 2*3.1415*6;
float motor_deg = 1.8;
uint32_t steps_per_mm; //motor steps per translational mm
uint32_t steps_per_rev; //motor steps per full rotation



//----------Functions

void calc_constants()
{
  steps_per_rev = round((360/motor_deg)*micro_steps); //calculate steps per full rotation
  steps_per_mm = round((360/motor_deg)*(double(micro_steps)/pitch)); //Calculate steps per mm
  //steps_per_mm = 170;
}

 
uint32_t calc_steps_dist(uint32_t d)
//calculates steps from inputed distance in mm
{
  return d*steps_per_mm;
}


void move_steps(uint32_t steps,float v ,int direct)
//function to move the stepper motor the input number steps at the input speed and in the given direction,speed and direction
//direction controlled by inputing direct as HIGH or LOW to switch between the two directions
{
  double time_per_step;
  time_per_step=1/(steps_per_mm*v); // convert mm/s into s/step to create a delay between steps that results in the desired speed
  time_per_step=time_per_step*1e6; // change from seconds to microseconds
  
  for(uint32_t i=0; i < steps; i++)
  {
    
    digitalWrite(dir_pin,direct);
    digitalWrite(step_pin,HIGH); 
    delayMicroseconds(time_per_step/2); //The delay between high pulses results in the stepper speed
    digitalWrite(step_pin,LOW);
    delayMicroseconds(time_per_step/2);
  }
}








void setup() 
{  
  Serial.begin(9600); //serial terminal for debugging
  driver.beginSerial(115200); //Default serial rate for stepper driver
    
  driver.push(); // Push at the start of setting up the driver resets the register to default
  
 //Stepper driver 
 pinMode(enable_pin, OUTPUT); //Enable stepper driver
 pinMode(step_pin,OUTPUT);  //Pin to set high to make motor take a step
 pinMode(dir_pin,OUTPUT);  //Sets motor direction
 pinMode(button_1,INPUT_PULLUP); //button for testing stepper
 digitalWrite(enable_pin, HIGH);

 driver.pdn_disable(true); 
 driver.I_scale_analog(false); // Use internal voltage reference
 driver.rms_current(750);      // Set motor RMS current Typical range 500-800 (1200 is the speced data sheet max)
 driver.mstep_reg_select(1);  //Needed to set microsteps over uart!
 driver.microsteps(micro_steps); // Set microsteps
 driver.en_spreadCycle(true);  //Change from default stealthChop to spreadcycle which has more power
 //driver.pwm_autoscale(true);  //Needed for stealthChop
  
 calc_constants();  //calculate steps/rotation
}



void loop() {
  Serial.println(steps_per_rev);
  Serial.println(steps_per_mm);
  
  uint32_t steps; //calculate steps to move 10mm
  steps=calc_steps_dist(10);
  Serial.println(steps);
  
  button_1_state=digitalRead(button_1);
  if(button_1_state== LOW)
  {
    digitalWrite(enable_pin, LOW);
    move_steps(steps,15,LOW); //move stepper to move slider 10mm-use this to check calcalated steps/mm and microstepping is working correctly
    delay(200);
    digitalWrite(enable_pin, HIGH);
  }
}
