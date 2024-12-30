#include "RP2040_PWM.h" //RP2040_PWM by Khoi Hoang
#include "PID_v1.h"

//pin definitions for motor driver
#define PWM 19 
#define AIN2 18   
#define AIN1 17
#define STB 16  //when this pin is 1 the motor driver works, when 0 it is not active

//pin definitions for ultrasonic sensor
#define TRIG_PIN 26 //pin that tells when to send the pulse
#define ECHO_PIN 27 //pin that read the received pulse

//variables for the motor
RP2040_PWM* PWM_Instance;
double frequency;

//variables for the sensor
double measure();
//saves the last valid measures (sometimes the sensor returns impossible values)
double last_dist = 0;

//varibles for the PID controller
double target = 10; //target distance for the ACC   y(t)
double Kp = 10;     //proportional coefficient  
double Ki = 1;      //intagrative coefficient
double Kd = 2;      //derivative coefficient
double input;
double output;      //u(t)


PID myPID(&input, &output, &target, Kp, Ki, Kd, DIRECT);

//function tha drives the motor
void drive(double duty, bool direction);

//set time to produce timestamps for the outputted data
time_t time0 = micros();

void setup() {
  Serial.begin(9600);
  //while(!Serial);

  //initialize pins
  pinMode(PWM, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(STB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  //initialize PWM
  frequency = 500;
  PWM_Instance = new RP2040_PWM(PWM, frequency, 0);

  //activate PIN that allows the motor driver to run the motor
  digitalWrite(STB, 1);

  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100);
}

void loop() {
  
  //read from sensor
  input = measure();
  if(input > 0) last_dist = input;  //correct invalid readings from the sensor
  else  input = last_dist;

  myPID.Compute();

  //if output < 0 the car is too far, power the motor forward
  //if output > 0 the car is too close, so slow down
  drive(abs(output), (output < 0));

  //print a string with the following format:
  //"timestamp, distance, pwm_output\n"
  time_t t1 = micros() - time0;
  Serial.print(t1);
  Serial.print(", ");
  Serial.print(input);
  Serial.print(", ");
  Serial.println(output);


  delayMicroseconds(200);
}



//method to read from the sensor
//sends a pulse for 10us and then computes the distance based on 
//the time needed for the pulse to come back
double measure(){

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  double duration = pulseIn(ECHO_PIN, HIGH);
  digitalWrite(TRIG_PIN, LOW);

  //duration = pulseIn(ECHO_PIN, HIGH);
  double distance = (duration*.0343)/2;
  

  if(distance <= 400) return distance;
  else return -1; //if the distance is more than 400 the measurement is corrupted

}

//method to drive the motor
/*
  duty is the duration of the PWM duty cycle
  direction dictates the direction of the motor
    True if need to go forward
    False if need to go backward
*/
void drive(double duty, bool direction){

  PWM_Instance->setPWM(PWM, frequency, duty);
  if(direction) {
    digitalWrite(AIN2, 1);
    digitalWrite(AIN1, 0);
  }
  else{
    digitalWrite(AIN2, 0);
    digitalWrite(AIN1, 1);
  }
  


}
