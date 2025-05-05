// Base code.
// 
// *  NOTE: this code will do only three things:
// *    --rotate one wheel, and 
// *    --blink the right front mainboard LED.
// *    
// *  You will need to add more code to
// *  make the car do anything useful. 
// 
#include<ECE3.h>
//#include <ECE3_LCD7.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7
int sensor_values[8];

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;
const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;
const int LED_RF = 41;

void doCalibration();
void getPID_error();
void getData();

int minimum[8] = {577,484,484,415,484,484,461,812};
int maximum[8] = {1923,1328,1375,849,1375,923,1493};
int weights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
const double MAXSPEED=255;
double error=0;
double last_error=0;
double deltaE = 0;
double totalError = 0;
double pid_error = 0;

double spd=70;
uint16_t = previousTime = 0;
uint16_t = currentTime = 0;
const int interval = 6; //ms
bool PID_ON=true;
double kp, kd, ki;

///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
//  ECE3_Init();

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  
}

void doCalibration(){
  for(int n=0; n<8; n++){
    all_sets[8][8] = ECE3_read_IR(sensorValues);
  }
  for(int j=0; j<8; j++){ //iterate through each row
    int maximumVal=all_set[j][0];
    int minimumVal=all_set[j][0];
    for (int i=0; i<8; i++){  //iterate through each column
      if(all_sets[j][k]>maximumVal){
        maximum[j] = all_sets[j][k];
      }
      if(all_sets[j][k]<minimumVal){
        minimum[j] = all_sets[j][k];
      }
   
    }
  }
  
}

void getPID_error(){
  deltaError = error-last_error;
  totalError += changeError;
  pid_error = kp*error + kd*deltaError + ki*totalError;
  if(spd+pid_error>MAXSPEED){
    pid_error = MAXSPEED-spd;
  }else if ((spd-pid_error)<-MAXSPEED){
    pid_error = -MAXSPEED+spd;
  }
  last_error = error;
}

void getError(){
  for(int k=0; k<8; k++){
    sensor_values[k] -= minimum[k];
    sensor_values[k]*= 1000/maximum[k];
    error += sensor_values[k]*weights[k];
  }
}

void loop() {
  // put your main code here, to run repeatedly: 
  currentTime = millis();
  if(PID_ON){
    if((currentTime-previousTime)>interval){
      doCalibration();
      previousTime=currentTime;
    }
    //ECE3_read_IR(sensorValues);

    for(int i=0; i<8; i++){
      sensor_values[i] = static_cast<int>(sensorValues[i]);
    }
  getError();
  getPID_error();
  // Serial.println(error);
  //+error: center of car too much to the left (+pid_error to left)
  //-error: center of car too much to the right (+pid_error to right)
  analogWrite(left_pwm_pin,spd+pid_error);
  analogWrite(right_pwm_pin,spd-pid_error);
  
}

 
//  int leftSpd = 70;

//  ECE3_read_IR(sensorValues);

//  analogWrite(left_pwm_pin,leftSpd);

// 
  
  ECE3_read_IR(sensorValues);

  // digitalWrite(LED_RF, HIGH);
  // delay(250);
  // digitalWrite(LED_RF, LOW);
  // delay(250);
    
}
