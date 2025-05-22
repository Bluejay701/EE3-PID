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
#include <stdio.h>
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

bool white_or_black[8] = {0};
int white_threshold[8] = {1000,900,900,800,900,750,900,1100};
int fake_no_path[8] = {714,620,573,457,550,503,526,809};

int minimum[8] = {0};//{577,484,484,415,484,484,461,812};
int maximum[8] = {1923,1328,1375,849,1375,923,1493};
int weights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};
const double MAXSPEED=40;
double error=0;
double last_error=0;
double deltaE = 0;
double totalError = 0;
double pid_error = 0;

double spd=25;
uint16_t previousTime = 0;
uint16_t currentTime = 0;
const int interval = 6; //ms
bool PID_ON=true;
//double kp, kd, ki;
// 5/19/25: CONSTANTS
double kp = -0.04;
double kd = -0.15;
double ki = 0;

// Recognize path type
//bool arch = false;
bool biasOrder[4] = {true, true, true, true}; // true means left biased
bool currentBias = biasOrder[0]; // true is left biased, false is right biased

bool turn_at_end = false;
int turn_buffer = 0;



int getL=0;
int getR=0;
int avg_pulse=0;


///////////////////////////////////
void setup() {
  ECE3_Init();

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

// set the data rate in bits/second for serial data transmission
  Serial.begin(9600);

  resetEncoderCount_left();
  resetEncoderCount_right();
   
  delay(2000); //Wait 2 seconds before starting 
  
}

//void doCalibration(){
//  for(int n=0; n<8; n++){
//    all_sets[8][8] = ECE3_read_IR(sensorValues);
//  }
//  for(int j=0; j<8; j++){ //iterate through each row
//    int maximumVal=all_set[j][0];
//    int minimumVal=all_set[j][0];
//    for (int i=0; i<8; i++){  //iterate through each column
//      if(all_sets[j][k]>maximumVal){
//        maximum[j] = all_sets[j][k];
//      }
//      if(all_sets[j][k]<minimumVal){
//        minimum[j] = all_sets[j][k];
//      }
//   
//    }
//  }
//  
//}

int average_pulse_count(){
  getL=getEncoderCount_left();
  getR=getEncoderCount_right();
//  Serial.print(getL);
//  Serial.print('\t');
//  Serial.print(getR);
  return((getEncoderCount_left()+getEncoderCount_right())/2);
}

void getPID_error(){
  deltaE = error-last_error;
  totalError += error;
  pid_error = kp*error + kd*deltaE + ki*totalError;
  // 5/19/25: commented out maxspeed stuff
//  if(spd+pid_error>MAXSPEED){
//    pid_error = MAXSPEED-spd;
//  }else if ((spd-pid_error)<-MAXSPEED){
//    pid_error = -MAXSPEED+spd;
//  }
  last_error = error;
}

void getError(){
  error = 0;
  for(int k=0; k<8; k++){
    // 5/19/25: commented out stuff related to minimums
//    if (sensor_values[k] < minimum[k]) {
//      sensor_values[k] = 0;
//    } else {
//      sensor_values[k] -= minimum[k];
//    }
    
    //sensor_values[k]*= 1000/maximum[k];
    error += sensor_values[k]*weights[k];
  }
}

bool checkEnd() {
  
  for(int k=0; k<8; k++){
    if(white_or_black[k]==0){
      return false;
    }
  }

  return true;
}

//void checkTrackType(){
//  avg_pulse = average_pulse_count();  // start encoder counts from 0
//
//    // CHANGE MADE 5/19/25: After wheels rotate 200 counts AND either condition (3 or 4 zeros in between 2 ones) are true, toggle ON boolean for arch
//    for (int n=0; n<8; n++){
//      if (avg_pulse>200 && ((white_or_black[n]==1 && white_or_black[n+4]==1) || (white_or_black[n]==1 && white_or_black[n+5]==1))){
//        arch=true;
////        Serial.println(avg_pulse);
//    }
//    else if (avg_pulse>200 && ((white_or_black[n]!=1 && white_or_black[n+4]!=1) && (white_or_black[n]!=1 && white_or_black[n+5]!=1))){
//      arch=false;
//    }
//
//    if (avg_pulse>100 && ((white_or_black[n]==1) && white_or_black[n+1]==1 && white_or_black[n+2]) 
//  }
//}
  
//void followArch(){
//  for(int m=0; m<8; m++){
//    // this assumes we are following the left path (won't work on the way back)
//    if(white_or_black[m]==1 && m<5){ // ignores the readings from right sensors I believe, which makes error only positive, i.e. follow the left line
//      sensor_values[m] = fake_no_path[m];
//    }
//  }
//}

void loop() {
  // put your main code here, to run repeatedly: 
//  currentTime = millis();
  if(PID_ON){
//    if((currentTime-previousTime)>interval){
//      doCalibration();
//      previousTime=currentTime;
//    }
    ECE3_read_IR(sensorValues);

    for(int i=0; i<8; i++){
      sensor_values[i] = sensorValues[i];
//      Serial.print(sensor_values[i]);
//      Serial.print('\t');

      // 5/19/25: added logic that allows us to modify the bool array like we had before
      if (sensor_values[i] > white_threshold[i]) {
        white_or_black[i] = true;
      } else {
        white_or_black[i] = false;
      }
    }

//    turn_at_end = true;

    Serial.print(getEncoderCount_left());
    Serial.println();
    Serial.print(getEncoderCount_right());
    Serial.println();

    if (checkEnd() && !turn_at_end) {
      if (turn_buffer > 2) {
        turn_buffer = 0;
        turn_at_end = true;
    resetEncoderCount_right();
    resetEncoderCount_left();
        } else {
          turn_buffer++;
        }
  }

  if (!checkEnd() && turn_buffer > 0) {
    turn_buffer = 0;
  }

  if (turn_at_end && abs(getEncoderCount_right()) > 400) {
    turn_at_end = false;
  }
  

    // block out values on the left or right depending on bias
    int first_black_index = -1;
      for(int m=7; m>=0; m--){
    // this assumes we are following the left path (won't work on the way back)
    if(white_or_black[m]==1){ // ignores the readings from right sensors I believe, which makes error only positive, i.e. follow the left line
      first_black_index = m;
      break;
    }
  }
  if (first_black_index != -1) {
    for (int i = 0; i < first_black_index; i++) {
    sensor_values[i] = fake_no_path[i];
  }
  }

  

    //CHANGE MADE 5/19/25: check track type every time after getting array of white_or_black 
//  checkTrackType();

//  if (arch){
//    followArch();
//  }
    
//    Serial.println();
  getError();
  getPID_error();
//  Serial.println(error);

//  for(int k=0; k<8; k++){
//  Serial.print(white_or_black[k]);
//  }
//  Serial.println();
//  for(int k=0; k<8; k++){
//  Serial.print(sensor_values[k]);
//  Serial.print('\t');
//  }
//  Serial.println();
//  Serial.println();
  

  // CHANGE MADE 5/19/25: allow wheels to turn in reverse

  int left_spd = spd + pid_error;
  int right_spd = spd - pid_error;
  int max_bound = MAXSPEED;
  
  if (left_spd < 0) {
    left_spd = -left_spd;
    max_bound = MAXSPEED; // this caps the absolute value of the negative speed to be the same as the max of the positive speed
    digitalWrite(left_dir_pin, HIGH);
  } else {
    digitalWrite(left_dir_pin, LOW);
    max_bound = MAXSPEED;
  }

  if (right_spd < 0) {
    right_spd = -right_spd;
    digitalWrite(right_dir_pin, HIGH);
    max_bound = MAXSPEED;
  } else {
    digitalWrite(right_dir_pin, LOW);
    max_bound = MAXSPEED;
  }

  
  //+error: center of car too much to the left (+pid_error to left)
  //-error: center of car too much to the right (+pid_error to right)
  //5/19/25: ADDED CONSTRAIN
  if (turn_at_end) {
    digitalWrite(right_dir_pin, LOW);
    digitalWrite(left_dir_pin, HIGH);
    left_spd = MAXSPEED;
    right_spd = MAXSPEED;
  }
  analogWrite(left_pwm_pin,constrain(left_spd, 0, max_bound));
  analogWrite(right_pwm_pin,constrain(right_spd, 0, max_bound));  // the bound changes depending on if its reversing or not
  
}

  
 
//  int leftSpd = 70;

//  ECE3_read_IR(sensorValues);

//  analogWrite(left_pwm_pin,leftSpd);

// 
  
//  ECE3_read_IR(sensorValues);

  // digitalWrite(LED_RF, HIGH);
  // delay(250);
  // digitalWrite(LED_RF, LOW);
  // delay(250);
    
}