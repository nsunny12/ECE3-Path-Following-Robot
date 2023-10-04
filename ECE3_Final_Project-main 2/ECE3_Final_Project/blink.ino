#include <ECE3.h>
#include <stdio.h>


const int left_nslp_pin=31;
const int right_nslp_pin=11;
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int blinky_pin = 75;
bool lightOn=false;
unsigned long startLight;

const int LED_RF = 41;
int baseSpeed = 40;
int distance = 800;

// int currRightSpd = baseSpeed;
// int currLeftSpd = baseSpeed;

int prevRightSpd = baseSpeed;
int prevLeftSpd = baseSpeed;
int prevError = 0;

uint16_t sensorValues[8];

float minimumVals[8] = {643 ,690 , 713, 597, 620, 620, 667, 738};
//float maximumVals[8] = {783, 804, 865.4, 824, 993, 1054.4,  1302,  1735.2};

bool isStopped = false;

bool phantomCheck = false;

bool donut = false;

int donutDistance = 300;

//int phantomCheckInt = 0;


void  ChangeBaseSpeed(int initialBaseSpd, int finalBaseSpd) {
/*  
 *   This functin changes the car base speed gradually (in about 300 ms) from
 *   initialspeed to final speed. This non-instantaneous speed change reduces the 
 *   load on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int numSteps = 5;
  int pwmLeftVal = initialBaseSpd; // initialize left wheel speed 
  int pwmRightVal = initialBaseSpd;  // initialize right wheel speed 
  int deltaLeft = (finalBaseSpd-initialBaseSpd)/numSteps; // left in(de)crement
  int deltaRight = (finalBaseSpd-initialBaseSpd)/numSteps;  // right in(de)crement

//  //start count time when light turns on or off
//  int startLight;

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(60);   
  } // end for int k
} // end void  ChangeBaseSpeed

int average()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
//  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}


void setup() {
  // put your setup code here, to run once:

  ECE3_Init();

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);
  //digitalWrite(left_nslp_pin,LOW);
  //digitalWrite(right_nslp_pin,LOW);
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission

  resetEncoderCount_left();
  resetEncoderCount_right();

    startLight=0;
   ChangeBaseSpeed(0,baseSpeed); // Start car up
   digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(blinky_pin,HIGH);
  lightOn=true;
  
  
}

void loop() {

  ECE3_read_IR(sensorValues);

  int phantomCheckInt = sensorValues[0]  + sensorValues[1]  + sensorValues[2]  + sensorValues[3]  +
                      sensorValues[4] + sensorValues[5]  + sensorValues[6]  + sensorValues[7];

   if(phantomCheckInt == 20000)
    {
      phantomCheck = true;
    }

  for (unsigned char i = 0; i < 8; i++)
  {
   
    //Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
      sensorValues[i] -= (minimumVals[i] - 50);
      //sensorValues[i] *= (1000/maximumVals[i]);
      //Serial.print(sensorValues[i]);
  }
  //Serial.println();

  float errorValue = (sensorValues[0] * -8 + sensorValues[1] * -4 + sensorValues[2] * -2 + sensorValues[3] * -1 +
                      sensorValues[4] + sensorValues[5] * 2 + sensorValues[6] * 4 + sensorValues[7] * 8) / 4;

  //when baseSpeed is 40
  float kp = 0.015;
  float kd = 0.0015;

//float kp = 0.025;
//  float kd = 0.005;


  float num = kp * errorValue +  kd*(prevError + errorValue);
  //if(num > 10) num = 10;
  //if(num < -10) num = -10;
//  Serial.print(errorValue);
//  delay(30); 
//  Serial.println();

//  changeWheelSpeeds(left_pwm_pin, left_pwm_pin + num , right_pwm_pin , right_pwm_pin - num );

 
//  ChangeBaseSpeed(0,baseSpeed); // Start car up
  
  int currRightSpd=baseSpeed+num;
  int currLeftSpd=baseSpeed-num;
  analogWrite(left_pwm_pin,currLeftSpd);    
  analogWrite(right_pwm_pin,currRightSpd); 
  prevRightSpd = currRightSpd;
  prevLeftSpd = currLeftSpd;
  //delay(30); 

  //Serial.println(currLeftSpd);


  prevError = errorValue;


  // this is for the stops and donut

  // 
  if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7]  > 12000)
  {
      
      // if you want to do the stop and donut
      if(!donut && !phantomCheck)
      {
        donut = true;

        ChangeBaseSpeed(baseSpeed,0);

        resetEncoderCount_left();
        resetEncoderCount_right();

        
        
        digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
        digitalWrite(right_dir_pin,HIGH);
        analogWrite(left_pwm_pin,baseSpeed);    
        analogWrite(right_pwm_pin,baseSpeed); 

        while(average() < donutDistance) {
          }

          ChangeBaseSpeed(baseSpeed,0);

        digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
        digitalWrite(right_dir_pin,LOW);

        analogWrite(left_pwm_pin,0);    
        analogWrite(right_pwm_pin,0); 

        ChangeBaseSpeed(0,baseSpeed);

      }
      // end of the track
      else if(donut && !phantomCheck)
      {
        ChangeBaseSpeed(baseSpeed,0);
        while (true)
        {
          
        }
      }
       phantomCheck = false;

      
    }

  if(((millis()-startLight)>=1000) && lightOn==true){
    digitalWrite(blinky_pin,LOW);
    lightOn=false;
    startLight=millis();
  }
  else if(((millis()-startLight)>=2000) && lightOn==false){
    digitalWrite(blinky_pin,HIGH);
    lightOn=true;
    startLight=millis();
  }
  
}
