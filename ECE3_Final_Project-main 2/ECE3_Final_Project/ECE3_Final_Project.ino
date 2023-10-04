// Motor Check code.
/*
 * This sketch tests for proper operation of the project car's motors.
 * The nominal motor PWM is 80, but the baseSpeed variable can be 
 * changed to any other value from 0 to 255.
 * 
 * The car moves forward for 800 encoder counts, then back for 800 encoder counts. It
 * repeats this oscillatory motion until the car is removed from the track.
 * 
 * The car startup and stop functions are gradually changed by the 
 *  ChangeBaseSpeed() function. This is meant to reduce the stress on the plastic
 * gear sets in the motors, prolonging motor life.
 */
 
#include <ECE3.h>
#include <stdio.h>


const int left_nslp_pin=31;
const int right_nslp_pin=11;
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int blinky_pin = 75;

const int LED_RF = 41;
int baseSpeed = 130;
int distance = 800;

// int currRightSpd = baseSpeed;
// int currLeftSpd = baseSpeed;

int prevRightSpd = baseSpeed;
int prevLeftSpd = baseSpeed;
int prevError = 0;

int rightEncoder=0;
uint16_t sensorValues[8];

float minimumVals[8] = {643 ,690 , 713, 597, 620, 620, 667, 738};
//float maximumVals[8] = {783, 804, 865.4, 824, 993, 1054.4,  1302,  1735.2};

bool isStopped = false;

bool phantomCheck = false;

bool donut = false;

int donutDistance = 425;

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

  //delay(200);
  delay(2000);

  ChangeBaseSpeed(0,80); // Start car up
  
  
}

void loop() {
  rightEncoder=getEncoderCount_right();
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


  /*
   * 
   * for the first second we want kp and kd to be kinda high and base speed to be 100
   * 
   * for the next 4 seconds we want kp and kd to be around 0.01 and 0.001 and base speed to be 200 ish
   * 
   * for the next second we want kp and kd to be kinda high again and base speed to be 100
   * 
   * for the next 2 seconds we want kp and kd to be around 0.01 and 0.001 and base speed about 200
   * 
   * turn and do the same thing
   * 
   */

// kp is how much it turns
  //when baseSpeed is 40
//  float kp = 0.015;
//  float kd = 0.0015;

//float kp = 0.01;
//  float kd = 0.001;
//
//
//  float testKP = 0.08;
//  float testKD = 0.008;

    float testKP = 0.04;
    float testKD = 0.4;
    //int testSpeed = 90;
    //baseSpeed=100;


//  // this is slow for curve
////  if((millis() <= 1750) || (millis() >= 3500 && millis() <=5500) || ( millis() >= 7000 && millis() <= 8000)) 
  //swap this with encodercount, each wheel rev is approx 22 cm, (180,80, .03, .4)

  if(!donut)
  {
    // middle curve
  if((rightEncoder <= 450) || (rightEncoder >= 2100 && rightEncoder <= 3600) || (rightEncoder >= 11250 && rightEncoder <= 12500))
  {
    testKP = 0.06;
    testKD = 0.4;
   baseSpeed = 80;
  }
  //beginning curve and end curve
  else if((rightEncoder >= 3600 && rightEncoder <= 10000))
  {
      testKP = 0.1;
    testKD = 0.4;
   baseSpeed = 100;
  }
//  else if((rightEncoder >= 3550 && rightEncoder <= 3800))
//  {
//    testKP = 0.03;
//    testKD = 0.4;
//    baseSpeed = 210;
//  }
//////  else if ((rightEncoder >= 2250 && rightEncoder <= 4000)){
//////    testKP = 0.015;
//////    testKD = 0.0015;
//////    baseSpeed = 40;
//////  }

  // straight places
  else 
  {
     testKP = 0.03;
    testKD = 0.4;
    baseSpeed = 230;
  }
  }

  else if(donut)
  {
    // middle curve
    if( (rightEncoder >= 1700 && rightEncoder <= 2900) )
    {
    testKP = 0.06;
    testKD = 0.4;
   baseSpeed = 80;
    }

  //beginning and end curve
  else if(rightEncoder <= 900  )
  {
    testKP = 0.1;
    testKD = 0.4;
   baseSpeed = 100;
  }
  else if (rightEncoder >= 4750)
  {
    testKP = 0.1;
    testKD = 0.4;
   baseSpeed = 120;
  }
  //straights
  else 
  {
     testKP = 0.03;
    testKD = 0.4;
    baseSpeed = 230;
  }
  } 


  /*
   *   if(!donut)
  {
  if(rightEncoder <= 550 || (rightEncoder >= 2000 && rightEncoder <= 3500) ||  (rightEncoder >= 4000 && rightEncoder <= 10000) || (rightEncoder >= 11250 && rightEncoder <= 12500))
  {
    testKP = 0.06;
    testKD = 0.4;
   baseSpeed = 80;
  }
//////  else if ((rightEncoder >= 2250 && rightEncoder <= 4000)){
//////    testKP = 0.015;
//////    testKD = 0.0015;
//////    baseSpeed = 40;
//////  }
  else 
  {
     testKP = 0.03;
    testKD = 0.4;
    baseSpeed = 180;
  }
  }
  else if(donut)
  {
    if(rightEncoder <= 900 || (rightEncoder >= 1700 && rightEncoder <= 2900) ||  (rightEncoder >= 5000))
  {
    testKP = 0.06;
    testKD = 0.4;
   baseSpeed = 80;
  }
  else 
  {
     testKP = 0.03;
    testKD = 0.4;
    baseSpeed = 200;
  }
  }
   */

//0.0225, 0.012


/*
 * THIS WORKS MOSTLY
 * testKP = 0.08;
    testKD = 0.6;
    baseSpeed = 240;
 */
//    testKP = 0.08;
//    testKD = 0.6;
//    baseSpeed = 240;


//  float num = kp * errorValue +  kd*(prevError + errorValue);



float num = testKP * errorValue +  testKD*(errorValue - prevError );
  //if(num > 10) num = 10;
  //if(num < -10) num = -10;
//  Serial.print(errorValue);
//  delay(30); 
//  Serial.println();

//  changeWheelSpeeds(left_pwm_pin, left_pwm_pin + num , right_pwm_pin , right_pwm_pin - num );

  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);
 
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
  if(sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7]  > 11000)
  {
      
      // if you want to do the stop and donut
      if(!donut && !phantomCheck)
      {
        donut = true;

        //ChangeBaseSpeed(baseSpeed,0);

        resetEncoderCount_left();
        resetEncoderCount_right();

        
        
        digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
        digitalWrite(right_dir_pin,HIGH);
        analogWrite(left_pwm_pin,baseSpeed);    
        analogWrite(right_pwm_pin,baseSpeed); 

        while(getEncoderCount_left() < donutDistance) {
          }

          //ChangeBaseSpeed(baseSpeed,0);

        digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
        digitalWrite(right_dir_pin,LOW);

        analogWrite(left_pwm_pin,0);    
        analogWrite(right_pwm_pin,0); 

        //ChangeBaseSpeed(0,baseSpeed);

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
}
