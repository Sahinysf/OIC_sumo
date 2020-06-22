#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFun_TB6612.h>

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#include <QTRSensors.h>

QTRSensors qtr;
QTRSensors qtr2;

const uint8_t SensorCount = 8;
uint16_t sensorValues1[SensorCount];
uint16_t sensorValues2[SensorCount];

VL53L0X laser_1;
VL53L0X laser_2;
VL53L0X laser_3;
VL53L0X laser_4;
VL53L0X laser_5;
VL53L0X laser_6;

#define  xshut_1 14
#define  xshut_2 15
#define  xshut_3 16
#define  xshut_4 17
#define  xshut_5 18
#define  xshut_6 19

int a,b,c,d,e,f;

#define L1_LED 33
#define L2_LED 32 

#define PWM_1 8
#define PWM_2 9

#define M1_a2 2
#define M1_a1 3
#define M1_b1 4
#define M1_b2 5
#define STBY 20



#define M2_a2 10
#define M2_a1 11
#define M2_b1 12
#define M2_b2 13 

const int offsetA = 1;
const int offsetB = 1;

#define buzzer 46

boolean Flag1 = true;

  Motor motor1 = Motor(M1_a1, M1_a2, PWM_1, offsetA, STBY);
  Motor motor2 = Motor(M1_b1, M1_b2, PWM_2, offsetB, STBY);
  Motor motor3 = Motor(M2_a1, M2_a2, PWM_1, offsetA, STBY);
  Motor motor4 = Motor(M2_b1, M2_b2, PWM_2, offsetB, STBY);

uint16_t position1; 
uint16_t position2;

void setup() {
  pinMode(xshut_6, OUTPUT);
  pinMode(xshut_5, OUTPUT);
  pinMode(xshut_4, OUTPUT);
  pinMode(xshut_3, OUTPUT);
  pinMode(xshut_2, OUTPUT);
  pinMode(xshut_1, OUTPUT);

  digitalWrite(xshut_1, LOW);
  digitalWrite(xshut_2, LOW);
  digitalWrite(xshut_3, LOW);
  digitalWrite(xshut_4, LOW);
  digitalWrite(xshut_5, LOW);
  digitalWrite(xshut_6, LOW);

  delay(50);
  Wire.begin();

  Serial.begin(9600);

  digitalWrite(xshut_1, HIGH);
  delay(50);
  Serial.println("00");
  laser_1.init(true);
  Serial.println("01");
  delay(50);
  laser_1.setAddress((uint8_t)01);
  Serial.println("02");

  digitalWrite(xshut_2, HIGH);
  delay(50);
  laser_2.init(true);
  Serial.println("03");
  delay(50);
  laser_2.setAddress((uint8_t)02);
  Serial.println("04");

  digitalWrite(xshut_3, HIGH);
  delay(50);
  laser_3.init(true);
  Serial.println("05");
  delay(50);
  laser_3.setAddress((uint8_t)03);
  Serial.println("06");  

  digitalWrite(xshut_4, HIGH);
  delay(50);
  Serial.println("07");
  laser_4.init(true);
  Serial.println("08");
  delay(50);
  laser_4.setAddress((uint8_t)04);
  Serial.println("09");   

  digitalWrite(xshut_5, HIGH);
  delay(50);
  Serial.println("10");
  laser_5.init(true);
  Serial.println("11");
  delay(50);
  laser_5.setAddress((uint8_t)05);
  Serial.println("12");   

  digitalWrite(xshut_6, HIGH);
  delay(50);
  Serial.println("13");
  laser_6.init(true);
  Serial.println("14");
  delay(50);
  laser_6.setAddress((uint8_t)06);
  Serial.println("15"); 

  Serial.println("addresses set");

  laser_1.startContinuous();
  laser_2.startContinuous();
  laser_3.startContinuous();
  laser_4.startContinuous();
  laser_5.startContinuous();
  laser_6.startContinuous();
  
  
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A1, A3, A5, A7, A9, A11, A13, A15}, SensorCount);
  qtr.setEmitterPin(2);

  
  
  qtr2.setTypeAnalog();
  qtr2.setSensorPins((const uint8_t[]){A0, A2, A4, A6, A8, A10, A12, A14}, SensorCount);
  qtr2.setEmitterPin(2);

delay(4350);

 /*   for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
    
  }
  
    for (uint16_t i = 0; i < 400; i++)
  {
    qtr2.calibrate();
    
  }
  */
  
   
}

void loop() {
  //if (Flag1==true){
  // put your main code here, to run repeatedly:
  //lasers();
//laser_function();
lasers();
 /* right();
  delay(472);
  stopp();
  delay(500);*/

  //Flag1=true;
  }

/*void color_function(){
  colors();
  if (position2>=1000){
    backward();
  }
  else if (position1>=1000){
    forward();
  }
}
*/


void laser_function(){
    lasers();
  if ((d<=400 && e<=400 && f<=400)&& (a>400 && b>400 && c>400)){
    forward();
    if ((d>400 && e>400 && f>400) && (a>400 && b>400 && c>400)){
      right();
    }
}
   if ((d<=400 && e>400 && f>400) && (a>400 && b>400 && c>400)){
      small_right();
      if ((d>400 && e<=400 && f>400)&& (a>400 && b>400 && c>400)){
        forward();
      }
    }
    if ((d>400 && e>400 && f<=400)&& (a>400 && b>400 && c>400)){
      small_left();
      if ((d>400 && e<=400 && f>400)&& (a>400 && b>400 && c>400)){
        forward();
      }
    }

//The other side

 else if ((a<=400 && b<=400 && c<=400)&&(d>400 && e>400 && f>400)){
    backward();
}
  else if ((a<=400 && b>400 && c>400)&&(d>400 && e>400 && f>400)){
      small_left();
      if ((a>400 && b<=400 && c>400)&&(d>400 && e>400 && f>400)){
        forward();
      }
    }
   else if ((a>400 && b>400 && c<=400)&&(d>400 && e>400 && f>400)){
      small_right();
      if ((a>400 && b<=400 && c>400)&&(d>400 && e>400 && f>400)){
        forward();
      }
    }
}



void lasers(){
  a=laser_1.readRangeContinuousMillimeters();
  //Serial.print(a);
  Serial.print(" ");

  b=laser_2.readRangeContinuousMillimeters();
  //Serial.print(b);
  Serial.print(" ");

  c=laser_3.readRangeContinuousMillimeters();
  //Serial.println(c);
  Serial.print(" ");

  d=laser_4.readRangeContinuousMillimeters();
  Serial.print(d);
  Serial.print(" ");

  e=laser_5.readRangeContinuousMillimeters();
  Serial.print(e);
  Serial.print(" ");

  f=laser_6.readRangeContinuousMillimeters();
  Serial.println(f);
}
//delay(100);
  



void colors(){
    position1 = qtr.readLineBlack(sensorValues1);
    //Serial.println(position1);
    position2 = qtr2.readLineBlack(sensorValues2);
    //Serial.println(position1);
}






void forward(){
  //if (a<=650 && b<=650 && c<=650){
      forward(motor1,motor2,150);
      forward(motor3,motor4,150);
    //Serial.println("FORWARD");
  }
  
void small_right(){ 
    forward(motor1,motor2,150);
    forward(motor3,motor4,90);}
void small_left(){
    forward(motor3,motor4,150);
    forward(motor1,motor2,90);
}

void backward(){
  back(motor1,motor2,-150);
  back(motor1,motor2,-150);
}

void stopp(){
  brake(motor1,motor2);
  brake(motor3,motor4);
}

void right(){
  forward(motor1,motor2,150);
  back(motor3,motor4,-150);
  
}

void left(){
  forward(motor3,motor4,150);
  back(motor1,motor2,-150);
}

/*void small_right(){
}

void small_left(){
}*/
