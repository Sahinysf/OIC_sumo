#include <Wire.h>
#include <VL53L0X.h>
#include <SparkFun_TB6612.h>

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#include <QTRSensors.h>

QTRSensors qtr;
QTRSensors qtr2;

const uint8_t SensorCount = 8;
int sensorValues1[SensorCount];
int sensorValues2[SensorCount];

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

int a, b, c, d, e, f;

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
int full_speed = 255;
int small_speed = 190;
uint16_t position1;
uint16_t position2;

unsigned long CurrentTime;
unsigned long ChaosTime;
boolean haos;
int distance = 200;
int distance2 = 600;

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

  delay(100);
  Wire.begin();

  Serial.begin(115200);

  digitalWrite(xshut_1, HIGH);
  //delay(100);
  //Serial.println("00");
  laser_1.init(true);
  //Serial.println("01");
  //delay(100);
  laser_1.setAddress((uint8_t)01);
  //Serial.println("02");

  digitalWrite(xshut_2, HIGH);
  //delay(100);
  laser_2.init(true);
  //Serial.println("03");
  //delay(100);
  laser_2.setAddress((uint8_t)02);
  //Serial.println("04");

  digitalWrite(xshut_3, HIGH);
  //delay(100);
  laser_3.init(true);
  //Serial.println("05");
  //delay(100);
  laser_3.setAddress((uint8_t)03);
  //Serial.println("06");

  digitalWrite(xshut_4, HIGH);
  //delay(100);
  //Serial.println("07");
  laser_4.init(true);
  //Serial.println("08");
  //delay(100);
  laser_4.setAddress((uint8_t)04);
  //Serial.println("09");

  digitalWrite(xshut_5, HIGH);
  //delay(100);
  //Serial.println("10");
  laser_5.init(true);
  //Serial.println("11");
  //delay(100);
  laser_5.setAddress((uint8_t)05);
  //Serial.println("12");

  digitalWrite(xshut_6, HIGH);
  //delay(100);
  //Serial.println("13");
  laser_6.init(true);
  //Serial.println("14");
  //delay(100);
  laser_6.setAddress((uint8_t)06);
  //Serial.println("15");

  //Serial.println("addresses set");

  laser_1.startContinuous();
  laser_2.startContinuous();
  laser_3.startContinuous();
  laser_4.startContinuous();
  laser_5.startContinuous();
  laser_6.startContinuous();



  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A1, A3, A5, A7, A9, A11, A13, A15
  }, SensorCount);
  qtr.setEmitterPin(33);
  qtr2.setTypeAnalog();
  qtr2.setSensorPins((const uint8_t[]) {
    A0, A2, A4, A6, A8, A10, A12, A14
  }, SensorCount);
  qtr2.setEmitterPin(32);

  millis();
}

void loop() {
  laser_function();
  //color_function();
}

void color_function() {
  //colors();
  int max = 4;
  int cnt_1 = 0;
  int cnt_2 = 0;
  qtr.read(sensorValues1);
  qtr2.read(sensorValues2);

  for (uint8_t i = 1; i < SensorCount; i++)
  {
    if (sensorValues1[i] > 400) {
      cnt_1++;
    }
    if (sensorValues2[i] > 400) {
      cnt_2++;
    }
    if (cnt_1 >=  max) {
      haos = false;
      backward();
      break;
    }
    if (cnt_2 >=  max) {
      haos = false;
      forward();
      break;
    }

  }


}



void laser_function() {
  //lasers();,
  a = laser_1.readRangeContinuousMillimeters();
  Serial.print(a);
  Serial.print(" ");

  b = laser_2.readRangeContinuousMillimeters();
  Serial.print(b);
  Serial.print(" ");

  c = laser_3.readRangeContinuousMillimeters();
  Serial.println(c);
  Serial.print(" ");

  d = laser_4.readRangeContinuousMillimeters();
  //Serial.print(d);
  //Serial.print(" ");

  e = laser_5.readRangeContinuousMillimeters();
  //Serial.print(e);
  //Serial.print(" ");

  f = laser_6.readRangeContinuousMillimeters();
  //Serial.println(f);


  if ((d <= distance& e <= distance) || (e <= distance2& f <= distance2)) {
    forward();
    haos = false;
    /*
      if ((d > distance && e > distance && f > distance && a > distance && b > distance && c > distance)) {
      right();
      haos = true;
      CurrentTime = millis();
      if (((millis() - CurrentTime) >= 5664) && (haos == true)) {
        haos_moving();
        haos = false;
      }
      }
    */
  }
  else if ((a <= distance & b <= distance2) || (b <= distance2 & c <= distance2)) {

    backward();
    haos = false;
  }
  else if (d <= distance || a <= distance) {
    small_left();
    haos = false;

  }
  else if (f <= distance2 || c <= distance2) {
    small_right();
    haos = false;
  }
  else {
    right();
    haos = true;
  }

}


void haos_moving() {
  forward();
  ChaosTime = millis();
  if ((millis() - ChaosTime) >= 1000) {
    small_right();
    ChaosTime = millis();
    if ((millis() - ChaosTime) >= 1000) {
      small_right();
      ChaosTime = millis();
      if ((millis() - ChaosTime) >= 1000) {
        small_right();
        ChaosTime = millis();
        if ((millis() - ChaosTime) >= 1000) {
          small_right();
          ChaosTime = millis();
          if ((millis() - ChaosTime) >= 1000) {
            small_left();

          }
        }
      }
    }
  }

}




void forward() {
  //if (a<=650 && b<=650 && c<=650){
  Serial.println("forward");
  forward(motor1, motor2, full_speed);
  forward(motor3, motor4, full_speed);
  //Serial.println("FORWARD");
}

void small_right() {
  Serial.println("smallright");
  forward(motor1, motor2, full_speed);
  forward(motor3, motor4, small_speed);
}
void small_left() {
  Serial.println("smallleft");
  forward(motor3, motor4, full_speed);
  forward(motor1, motor2, small_speed);
}

void backward() {
  Serial.println("backward");
  back(motor1, motor2, full_speed);
  back(motor3, motor4, full_speed);
}

void stopp() {
  Serial.println("stop");
  brake(motor1, motor2);
  brake(motor3, motor4);
}

void right() {
  Serial.println("right");
  forward(motor1, motor2, full_speed);
  back(motor3, motor4, full_speed);

}

void left() {
  Serial.println("left");
  forward(motor3, motor4, full_speed);
  back(motor1, motor2, full_speed);
}
