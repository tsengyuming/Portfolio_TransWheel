#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#define address 0x1E
const int ENA = 11 ;
const int IN1 = 4 ;
const int IN2 = 2 ;
const int ENB = 3 ;
const int IN3 = 5 ;
const int IN4 = 6 ; 
//const int trigger = 9 ;
//const int echo = 10;
//const int Trigger = 13 ;
//const int Echo = 14;
//const int max_read = 300;
//const int no_good = 15 ;
//const int fixed_angle=30;
int Left_SENSOR = 7;
int Right_SENSOR = 8;
//int read_ahead; //actual distance
int pos=180;
int weathertransform=0;
int k=0;
int m=0;
int n=0;
int X=0;
Servo sensorStation1;
Servo sensorStation2;
Servo sensorStation3;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
sensorStation1.attach(A2);
sensorStation2.attach(A3);
Wire.begin();
//Put the HMC5883 IC into the correct operating mode
Wire.beginTransmission(address); //open communication with HMC5883
Wire.write(0x02); //select mode register
Wire.write(0x00); //continuous measurement mode
Wire.endTransmission();
pinMode(ENA, OUTPUT);
pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(ENB, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);
//pinMode(trigger, OUTPUT);
//pinMode(echo, INPUT);
pinMode(Left_SENSOR, INPUT);
pinMode(Right_SENSOR, INPUT);
sensorStation1.write(180);
sensorStation2.write(0);
delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
goForward();
delay(1800);
//Serial.println("AHEAD:");
// Serial.println(read_ahead);
fastStop();
for(pos = 180; pos > 0; pos -= 1)
  {
    sensorStation1.write(pos);
    sensorStation2.write(180-pos);
    if(pos>60&&pos<150)
      {
        goBackward();
        //delay(15);
      }
    else
        fastStop();
    delay(15);
  }
fastStop();
delay(500);
goForwardlow();
delay(3500);
weathertransform=1;

Serial.println("weathertransform:");
Serial.println(weathertransform); 
}

fastStop();
{
for(pos = 0; pos <180; pos +=2)
  {
    sensorStation1.write(pos);
    sensorStation2.write(180-pos);
    delay(15);
    if(pos>30&&pos<120)
      {
        golowForward();
        //delay(15);
      }
    else
        fastStop();
    delay(15);
   }
goFinalBackward();
delay(1300);
fastStop();
while (1)
  {
    if (readleftDistance() == 0)
      {
        turnRight();
        delay(50);
      }
    else if (readrightDistance() == 0)
      {
        turnLeft();
        delay(50);
      }
    else
      {
        goForward();
      }
  }
}

void fastStop()
{
  Serial.println("STOP");
  digitalWrite(ENA, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(ENB, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void goForward()
{
  Serial.println("FORWARD");
  analogWrite(ENA,255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB,255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void goForwardlow()
{
  Serial.println("FORWARDlow");
  analogWrite(ENA,255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB,255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void golowForward()
{
  Serial.println("FORWARDlow");
  analogWrite(ENA,30);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB,30);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void goFinalBackward()
{
  Serial.println("BACKWARD");
  analogWrite(ENA,200);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB,200);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void goBackward()
{
  Serial.println("BACKWARD");
  analogWrite(ENA,55);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB,55);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft()
{
  Serial.println("LEFT");
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, 255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight()
{
  Serial.println("RIGHT");
  analogWrite(ENA, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, 0);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

int readleftDistance()
{
  int leftDistance = digitalRead(Left_SENSOR);
  Serial.println(leftDistance);
  return leftDistance;
}

int readrightDistance()
{
  int RightDistance = digitalRead(Right_SENSOR);
  Serial.println(RightDistance);
  return RightDistance;
}
