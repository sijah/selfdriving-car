//self driving car 
// written by sijah
// feel free to edit give respect
// thanks to all friends @awh



#include "HMC5883L.h"


#include <I2Cdev.h>

#include <Wire.h>

#include <SoftwareSerial.h>
#include <TinyGPS.h>



#include <Servo.h>
#include <NewPing.h>
int mo1 = 2, mo2 = 3, mo3 = 4, mo4 = 5;
int vcc = 13;
#define trig1  9
#define echo1  8
#define MAX_DISTANCE 200
//int trig2 = 11;
//int echo2 = 10;
//int gnd = 4;
long cm1;//,cm2;
int i = 0;
NewPing sonar(trig1, echo1, MAX_DISTANCE);
//NewPing sonar2(trig2,echo2,250);
int f1 = 0;
float heading;
TinyGPS gps;
SoftwareSerial ss(11, 12);
void gpshead();
void stopp();
void rightturn();
void gostraight();
void leftturn();
void turn();
//Servo myservo;
float x2lon = radians(00000000), x2lat = radians(000000000); //enter ur final location here as lon and lat
HMC5883L mag;
int16_t mx, my, mz;
float head, distance = 0.0;
void setup() {
  Wire.begin();
  mag.initialize();
  ss.begin(9600);
  Serial.begin(9600);
  pinMode (vcc, OUTPUT);
  // pinMode (gnd,OUTPUT);
  pinMode(mo1, OUTPUT);
  pinMode(mo2, OUTPUT);
  pinMode(mo3, OUTPUT);
  pinMode(mo4, OUTPUT);
  //myservo.attach(7);
}
void loop() {
  digitalWrite(vcc, HIGH);
  if (f1 != 0)
    gostraight();
  cm1 = sonar.ping_cm();
  delay(50);
  cm1 = sonar.ping_cm();
  Serial.print("cm");
  Serial.print(cm1);
  if (cm1 < 20 && cm1 > 0)
  {
    mag.getHeading(&mx, &my, &mz);
    heading = atan2(my, mx);
    if (heading < 0)
      heading += 2 * M_PI;
    heading = heading * 180 / M_PI;
    if ((i % 2) == 0)
    { head = heading + 100;
      if (head > 360)
        head = head - 360;
      leftturn();
      /*myservo.write(180);
        delay(2000);*/
      //myservo.write(180);
      //i++;
    }
    else
    { head = heading - 100;
      if (head < 0)
        head = head + 360;
      rightturn();
      /*myservo.write(0);
        delay(1000);*/
      // myservo.write(0);
      //i++;
    }
    while ((heading > head + 8) || (heading < head - 8)) // this loop turns the bot till its facing (head)degrees east of north
    {
      turn();
      delay(5);
      mag.getHeading(&mx, &my, &mz);
      heading = atan2(my, mx);
      if (heading < 0)
        heading += 2 * M_PI;
      heading = heading * 180 / M_PI;
    }
    stopp();
    delay(100);
  }
  else {
    /*cm2=sonar2.ping_cm();
      delay(50);
      cm2+=sonar2.ping_cm();
      cm2=cm2/2;*/
    //while((cm1<50)&&(cm1>0))
    gostraight();
    delay(50);
    //cm2=sonar2.ping_cm();
    //delay(50);
    //cm2+=sonar2.ping_cm();
    //cm2=cm2/2;}
    delay(500);
    stopp();

    do {
      gpshead();
    } while (distance == 0.0);
    if (distance < 15)
      while (1)
        stopp();
    if (f1 == 0)
    { mag.getHeading(&mx, &my, &mz);
      heading = atan2(my, mx);
      if (heading < 0)
        heading += 2 * M_PI;
      heading = heading * 180 / M_PI;


      while ((heading > head + 8) || (heading < head - 8))
      {
        turn();
        delay(5);
        mag.getHeading(&mx, &my, &mz);
        heading = atan2(my, mx);
        if (heading < 0)
          heading += 2 * M_PI;
        heading = heading * 180 / M_PI;
      }
      f1 = 4;
    }
    f1--;
  }
}
void turn()
{ float tur = heading - head;
  //if (tur < 0.0 && tur > -180 )
  // { //if (tur > -180.0)
  //rightturn();
  // }
  //  else {

  //leftturn();

  if (tur < 0.0)
  { if (tur > -180.0)
      rightturn();
    else
      leftturn();
  }
  else
  { if (tur < 180.0)
      leftturn();
    else rightturn();
  }
}
void leftturn()
{ digitalWrite(mo1, LOW);
  digitalWrite(mo2, LOW);
  digitalWrite(mo3, HIGH);
  digitalWrite(mo4, LOW);
  delay(10);
}
void stopp()
{
  digitalWrite(mo1, LOW);
  digitalWrite(mo2, LOW);
  digitalWrite(mo3, LOW);
  digitalWrite(mo4, LOW);
}

void rightturn()
{ digitalWrite(mo1, LOW);
  digitalWrite(mo2, HIGH);
  digitalWrite(mo3, LOW);
  digitalWrite(mo4, LOW);
}
void gostraight()
{ digitalWrite(mo1, LOW);
  digitalWrite(mo2, HIGH);
  digitalWrite(mo3, HIGH);
  digitalWrite(mo4, LOW);
}
void gpshead()
{
  bool newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData)
  {
    float flat1, flon1;
    unsigned long age;
    gps.f_get_position(&flat1, &flon1, &age);
    flon1 = radians(flon1);  //also must be done in radians
    flat1 = radians(flat1);  //also must be done in radians
    head = atan2(sin(x2lon - flon1) * cos(x2lat), cos(flat1) * sin(x2lat) - sin(flat1) * cos(x2lat) * cos(x2lon - flon1));
    head = head * 180 / 3.1415926535; // convert from radians to degrees
    float dist_calc = 0;
    float diflat = 0;
    float diflon = 0;
    diflat = x2lat - flat1; //notice it must be done in radians
    diflon = (x2lon) - (flon1); //subtract and convert longitudes to radians
    distance = (sin(diflat / 2.0) * sin(diflat / 2.0));
    dist_calc = cos(flat1);
    dist_calc *= cos(x2lat);
    dist_calc *= sin(diflon / 2.0);
    dist_calc *= sin(diflon / 2.0);
    distance += dist_calc;
    distance = (2 * atan2(sqrt(distance), sqrt(1.0 - distance)));
    distance *= 6371000.0; //Converting to meters
    if (head < 0) {
      head += 360; //if the heading is negative then add 360 to make it positive
    }
  }
  else
  {
    head = 0.0;
    distance = 0.0;
  }
}
