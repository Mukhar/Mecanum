const byte analogPinF = 1;  
const byte analogPinB = 2;
const byte jpF = 4;
const byte jpB = 5;

int positionVal,readVal,v=150;
float derivative=0,proportional=0,errF=0,errB=0,err=0,err_rot=0;

int kp=20,kd=12;                          //PID constants

#include <Sabertooth.h>
#include <SPI.h>
#include <Kangaroo.h>
#include <Encoder.h>
KangarooSerial  J(Serial3);

KangarooChannel J1(J,'1');  // LEKT
KangarooChannel J2(J,'2');//RIGHT
KangarooSerial  K(Serial2);

KangarooChannel K1(K,'1');// RIGHT
KangarooChannel K2(K,'2');//LRKT
int l= 40*40;
int i=0;

  void setup()
{ Serial.begin(115200);
  Serial3.begin(115200);
  Serial2.begin(115200);    
  K1.start();
  K1.home().wait();
  Serial.println("K1 started");
  K2.start();
  K2.home().wait();  
  Serial.println("K2 started");
  J1.start();
  J1.home().wait(); 
  Serial.println("J1 started");
  J2.start();
  J2.home().wait();
  Serial.println("J2 started");

  Serial.begin(115200);
pinMode(analogPinF,INPUT);
pinMode(jpF,INPUT);
pinMode(analogPinB,INPUT);
pinMode(jpB,INPUT);

}


  void loop()
{

err_cF(); // for finding the error //// deviation of line from the center of the center 
err_cB();

err=errF+errB;
err_rot = errF-errB;

Straight(1000,err,err_rot);

  if (positionVal>20 and positionVal<55){
  Serial.println("Stage1");
  Straight(2000,err/2,err_rot/2);}
  else if (positionVal <20 || positionVal>55 && positionVal<70){
  Serial.println("Stage2");
  Straight(1500,err*2,err_rot*2);}
  else if(positionVal>70){
  Straight(0,0,0);
  Serial.println("Stage3");}
 }


          //// Assume motors are adjusted such that on giving +ve speed they move forward \\\\

void check(int Speed)
{
 i=Speed;
  K1.s(i);
  K2.s(-i);
  J1.s(-i);
  J2.s(i);
  
   }

void Straight(int Speed, float err , float err_rot)
{
 i=Speed;
  K1.s(i - err - err_rot);
  K2.s(i + err + err_rot);
  J1.s(i - err + err_rot);
  J2.s(i + err - err_rot);
  
   }

 void Right(int Speed)
{
  
  K1.s(i);
  K2.s(-i);
  J1.s(-i);
  J2.s(i);
}

 void Left(int Speed)
{
  for(int i=0 ; i <= Speed ; i+=10 )
  {
  K1.s(-i);
  K2.s(i);
  J1.s(i);
  J2.s(-i);
  }
  delay(1000);
 }

 /*
 void for_going_at_any_angle(int Speed,int θ)
{
  for(int i=0 ; i <= Speed ; i+=10 )
  {
  K1.s(i*((cos θ)-(sin θ))); 
  K2.s(i*((cos θ)+(sin θ)));
  L1.s(i*((cos θ)+(sin θ)));
  L2.s(i*((cos )-(sin θ)));
  }
  delay(1000);
 }
 */
 
 void err_cF(){
  float temp = ((float)analogRead(analogPinF) / 921) * 70;  
  derivative = (temp + (-35) - proportional);
    proportional = temp + (-35);  
    errF = (kp * proportional) + (kd * derivative);
    readVal = analogRead(analogPinF);     
    positionVal = ((float)readVal / 921) * 70;
    }

     void err_cB(){
  float temp = ((float)analogRead(analogPinB) / 921) * 70;  
  derivative = (temp + (-35) - proportional);
    proportional = temp + (-35);  
    errB = (kp * proportional) + (kd * derivative);
    readVal = analogRead(analogPinB);     
    positionVal = ((float)readVal / 921) * 70;
    }
