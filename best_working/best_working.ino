const byte analogPinR = 4;  
const byte analogPinL = 3;
const byte jpR = 6;
const byte jpL = 5;

int positionValL,readValL,positionValR,readValR;
float derivativeL=0,proportionalL=0,derivativeR=0,proportionalR=0,errR=0,errL=0,err=0,err_rot=0;

int kp=12,kd=5;                          //PID constants

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
pinMode(analogPinR,INPUT);
pinMode(jpR,INPUT);
pinMode(analogPinL,INPUT);
pinMode(jpL,INPUT);

}


  void loop()
{

err_cR(); // for finding the error //// deviation of line from the center of the center 
err_cL();

err=(errR+errL);                                                                                                                                                                                                                                                                                      
err_rot = errL-errR;

if(positionValR<71 && positionValL <71)
{
Left(1500,err,err_rot);
/*
  if (positionValF>20 and positionValF<55){
  Serial.println("Stage1");
  Straight(2000,err/2,err_rot/2);}
  else if (positionValF <20 || positionValF>55 && positionValF<70){
  Serial.println("Stage2");
  Straight(1500,err*2,err_rot*2);}
  else if(positionValF>70){
  Straight(0,0,0);
  Serial.println("Stage3");}
*/
}
 }


          //// Assume motors are adjusted such that on giving +ve speed they move forward \\\\


void Left(int Speed, float err , float err_rot)
{
 i=Speed;
   K1.s(i+err- err_rot);
  K2.s(-i+err+ err_rot);
  J1.s(i+err+err_rot);
  J2.s(-i+err- err_rot);
  
  
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
 
 void err_cR(){
  float tempR = ((float)analogRead(analogPinR) / 921) * 70;  
  derivativeR = (tempR + (-35) - proportionalR);
    proportionalR = tempR + (-35);  
    errR = (kp * proportionalR) + (kd * derivativeR);
    readValR = analogRead(analogPinR);     
    positionValR = ((float)readValR / 921) * 70;
    }

     void err_cL(){
  float tempL = ((float)analogRead(analogPinL) / 921) * 70;  
  derivativeL = (tempL + (-35) - proportionalL);
    proportionalL = tempL + (-35);  
    errL = (kp * proportionalL) + (kd * derivativeL);
    readValL = analogRead(analogPinL);     
    positionValL = ((float)readValL / 921) * 70;
    Serial.print(positionValL);
    }
