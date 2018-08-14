const byte analogPinF = 1;  
const byte analogPinB = 2;
const byte jpF = 4;
const byte jpB = 3;
const byte analogPinR = 4;  
const byte analogPinL = 3;
const byte jpR = 6;
const byte jpL = 5;
int j1=0,j2=0,j3=0;
int positionValL,readValL,positionValR,readValR;
float derivativeL=0,proportionalL=0,derivativeR=0,proportionalR=0,errR=0,errL=0,err1=0,err_rot1=0;
int positionValB,readValB,positionValF,readValF;
float derivativeB=0,proportionalB=0,derivativeF=0,proportionalF=0,errF=0,errB=0,err=0,err_rot=0;
int j=0 ;
int c1 =1;c2 =1;
int kp=19,kd=7;                          //PID constants
int k=0;
attachInterrupt(digitalPinToInterrupt(jpF), juF, RISING);
attachInterrupt(digitalPinToInterrupt(jpB), juB, RISING);
attachInterrupt(digitalPinToInterrupt(jpL), juL, RISING);
 void juF(){
j1=1;
 }
void juB(){
  j2=1;
 }
 void juL(){
  j3=1;
  j1=0;
 }
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
void loop(){
  err_cR();
  err_cL();
  err1=(errR+errL);                                                                                                                                                                                                                                                                                      
  err_rot1 = errL-errR;

  err_cF(); 
  err_cB();

  err=-(errF+errB);                                                                                                                                                                                                                                                                                      
  err_rot = errF-errB;

  if(j1==1)
  {
 Left(k,err1,err_rot1);
  }

   else if(j2==1)
   {
    while(positionValB<70)
    {
         Left(k/3,err1,err_rot1);

    }
    Break();
      Back(500,err1,err_rot1);

   }
   else if(j3==1)
   {
    while(positionValR<70)
    {
        Back(k/3,err1,err_rot1);

    }
    Break();
    while(positionValL>45)
    {     
    Rotate(500);
      }
}
else {
Straight(i,err,err_rot); 
}
 /*Mudit 
 */

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


void Right(int Speed, float err1 , float err_rot1)
{
 i=Speed;
   K1.s(-i+err1- err_rot1);
  K2.s(i+err1+ err_rot1);
  J1.s(-i+err1+err_rot1);
  J2.s(i+err1- err_rot1);  
}
   
void Left(int Speed, float err1 , float err_rot1)
{
 i=Speed;
   K1.s(i+err1- err_rot1);
  K2.s(-i+err1+ err_rot1);
  J1.s(i+err1+err_rot1);
  J2.s(-i+err1- err_rot1);  
}

void Straight(int Speed, float err , float err_rot)
{
 i=Speed;
  K1.s(i + err - err_rot);
  K2.s(i - err + err_rot);
  J1.s(i + err + err_rot);
  J2.s(i - err - err_rot);
}

void Back(int Speed, float err , float err_rot)
{
 i=Speed;
  K1.s(-i + err - err_rot);
  K2.s(-i - err + err_rot);
  J1.s(-i + err + err_rot);
  J2.s(-i - err - err_rot);
}   

void Rotate(int kk){
  K1.s(-kk);
  K2.s(kk);
  J1.s(kk);
  J2.s(-kk);
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
  float tempF = ((float)analogRead(analogPinF) / 921) * 70;  
  derivativeF = (tempF + (-35) - proportionalF);
    proportionalF = tempF + (-35);  
    errF = (kp * proportionalF) + (kd * derivativeF);
    readValF = analogRead(analogPinF);     
    positionValF = ((float)readValF / 921) * 70;
    }

     void err_cB(){
  float tempB = ((float)analogRead(analogPinB) / 921) * 70;  
  derivativeB = (tempB + (-35) - proportionalB);
    proportionalB = tempB + (-35);  
    errB = (kp * proportionalB) + (kd * derivativeB);
    readValB = analogRead(analogPinB);     
    positionValB = ((float)readValB / 921) * 70;
    Serial.print(positionValB);
    }
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