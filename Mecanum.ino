const byte analogPin = 1;
const byte jp = 4;
int positionVal, readVal, v = 150;
float derivative = 0, proportional = 0, err = 0, kp = 20, kd = 12;
#include < Sabertooth.h > 
#include < SPI.h > 
#include < Kangaroo.h > 
#include < Encoder.h >
  KangarooSerial J(Serial3);

KangarooChannel J1(J, '1'); // LEKT
KangarooChannel J2(J, '2'); //RIGHT
KangarooSerial K(Serial2);

KangarooChannel K1(K, '1'); // RIGHT
KangarooChannel K2(K, '2'); //LRKT
int l = 40 * 40;
int i = 0;

void setup() {
  Serial.begin(115200);
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
  pinMode(analogPin, INPUT);
  pinMode(jp, INPUT);

}

////// K1-top_left  K2-top_right  L1-bottom-left  L2-bottom_right  \\\\\\\

void loop() {

  err_c(); // for finding the error //// deviation of line from the center of the center 
  Serial.println(err);
  /*
  //Straight(1000,err);
  //check(500);
    if (positionVal>20 and positionVal<55){
    Serial.println("Stage1");
    
    Straight(5000,err/2);}
    else if (positionVal <20 || positionVal>55 && positionVal<70){
    Serial.println("Stage2");
    Straight(4000,err*6);}
    else if(positionVal>70){
    Straight(2200,0);
    Serial.println("Stage3");}

  }
  */
  if (positionVal > 30 and positionVal < 55) {
    rampp(i);
    Serial.println("Forward");
    Straight(i, err / 5);
  } 
  else if (positionVal < 30) {
    Serial.println("LEFT");
    StraightL(i, err * 2);
  } 
  else if (positionVal > 55 && positionVal < 70) {
    rampp(i);
    StraightR(i, err * 2);
    Serial.println("RIGHT");
  } 
  else  
  {
      if (i < 40) {
      i = 0;
    } 
    else {
      i = i - 40;
    }
    Straight(i, 0);
    Serial.println("out of bounds");
  }
  
  
}

//// Assume motors are adjusted such that on giving +ve speed they move forward \\\\
void rampp(int i) {
  if (i < 3000)
    {i = i + 50;}
}
void check(int Speed) {
  i = Speed;
  K1.s(i);
  K2.s(-i);
  J1.s(-i);
  J2.s(i);

}

void StraightL(int Speed, float err) {
  i = Speed;
  K1.s(i - err);
  K2.s(i + err);
  J1.s(i - err);
  J2.s(i + err);

}
void StraightR(int Speed, float err) {
  i = Speed;
  K1.s(i + err);
  K2.s(i - err);
  J1.s(i + err);
  J2.s(i - err);

}
void Straight(int Speed, float err) {
  i = Speed;
  K1.s(i - err);
  K2.s(i + err);
  J1.s(i + err);
  J2.s(i - err);

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

void err_c() {
  float temp = ((float) analogRead(analogPin) / 921) * 70;
  derivative = (temp + (-35) - proportional);
  proportional = temp + (-35);
  err = (kp * proportional) + (kd * derivative);
  readVal = analogRead(analogPin);
  positionVal = ((float) readVal / 921) * 70;
}
