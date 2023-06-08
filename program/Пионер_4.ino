#include <Ultrasonic.h>
#include <Servo.h>
#include <SPI.h>
#include <Pixy.h>
#define period 2700
#define ENA 6
#define IN1 4
#define IN2 5
float distr, distl, i=0, s;
uint16_t blocks, timer = 600;
char buf[32]; 
static int f;
uint8_t power;
Ultrasonic ultrasonicr(10, 9);
Ultrasonic ultrasonicl(8, 7);
Servo servo;
Pixy pixy;
void setup() {
  Serial.begin(9600);
  servo.attach(A0);
  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  servo.write(79);
  pixy.init();
  power=110;
}
void PID();
void Time();
void lmove();
void rmove();
void loop() {  
  analogWrite(ENA, power);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  distr = ultrasonicr.distanceRead();
  distl = ultrasonicl.distanceRead();   
  blocks = pixy.getBlocks();  
  PID();
  for (f=0;f<blocks;f++){
    if(pixy.blocks[f].signature==1){rmove();}
    if(pixy.blocks[f].signature==2){lmove();}
  }
  Time();
  delay(50);
}
void PID(){
 //пид регулятор центральный
  float err=0, p, kp=0.2, d, kd=0.05, errold=0, u, v;
  err=distl-distr;
  p=err*kp;
  d=kd*(err-errold);
  u=p+d;
  if(u>20){u=20;}
  if(u<-20){u=-20;}
  v=u+79;
  servo.write(v);
  errold=err;
}
void Time(){
  if (millis() - timer >= period){
    if (distr > 150 || distl > 150){
      i++;
      timer = millis();
      Serial.println(i);
      if (i == 17){
        delay(100);
        while (millis() - timer < 2500){
          distr = ultrasonicr.distanceRead();
          distl = ultrasonicl.distanceRead();
          PID();
        }
        power=0;
      }
    }
  }
}
void rmove(){
  if(pixy.blocks[f].height>50){
    servo.write(58);
  }
  else {return;}
}
void lmove(){
  if(pixy.blocks[f].height>50){
    servo.write(100);
  }
  else {return;}
}