#include <Servo.h>
#include<ProperLED.h>
#include<IntervalCheckTimer.h>
an_LED green, red;
IntervalCheckTimer timer;
Servo servo;
Servo servo2;
int angle=90;
int angle2=90;
bool found = true;
int instruction=false;
int lost=2;
void setup() {
  Serial.begin(9600);
  timer.setInterCheck(3000);
  servo.attach(5);
  servo2.attach(11);
  servo.write(angle);
  servo2.write(angle2);
}
void mover(int delta,int delta2, int x, int y){
     angle = angle + delta;
     angle2 = angle2 + delta2;
     int now = servo.read();
     int now2= servo2.read();
    // int x = 3;  //timing variable
    // int y = 1; // angle variable
     int pos, pos2;
         if (now>angle){
         pos=now-y;
         servo.write(pos);
         delay(x);
         }
         else if(now<angle) {
         pos=now+y;
         servo.write(pos);
         delay(x);
         }
   delay(10);
         if (now2>angle2){
         pos2=now2-y;
         servo2.write(pos2);
         delay(x);
         }
         else if(now2<angle2) {
         pos2=now2+y;
         servo2.write(pos2);
         delay(x);
         }
}
void loop() {

int delta=0;
int delta2=0;
if (Serial.available()>0){
    instruction = Serial.read();
    }
if (instruction == 120)
  {
   found = false;
 }

 else if(instruction > 96 && instruction < 107)
  {
   delta = map(instruction,97,106,-1,-15);
   found = true;
  }
 
 if (instruction > 106 && instruction < 117)
  {
    delta = map(instruction,107,116,1,15);
    found = true;
  }
 
 if (instruction > 64 && instruction < 75)
  {
    delta2 = map(instruction,65,74,-1,-8); // I limit on purpose angle at y position
    found = true;
  }

 if (instruction > 74 && instruction < 85)
  {
   delta2 = map(instruction,75,84,1,8);   // I limit on purpose angle at y position
   found = true;
  }

    
  mover(delta, delta2, 1, 1);

 /*
  if (timer.isMinChekTimeElapsedAndUpdate() && found == false)
  {
    
    servo.write(0);
    
    for(int x = 0 ; x<180 ; x++)
    {
        delay(40);
        servo.write(x);
       
          if (Serial.available()>0){
          instruction = Serial.read();
          if (instruction != 120 && instruction != 0)
            {
              found = true;
              break;
            }
       }
    
    }
    lost--;
    if(lost==0)
      servo2.write(120);
    else if(lost==-4){
      servo2.write(60);
      lost=2;
       }
  

  }*/
}
