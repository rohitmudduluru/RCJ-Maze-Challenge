#include "dropper.h"

Servo servo;


void moveStepper(int steps){;
  for(int x = 0; x < steps; x++)
  {
    digitalWrite(STEP, HIGH);
    delay(5);
    digitalWrite(STEP, LOW);
    delay(5);
  }
}

void dropVictims(char side, char vic) {
  bool onLeft;
  if(side=='L') onLeft = true;
  else onLeft = false;
  switch(vic){
    case 'h':
      drop(onLeft);
    case 's':
      drop(onLeft);
    case 'u': 
      break;
  }
  return;
}

void drop(bool onLeft){
  servo.write(onLeft?30:150);
  digitalWrite(DIR, LOW);

  moveStepper(145);
  for(int i= 0; i < 8; i++){
    digitalWrite(DIR, HIGH);
    moveStepper(20);
    digitalWrite(DIR, LOW);
    moveStepper(20);
  }
  digitalWrite(DIR, HIGH);
  moveStepper(10);
  servo.write(onLeft?0:180);
  delay(500);
  servo.write(90);

}