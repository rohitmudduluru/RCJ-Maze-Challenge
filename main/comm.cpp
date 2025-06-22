#include "comm.h"


volatile bool interrupted = false;
int restartPi = -1;
bool doVictims = true;
bool goingForward = false;
void setFlag(){
  stop_motors();
  interrupted = true;
  Serial.println("INTERRUPTED");
}

void interruptFunc() {
  Serial.println("Interrupted!");
  if(!doVictims||restartPi!=-1){
    Serial.println("NO LIKEY INTERRUPT");
    Serial1.print("n");
    return;
  }
  stop_motors();
  while(Serial1.available()) Serial1.read();
  Serial1.print("a");
  // stop_motors(); delay(500);
 // forwardCm(30,8);
  // while (Serial1.available()) Serial1.read();  // clear buffer
  unsigned long time = millis(); 
  while (!Serial1.available()) {
    if(millis()-time>1000){
      Serial.println("Timed out");
      Serial1.print("n");
      interrupted = false;
      return;
    }
    Serial.println("waiting1");
  }
  char side = (char)Serial1.read();
  while (!Serial1.available()) Serial.println("waiting2");
  char vic = (char)Serial1.read();
  Serial.print("Side: "); Serial.print(side); Serial.print(" Vic: "); Serial.println(vic);

  if((side=='R'&&readTOF(RIGHT_TOF)<20)||(side=='L'&&readTOF(LEFT_TOF)<20)){
    blink();
    dropVictims(side, vic);
  }
 // backwardCm(30,8);
  stop_motors(); delay(250);
  interrupted=false;
  restartPi = cmToEnc(10);
}

void commBegin() {
  Serial1.setTX(TX_PIN);
  Serial1.setRX(RX_PIN);
  Serial1.begin(115200);
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), setFlag, RISING);
  Serial1.print("b");
}