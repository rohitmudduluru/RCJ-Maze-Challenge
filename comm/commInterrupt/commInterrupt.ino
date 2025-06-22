void interFunc(){
  Serial.println("Interrupted!");
  while(Serial1.available()) Serial1.read();
  while(!Serial1.available());
  Serial.print((char)Serial1.read());
  while(!Serial1.available());
  Serial.println((char)Serial1.read());
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(115200);
  pinMode(7,INPUT);
  attachInterrupt(digitalPinToInterrupt(7), interFunc, RISING);
  while(!Serial);
  Serial.println("Interrupt set up");
  while(!Serial1);
  while(Serial1.available())Serial1.read();
}
void loop() {

}
