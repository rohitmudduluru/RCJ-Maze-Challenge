int num = 0;
void interFunc(){
 num++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(7,INPUT);
  attachInterrupt(digitalPinToInterrupt(7), interFunc, RISING);
}

void loop() {
  Serial.println(num);
}
