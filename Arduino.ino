void setup() {
  pinMode(13, OUTPUT); //Drone power switch
  digitalWrite(13, HIGH);
  pinMode(6, OUTPUT); //Electromagnet relay
  pinMode(7, OUTPUT); //Charge relay
  pinMode(8, INPUT); //IR sensor

}

void loop() {
 if(digitalRead(8) == LOW)
 {
  digitalWrite(6, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(7, HIGH);
  delay(2700000);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  delay(2000);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);
 }
}
