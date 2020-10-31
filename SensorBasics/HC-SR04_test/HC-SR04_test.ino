int Trig[8] = {30, 31, 32, 33, 34, 35, 36, 37};
int Echo[8] = {10, 11, 12, 13,  0, 15, 14, 53};
int Duration;
float Distance;

void setup() {
  Serial.begin(500000);

  uint8_t i;
  for (i = 0; i < 8; i++) {
    pinMode(Trig[i], OUTPUT);
    pinMode(Echo[i], INPUT);
  }
}

void loop() {
  uint8_t i;
  
  for (i = 0; i < 8; i++) {
    digitalWrite(Trig[i], LOW);
    delayMicroseconds(1);
    digitalWrite(Trig[i], HIGH);
    delayMicroseconds(11);
    digitalWrite(Trig[i], LOW);
    Duration = pulseIn(Echo[i], HIGH, 100000);
    if (Duration > 0) {
      Distance = Duration / 2;
      Distance = Distance * 340 * 100 / 1000000; // ultrasonic speed is 340m/s = 34000cm/s = 0.034cm/us
      Serial.print(i);
      Serial.print(" : ");
      Serial.print(Duration);
      Serial.print(" : us ");
      Serial.print(Distance);
      Serial.println(" cm");
    }
    Serial.print("\n");
  }
  delay(500);
}
