uint8_t ir_pin[12] = {38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49};

void setup() {
  Serial.begin(500000);

  uint8_t i;
  for (i = 0; i < 12; i++) {
    pinMode(ir_pin[i], INPUT_PULLUP);
  }
}

void loop() {
  uint8_t i;

  Serial.print("IR dection : ");
  Serial.print(digitalRead(ir_pin[0]));
  for (i = 1; i < 12; i++) {
    Serial.print(", ");
    Serial.print(digitalRead(ir_pin[i]));
  }
  Serial.print("\n");
  delay(200);
}
