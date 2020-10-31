uint8_t bat_pin = 0;

void setup() {
  Serial.begin(500000);
}

void loop() {
  long vol_raw;
  long vol_x100;

  vol_raw = analogRead(bat_pin);
  vol_x100 = (vol_raw * 100 * 5 * 2) / 1024 ;
  
  Serial.print("Battery voltage : ");
  Serial.print(vol_x100);
  Serial.print(" [0.01V]  ");
  Serial.print(vol_raw);
  Serial.print(" [-]\n");
  delay(100);
}
