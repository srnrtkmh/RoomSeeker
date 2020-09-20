/*
   How to test encoder with Arduino
   url: http://osoyoo.com/?p=30267
*/
#define outputA 40
#define outputB 41
int counter = 0;
int aState, bState;
int aLastState, bLastState;
void setup() {
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);

  Serial.begin (115200);
  // Reads the initial state of the outputA
  aLastState = digitalRead(outputA);
}
void loop() {
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
  bState = digitalRead(outputB); // Reads the "current" state of the outputB

  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
  if (aState != aLastState) {
    if (aState == 0) {
      if (bState == 0) counter--;
      else             counter++;
    } else {
      if (bState == 0) counter++;
      else             counter--;
    }
    Serial.print("A Position: ");
    Serial.println(counter);
  } else if (bState != bLastState) {
    if (bState == 0) {
      if (aState == 0) counter++;
      else             counter--;
    } else {
      if (aState == 0) counter--;
      else             counter++;
    }
    Serial.print("B Position: ");
    Serial.println(counter);
  }

  aLastState = aState; // Updates the previous state of the outputA with the current state
  bLastState = bState; // Updates the previous state of the outputB with the current state
}
