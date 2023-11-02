
long lastDebounceTime = 0; 
long debounceDelay = 700;    

void setup() {

  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);
  
}

void loop() {
    // read button press, 0 = press
    int buttonState = digitalRead(2);
    
    if (buttonState == LOW) {
      // debounce the button press to only look for a press every 700ms
      if ( (millis() - lastDebounceTime) > debounceDelay) {
        lastDebounceTime = millis(); //set the current time
        Serial.write(83);
        Serial.write(13);
        Serial.write(10);
    
    }
  }
}
