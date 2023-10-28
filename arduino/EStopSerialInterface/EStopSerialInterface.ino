
long lastDebounceTime = 0; 
long debounceDelay = 700;    

void setup() {

  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);
  
}

void loop() {
  
    int buttonState = digitalRead(2);

    if (buttonState == LOW) {
      if ( (millis() - lastDebounceTime) > debounceDelay) {
        lastDebounceTime = millis(); //set the current time
        Serial.write(83);
        Serial.write(13);
        Serial.write(10);
    
    }
  }
}
