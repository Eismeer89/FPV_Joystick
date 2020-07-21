//Analog in 5 - Main trigger gun (Second trigger)

byte frame1;
byte frame2;
byte frame1m=B11111111; // default state
byte frame2m=B10111111; // default state

// Anti contact bounce
uint8_t buttonPinEnterCount = 0;
uint8_t buttonPinEscCount = 0;
uint8_t buttonPinRightCount = 0;
uint8_t buttonPinDownCount = 0;
uint8_t buttonPinLeftCount = 0;
uint8_t buttonPinUpCount = 0;
uint8_t RatesPinHighCount = 0;
uint8_t RatesPinLowCount = 0;
uint8_t CameraPinUpCount = 0;
uint8_t CameraPinLeftCount = 0;
uint8_t CameraPinDownCount = 0;
uint8_t CameraPinRightCount = 0;
uint8_t triggerGunCount = 0;
uint8_t delayCount = 50;  // number of cycles to confirm the validity of a button click

void setup() {
Serial.begin(115200);
pinMode(2,  INPUT_PULLUP);  // enable PullUp resistor buttonPinEnter
pinMode(6,  INPUT_PULLUP);  // enable PullUp resistor buttonPinEsc

pinMode(3,  INPUT_PULLUP);  // enable PullUp resistor RatesPinHigh
pinMode(4,  INPUT_PULLUP);  // enable PullUp resistor RatesPinLow

pinMode(5,  INPUT_PULLUP);  // enable PullUp resistor buttonPinUp
pinMode(12, INPUT_PULLUP);  // enable PullUp resistor buttonPinDown
pinMode(13, INPUT_PULLUP);  // enable PullUp resistor buttonPinRight
pinMode(11, INPUT_PULLUP);  // enable PullUp resistor buttonPinLeft

pinMode(10, INPUT_PULLUP);  // enable PullUp resistor CameraPinUp
pinMode(8,  INPUT_PULLUP);  // enable PullUp resistor CameraPinDown
pinMode(7,  INPUT_PULLUP);  // enable PullUp resistor CameraPinRight
pinMode(9,  INPUT_PULLUP);  // enable PullUp resistor CameraPinLeft

// First frame creation
bitWrite(frame1, 7, 1); // no use bit
bitWrite(frame1, 6, 1); // sync bit

// Second frame creation
bitWrite(frame2, 6, 0); // sync bit
}

void loop() {
// buttonPinEnter
if(!digitalRead(2)) { buttonPinEnterCount++; }
  else {buttonPinEnterCount = 0; }
if(buttonPinEnterCount > delayCount) { bitWrite(frame1, 0, 0); buttonPinEnterCount = delayCount + 1; }  // buttonPinEnter Enable
  else { bitWrite(frame1, 0, 1); } // buttonPinEnter Disable
// buttonPinEsc
if(!digitalRead(6)) { buttonPinEscCount++; }
  else {buttonPinEscCount = 0; }
if(buttonPinEscCount > delayCount) { bitWrite(frame1, 1, 0); buttonPinEscCount = delayCount + 1; }  // buttonPinEsc Enable
  else { bitWrite(frame1, 1, 1); } // buttonPinEsc Disable
// RatesPinHigh
if(!digitalRead(3)) { RatesPinHighCount++; }
  else {RatesPinHighCount = 0; }
if(RatesPinHighCount > delayCount) { bitWrite(frame1, 3, 0); RatesPinHighCount = delayCount + 1; }  // RatesPinHigh Enable
  else { bitWrite(frame1, 3, 1); } // RatesPinHigh Disable
// RatesPinLow
if(!digitalRead(4)) { RatesPinLowCount++; }
  else {RatesPinLowCount = 0; }
if(RatesPinLowCount > delayCount) { bitWrite(frame1, 2, 0); RatesPinLowCount = delayCount + 1; }  // RatesPinLow Enable
  else { bitWrite(frame1, 2, 1); } // RatesPinLow Disable
// trigger gun
if(analogRead(5) < 512) { triggerGunCount++; }
  else {triggerGunCount = 0; }
if(triggerGunCount > delayCount) { bitWrite(frame2, 7, 0); triggerGunCount = delayCount + 1; }  // trigger gun Enable
  else { bitWrite(frame2, 7, 1); } // trigger gun Disable
// buttonPinUp
if(!digitalRead(5)) { buttonPinUpCount++; }
  else {buttonPinUpCount = 0; }
if(buttonPinUpCount > delayCount) { bitWrite(frame1, 5, 0); buttonPinUpCount = delayCount + 1; }  // buttonPinUp Enable
  else { bitWrite(frame1, 5, 1); } // buttonPinUp Disable
// buttonPinDown
if(!digitalRead(12)) { buttonPinDownCount++; }
  else {buttonPinDownCount = 0; }
if(buttonPinDownCount > delayCount) { bitWrite(frame2, 1, 0); buttonPinDownCount = delayCount + 1; }  // buttonPinDown Enable
  else { bitWrite(frame2, 1, 1); } // buttonPinDown Disable
// buttonPinRight
if(!digitalRead(13)) { buttonPinRightCount++; }
  else {buttonPinRightCount = 0; }
if(buttonPinRightCount > delayCount) { bitWrite(frame1, 4, 0); buttonPinRightCount = delayCount + 1; }  // buttonPinRight Enable
  else { bitWrite(frame1, 4, 1); } // buttonPinRight Disable
// buttonPinLeft
if(!digitalRead(11)) { buttonPinLeftCount++; }
  else {buttonPinLeftCount = 0; }
if(buttonPinLeftCount > delayCount) { bitWrite(frame2, 0, 0); buttonPinLeftCount = delayCount + 1; }  // buttonPinLeft Enable
  else { bitWrite(frame2, 0, 1); } // buttonPinLeft Disable
// CameraPinUp
if(!digitalRead(10)) { CameraPinUpCount++; }
  else {CameraPinUpCount = 0; }
if(CameraPinUpCount > delayCount) { bitWrite(frame2, 5, 0); CameraPinUpCount = delayCount + 1; }  // CameraPinUp Enable
  else { bitWrite(frame2, 5, 1); } // CameraPinUp Disable
// CameraPinDown
if(!digitalRead(8)) { CameraPinDownCount++; }
  else {CameraPinDownCount = 0; }
if(CameraPinDownCount > delayCount) { bitWrite(frame2, 3, 0); CameraPinDownCount = delayCount + 1; }  // CameraPinDown Enable
  else { bitWrite(frame2, 3, 1); } // CameraPinDown Disable
// CameraPinRight
if(!digitalRead(7)) { CameraPinRightCount++; }
  else {CameraPinRightCount = 0; }
if(CameraPinRightCount > delayCount) { bitWrite(frame2, 2, 0); CameraPinRightCount = delayCount + 1; }  // CameraPinRight Enable
  else { bitWrite(frame2, 2, 1); } // CameraPinRight Disable
// CameraPinLeft
if(!digitalRead(9)) { CameraPinLeftCount++; }
  else {CameraPinLeftCount = 0; }
if(CameraPinLeftCount > delayCount) { bitWrite(frame2, 4, 0); CameraPinLeftCount = delayCount + 1; }  // CameraPinLeft Enable
  else { bitWrite(frame2, 4, 1); } // CameraPinLeft Disable

// Send data to Arduino Due
if (Serial.available() > 0) { Serial.read(); Serial.write(frame1); Serial.write(frame2); } // send all frame on request from a Arduino Due
if (frame1 != frame1m) { Serial.write(frame1); frame1m = frame1; } // send modified first frame
if (frame2 != frame2m) { Serial.write(frame2); frame2m = frame2; } // send modified second frame

/*
// debugging
if (frame1 != frame1m) { 
  Serial.print("frame1: ");
  Serial.print(bitRead(frame1, 7)); 
  Serial.print(bitRead(frame1, 6));
  Serial.print(bitRead(frame1, 5)); 
  Serial.print(bitRead(frame1, 4));
  Serial.print(bitRead(frame1, 3)); 
  Serial.print(bitRead(frame1, 2)); 
  Serial.print(bitRead(frame1, 1));       
  Serial.print(bitRead(frame1, 0)); 
  Serial.print('\n');        
  frame1m = frame1; 
  } // send modified first frame

if (frame2 != frame2m) { 
  Serial.print("frame2: ");
  Serial.print(bitRead(frame2, 7)); 
  Serial.print(bitRead(frame2, 6));
  Serial.print(bitRead(frame2, 5)); 
  Serial.print(bitRead(frame2, 4));
  Serial.print(bitRead(frame2, 3)); 
  Serial.print(bitRead(frame2, 2)); 
  Serial.print(bitRead(frame2, 1));       
  Serial.print(bitRead(frame2, 0)); 
  Serial.print('\n');  
  frame2m = frame2; } // send modified second frame
*/
}
