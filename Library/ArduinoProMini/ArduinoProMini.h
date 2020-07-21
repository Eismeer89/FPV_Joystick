#ifndef ARDUINOPROMINI_H_
#define ARDUINOPROMINI_H_

#include <Arduino.h>

class ArduinoProMini
{
public:
	ArduinoProMini();
	~ArduinoProMini();

	void  setFrame1(byte frame1);
	void  setFrame2(byte frame2);
	byte getFrame1();
	byte getFrame2();

	void  ReadFrame();
	void  setControls(		int& Enter, 
					int& Cancel, 
					int& RatesLow, 
					int& RatesHigh, 
					int& CameraRight, 
					int& CameraUp, 
					int& CameraLeft, 
					int& CameraDown, 
					int& Right, 
					int& Down, 
					int& Left,
					int& Up,
					int& Trigger		);

private:
	byte	frame1 = B11111111, frame2 = B10111111;
};

#endif // ARDUINOPROMINI_H_
