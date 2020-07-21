#include "ArduinoProMini.h"

ArduinoProMini::ArduinoProMini()
{

}
ArduinoProMini::~ArduinoProMini()
{

}


void		ArduinoProMini::setFrame1(byte _frame1)
{
	frame1 = _frame1;
}
void		ArduinoProMini::setFrame2(byte _frame2)
{
	frame2 = _frame2;
}

byte		ArduinoProMini::getFrame1()
{
	return frame1;
}
byte		ArduinoProMini::getFrame2()
{
	return frame2;
}

void		ArduinoProMini::ReadFrame()
{
	while	(Serial1.available() > 0)
	{
		byte Frame = Serial1.read();
		if (bitRead(Frame, 6) == 1) { setFrame1(Frame); }
		else			    { setFrame2(Frame); }
	}
}

void		ArduinoProMini::setControls(		int& Enter,
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
							int& Trigger		)
{
	ReadFrame();
	Enter			= bitRead(getFrame1(), 0);
	Cancel			= bitRead(getFrame1(), 1);
	RatesLow		= bitRead(getFrame1(), 2);
	RatesHigh		= bitRead(getFrame1(), 3);
	CameraRight		= bitRead(getFrame1(), 4);
	CameraUp		= bitRead(getFrame1(), 5);
	CameraLeft		= bitRead(getFrame2(), 0);
	CameraDown		= bitRead(getFrame2(), 1);
	Right			= bitRead(getFrame2(), 2);
	Down			= bitRead(getFrame2(), 3);
	Left			= bitRead(getFrame2(), 4);
	Up			= bitRead(getFrame2(), 5);
	Trigger                 = bitRead(getFrame2(), 7);
}
