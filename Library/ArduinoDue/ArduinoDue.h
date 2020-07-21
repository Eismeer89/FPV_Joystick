#ifndef _ARDUINODUE_H_
#define _ARDUINODUE_H_

// for my wife
// with love from her husband
// V&V

#if ARDUINO >= 100
#include "Arduino.h"
#endif

class ArduinoDue
{
public:
	ArduinoDue();
	~ArduinoDue();

	void  Init();
	void  Init(int Serial_speed, int Serial1_speed);

	void  PinsInit();

	void  AnalogRead	(int* pins);
	void  DigitalRead	(int&, int&, int&, int&);

	void  PPMInit(int PPM_Out, float Frame_lenght, float Pulse_lenght, bool InvertPPM);
	void  PPMCalculate(int* ppm_channels, uint32_t* periods);
	
	void  setTimer(int ms);
	void  startTimer();
	bool  isTimerEnd();
	float   Haw_much_time_elapsed();

	void  setTimer1(int ms);
	void  startTimer1();
	bool  isTimerEnd1();
	float   Haw_much_time_elapsed1();

	const unsigned char* MyLogo(int i);
	const unsigned char* CheckItem();

	int PPMFrame;

	int		_ms, _tBegin, _tEnd;
	bool    _TimerSetup = false;
	bool	_TimeBegin = false;

	int		_ms1, _tBegin1, _tEnd1;
	bool    _TimerSetup1 = false;
	bool	_TimeBegin1 = false;
};

#endif // ARDUINODUE_H_
