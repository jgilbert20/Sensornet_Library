
#ifndef Sensornet_H
#define Sensornet_H


#include "Arduino.h"

#include <RFM69.h>
#include <SPI.h>  

#define MAX_MESSAGE_LEN RF69_MAX_DATA_LEN

class Sensornet  
{
 public:
	Sensornet(void);
  	boolean begin();
  	uint8_t writeParam(uint8_t p, uint8_t v);
  	void configureRadio( int node, int network, int gateway, int frequency, char *key );
  	void sleep(byte pinToWakeOn, byte direction, byte bPullup);
  	byte sleepForaWhile (word msecs);

	RFM69 radio;
 private:
   uint8_t _node;
 uint8_t _network ;
 uint8_t _gateway ;
uint8_t  _frequency ;
 };
 

 #endif