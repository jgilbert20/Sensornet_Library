
#ifndef Sensornet_H
#define Sensornet_H


#include "Arduino.h"

#include <RFM69.h>
#include <SPI.h>  

#define MAX_MESSAGE_LEN RF69_MAX_DATA_LEN


typedef enum nodeID
{ 
	API_TEST = 101,
	DUST_SENSOR = 102
} nodeID;


typedef enum 
{ 
	HTU21D_RH,
	HTU21D_C
} sensorType;


typedef struct nodeDescriptor
{
	const char* name;
	int address;
};


class Sensornet  
{
 public:
	Sensornet(void);

	void queueReading( sensorType sensor, float value );
    void sendStructured( String sensor, float reading, String units, String memo );
 	void sendReading( String sensor, float reading, String units );

  	boolean begin();
  	uint8_t writeParam(uint8_t p, uint8_t v);
  	void configureRadio( nodeID node, int network, int gateway, int frequency, char *key );
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