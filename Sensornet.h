
#ifndef Sensornet_H
#define Sensornet_H


#include "Arduino.h"

#include <RFM69.h>
#include <SPI.h>  

#define MAX_MESSAGE_LEN RF69_MAX_DATA_LEN


typedef enum nodeID
{ 
	API_TEST    = 101,
	DUST_SENSOR = 102,
	XMAS_SENSOR = 103,
	DISPLAY_TEST = 104
} nodeID;

#define SN_CODEBOOK_MAX_SIZE 13

typedef struct compactedMessage
{
  char type;
  unsigned short sequence;
  unsigned long timestamp;
  byte codebookID;
  float reading[SN_CODEBOOK_MAX_SIZE];

} compactedMessage;

typedef enum sensorType
{ 
	SENSOR_TEST_A    = 0,
	SENSOR_TEST_B 	= 1,
	HTU21D_RH = 2,
	HTU21D_C = 3
} sensorType;


typedef struct nodeDescriptor
{
	const char* name;
	int address;
};


typedef struct
{
  uint8_t sensor;
  float reading;
  int millisdiff;
}
sensorReading;


class Sensornet  
{
 public:
	unsigned long messageSequence;
	nodeDescriptor thisNodeDesc;


	Sensornet(void);

	void newQuanta();

	void flushQueue(); 
	void setCodebook( int codebookID );
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