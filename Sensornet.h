
#ifndef Sensornet_H
#define Sensornet_H


#include "Arduino.h"

#include <RFM69.h>
#include <SPI.h>

#define MAX_MESSAGE_LEN RF69_MAX_DATA_LEN


typedef enum nodeID
{
    SN_NODE_GATEWAY = 1,
    SN_NODE_PROTO1  = 20,
    SN_NODE_PROTO2  = 22,
    SN_NODE_PROTO3  = 23,
    SN_NODE_PROTO4  = 24,
    SN_NODE_OUTSIDE = 25,
    SN_NODE_ISAROOM = 56,
    SN_NODE_BREAD1  = 30,
    SN_NODE_BREAD2  = 31,
    API_TEST        = 101,
    DUST_SENSOR     = 102,
    XMAS_SENSOR     = 103,
    DISPLAY_TEST    = 104
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
    HTU21D_RH     = 1,
    HTU21D_C      = 2,
    SENSOR_TEST_A = 22,
    SENSOR_TEST_B = 23

} sensorType;


typedef struct nodeDescriptor
{
    const char *name;
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
    int getSensorIDforName( const char *n );
    int getSensorIDforName( String n );

    int queueReading( sensorType sensor, float value );
    void sendStructured( String sensor, float reading, String units, String memo );
    void sendReading( String sensor, float reading, String units );
    int writeCompressedPacketToSerial( nodeID origin, char *buffer, int len, int rssi );
    int writePacketToSerial( nodeID origin, char *buffer, int len, int rssi );

    boolean begin();
    uint8_t writeParam(uint8_t p, uint8_t v);
    void configureRadio( nodeID node, int network, int gateway, int frequency, char *key );
    void sleep(byte pinToWakeOn, byte direction, byte bPullup);
    byte sleepForaWhile (word msecs);
    char *borrowMessageBuffer();
    void startLoop();
    void endLoop();
    void resetCompression();
    void systemHibernate( word t );
    void radioSleep();
    void markRadioTXStart();
    void markRadioTXEnd();
    void markRadioPoweredUp();
    void markRadioPoweredDown();
    void printTimeStats();
    bool sendWithRetry(byte toAddress, const void *buffer, byte bufferSize, byte retries = 2, byte retryWaitTime = 40);
    boolean isGateway();

    // debugging helper functions from altaveesta

    RFM69 radio;
private:
    boolean compressionSterile;
    uint8_t _node;
    uint8_t _network ;
    uint8_t _gateway ;
    boolean _isGateway ;
    uint8_t  _frequency ;
};

void debug_cbuf(char cbuf[], int idx, bool clear);
void print_binary(int v, int num_places);
void print_hex(int v, int num_places);


#endif