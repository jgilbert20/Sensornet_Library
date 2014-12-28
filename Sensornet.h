
#ifndef Sensornet_H
#define Sensornet_H


#include "Arduino.h"

#include <RFM69.h>
#include <SPI.h>

#define MAX_MESSAGE_LEN RF69_MAX_DATA_LEN


typedef enum nodeID
{
    SN_NODE_GATEWAY  = 1,
    SN_NODE_POWERMON = 10,
    SN_NODE_OUTSIDE  = 11,

    SN_NODE_PROTO1   = 20,
    SN_NODE_PROTO2   = 22,
    SN_NODE_PROTO3   = 23,
    SN_NODE_PROTO4   = 24,
    SN_NODE_BREAD1   = 30,
    SN_NODE_BREAD2   = 31,
    SN_NODE_ISAROOM  = 56,

    API_TEST         = 101,
    DUST_SENSOR      = 102,
    XMAS_SENSOR      = 103,
    DISPLAY_TEST     = 104
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
    BATT_V              = 0,
    HTU21D_RH           = 1,
    HTU21D_C            = 2,
    TSL2591_LUX   = 3,
    TSL2591_FULL  = 4,
    TSL2591_IR    = 5,
    LUX_FLT       = 6,
    LUX_FLT_BROAD  = 7,
    LUX_FLT_INFRA =  8,
    DALLAS        =  9,
    RADIO_BG_RSSI =  10,
    MCP9808       =  11,
    BMP_TEMP      = 12,
    BMP_PRESSURE  = 13,
    CURRENT_A     = 14,
    CURRENT_B     = 15,
    AM2315_TEMP   = 16,
    AM2315_RH     = 17,
    SHT15_TEMP    = 18,
    SHT15_RH      = 19,
    THERM0_TEMP   = 20,
    THERM1_TEMP   = 21,
    SENSOR_TEST_A       = 22,
    SENSOR_TEST_B       = 23,
    RADIO_ACK_SIBOOT    = 24,
    RADIO_ACK_SILAST    = 25,
    RADIO_ACK_EWMA      = 26,
    T_AVTX_SIBOOT       = 27,
    T_AVTX_SILAST       = 28,
    T_AVTX_EWMA         = 29,
    T_LOOP_SIBOOT       = 30,
    T_LOOP_SILAST       = 31,
    T_DUTYCYCLE_SIBOOT  = 32,
    T_DUTYCYCLE_SILAST  = 33,
    TOTAL_LOOPS         = 34

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
   // nodeDescriptor thisNodeDesc;

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
    void transmitStatistics();
    void resetStatisticsCycle();
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