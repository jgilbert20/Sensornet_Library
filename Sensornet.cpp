#include "Sensornet.h"
#include "Arduino.h"

#include <RFM69.h>

#define SENSORNET_NOT_POPULATED -999999.99

unsigned long quantaStartTime;
int currentCodebookIndex;
sensorType *currentCodebook;
compactedMessage compactedMessageBuffer;

const char *COMMA = ",";

// A large common buffer for packing and unpacking data
char messageChar[MAX_MESSAGE_LEN + 1];

// Adds about 40 bytes (13*3) plus tiny overhead?
int codebookRegistry[][SN_CODEBOOK_MAX_SIZE] =
{
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 22, 23 },
    { SENSOR_TEST_A, SENSOR_TEST_B, HTU21D_RH, HTU21D_C },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }
};

// Each entry is 4 bytes (ptrs are apparently 16bit)
typedef struct sensorDescriptor
{
    const __FlashStringHelper *name;
    const __FlashStringHelper *unit;
} sensorDescriptor;

unsigned long timeSpentTX                  = 0;
unsigned long timeSpentRadioOn             = 0;
unsigned long timeSpentLoop                = 0;
unsigned long timeSpentSleeping            = 0;
unsigned long totalMessagesSent            = 0;
unsigned long totalCompactedMessagesSent   = 0;
unsigned long totalLongFormMessagesSent    = 0;
unsigned long totalMessagesAcknowledged    = 0;
unsigned long totalLoops                   = 0;

unsigned long statcycle_timeSpentTX                  = 0;
unsigned long statcycle_timeSpentRadioOn             = 0;
unsigned long statcycle_timeSpentLoop                = 0;
unsigned long statcycle_timeSpentSleeping            = 0;
unsigned long statcycle_totalMessagesSent            = 0;
unsigned long statcycle_totalCompactedMessagesSent   = 0;
unsigned long statcycle_totalLongFormMessagesSent    = 0;
unsigned long statcycle_totalMessagesAcknowledged    = 0;
unsigned long statcycle_totalLoops                   = 0;


unsigned long currentLoopStarttime = 0;
unsigned long currentTimeLastAwoken = 0;
unsigned long lastTransmitStarted = 0;
unsigned long lastTimeRadioOn = 0;

boolean radioPoweredUp = false;


char *Sensornet::borrowMessageBuffer()
{
    return messageChar;
}

void Sensornet::printTimeStats()
{
    Serial.print( F( "D: [SN] Total Loops: ")); Serial.println( totalLoops );
    Serial.print( F( "D: [SN] TX Time: ")); Serial.println( timeSpentTX );
    if( totalLoops != 0 )
        Serial.print( F( "D: [SN] TX Time/L: ")); Serial.println( timeSpentTX / totalLoops  );
    Serial.print( F( "D: [SN] Loop Time: ")); Serial.println( timeSpentLoop );
    if( totalLoops != 0 )
        Serial.print( F( "D: [SN] Loop Time/L: ")); Serial.println( timeSpentLoop / totalLoops );
    Serial.print( F( "D: [SN] Sleep Time: ")); Serial.println( timeSpentSleeping );
    Serial.print( F( "D: [SN] Millis(): ")); Serial.println( millis() );
    Serial.print( F( "D: [SN] Total Msg Sent (all types): ")); Serial.println( totalMessagesSent );
    Serial.print( F( "D: [SN] Total Msg Acked: ")); Serial.println( totalMessagesAcknowledged );
    Serial.print( F( "D: [SN] Total Msg Compacted: ")); Serial.println( totalCompactedMessagesSent );
    Serial.print( F( "D: [SN] Total Msg Longform: ")); Serial.println( totalLongFormMessagesSent );
}

void Sensornet::radioSleep()
{
    markRadioPoweredDown();
    radio.sleep();
}

void Sensornet::systemHibernate( word t )
{
    radioSleep();
    Serial.flush();
    sleepForaWhile( t );
}

void Sensornet::startLoop()
{
    Serial.println( F("D: --- MARK LOOP START"));
    if ( currentLoopStarttime != 0 )
        return;
    currentLoopStarttime = millis();
}

void Sensornet::endLoop()
{
    Serial.println( F("D: --- MARK LOOP END"));

    timeSpentLoop += millis() - currentLoopStarttime;
    totalLoops++;
    statcycle_timeSpentLoop += millis() - currentLoopStarttime;
    statcycle_totalLoops++;
    currentLoopStarttime = 0;
}


void Sensornet::markRadioTXStart()
{
    Serial.println( F("D: --- MARK RADIO TX START"));
    if ( lastTransmitStarted != 0 )
        return;
    markRadioPoweredUp();
    lastTransmitStarted = millis();
}

void Sensornet::markRadioTXEnd()
{
    Serial.println( F("D: --- MARK RADIO TX END"));
    if ( lastTransmitStarted == 0 )
        return;

    timeSpentTX += millis() - lastTransmitStarted;
    statcycle_timeSpentTX += millis() - lastTransmitStarted;
    lastTransmitStarted = 0;
}

void Sensornet::markRadioPoweredUp()
{
    if ( !radioPoweredUp)
    {
        lastTimeRadioOn = millis();
    }
    radioPoweredUp = true;
}

void Sensornet::markRadioPoweredDown()
{
    if ( radioPoweredUp)
    {
        timeSpentRadioOn = millis() - lastTimeRadioOn;
        statcycle_timeSpentRadioOn = millis() - lastTimeRadioOn;
        lastTimeRadioOn = 0;
    }

    radioPoweredUp = false;
}

// Sensornet name and unit lookup table. Array size of 50 seems to use about
// 200 bytes, sugginesting each entry is 4 bytes. This is actually a dominate
// memory use of the library. Fortunatly, this table can be ommited if none of
// the longform sensor reading syntaxes are used. If your code never calls the
// function that takes a sensor name as a string, you can avoid this entire
// table (and it could be potentially optimized out -- to check)

#define SENSORNET_SENSOR_LUT_SIZE  45   // Be careful to make this exact

sensorDescriptor sensorLookup[SENSORNET_SENSOR_LUT_SIZE];

void setSensorMap( int index,  const __FlashStringHelper *name,  const __FlashStringHelper *unit )
{
    sensorLookup[index].name = name;
    sensorLookup[index].unit = unit;
}

void populateSL()
{
    setSensorMap( 0,  F("BATT-V"),                        F("volts")     );
    setSensorMap( 1,  F("HTU21D-RH"),                     F("%RH")       );
    setSensorMap( 2,  F("HTU21D-Temp"),                   F("C")         );
    setSensorMap( 3,  F("TSL2591-Lux"),                   F("lux")       );
    setSensorMap( 4,  F("TSL2591-Full"),                  F("raw")       );
    setSensorMap( 5,  F("TSL2591-IR"),                    F("raw")       );
    setSensorMap( 6,  F("LUX-FLT"),                       F("lux")       );
    setSensorMap( 7,  F("LUX-FLT-Broad"),                 F("raw")       );
    setSensorMap( 8,  F("LUX-FLT-Infra"),                 F("raw")       );
    setSensorMap( 9,  F("Dallas"),                        F("C")         );
    setSensorMap( 10, F("Radio-BG-RSSI"),                 F("DB")        );
    setSensorMap( 11, F("MCP9808"),                       F("C")         );
    setSensorMap( 12, F("BMP-Temp"),                      F("C")         );
    setSensorMap( 13, F("BMP-Pressure"),                  F("Pa")        );
    setSensorMap( 14, F("Current-A"),                     F("Arms")      );
    setSensorMap( 15, F("Current-B"),                     F("Arms")      );
    setSensorMap( 16, F("AM2315-Temp"),                   F("C")         );
    setSensorMap( 17, F("AM2315-RH"),                     F("%RH")       );
    setSensorMap( 18, F("SHT15-Temp"),                    F("C")         );
    setSensorMap( 19, F("SHT15-RH"),                      F("%RH")       );
    setSensorMap( 20, F("Therm0-Temp"),                   F("C")         );
    setSensorMap( 21, F("Therm1-Temp"),                   F("C")         );
    setSensorMap( 22, F("Sensor-Test-A"),                 F("unita")     );
    setSensorMap( 23, F("Sensor-Test-B"),                 F("unitb")     );
    setSensorMap( 24, F("Radio-Ack-SiBoot"),              F("%")         );
    setSensorMap( 25, F("Radio-Ack-SiLast"),              F("%")         );
    setSensorMap( 26, F("Radio-Ack-EWMA"),                F("%")         );
    setSensorMap( 27, F("T-AvTx-SiBoot"),                 F("ms")        );
    setSensorMap( 28, F("T-AvTx-SiLast"),                 F("ms")        );
    setSensorMap( 29, F("T-AvTx-EWMA"),                   F("ms")        );
    setSensorMap( 30, F("T-Loop-SiBoot"),                 F("ms")        );
    setSensorMap( 31, F("T-Loop-SiLast"),                 F("ms")        );
    setSensorMap( 32, F("T-DutyCycle-SiBoot"),            F("%")         );
    setSensorMap( 33, F("T-DutyCycle-SiLast"),            F("%")         );
    setSensorMap( 34, F("Total-Loops"),                   F("loops")     );
    setSensorMap( 35, F("LUX-LOW"),                       F("lux")       );
    setSensorMap( 36, F("LUX-Ratio"),                     F("%")         );
    setSensorMap( 37, F("LUX-LOW-Broad"),                 F("raw")       );
    setSensorMap( 38, F("LUX-LOW-Infra"),                 F("raw")       );
    setSensorMap( 39, F("DUM"),                           F("na")        );
    setSensorMap( 40, F("MDWIND"),                        F("raw")       );
    setSensorMap( 41, F("Radio-Recent-BG-RSSI"),          F("DB")        );
    setSensorMap( 42, F("SI1145-Vis"),                    F("raw")       );
    setSensorMap( 43, F("SI1145-IR"),                     F("raw")       );
    setSensorMap( 44, F("SI1145-UvIndex"),                F("index")     );

};

Sensornet::Sensornet()
{
    ;
    currentCodebook  = null;
    populateSL();
}

nodeDescriptor getNodeDescriptor(nodeID id)
{

    nodeDescriptor n;

    switch ( id )
    {
    case API_TEST:
        n.name = "API-Test";
        break;

    case DUST_SENSOR:
        n.name = "Dust-Sensor";
        break;

    case XMAS_SENSOR:
        n.name = "Xmas-Sensor";
        break;

    case DISPLAY_TEST:
        n.name = "Display-Test";
        break;

    case SN_NODE_PROTO1:
        n.name = "Proto1";
        break;
    case SN_NODE_PROTO2:
        n.name = "Proto2";
        break;
    case SN_NODE_PROTO3:
        n.name = "Proto3";
        break;
    case SN_NODE_PROTO4:
        n.name = "Proto4";
        break;

    case SN_NODE_BREAD1:
        n.name = "Bread1";
        break;
    case SN_NODE_BREAD2:
        n.name = "Bread2";
        break;
    case SN_NODE_GATEWAY:
        n.name = "Gateway-AVR";
        break;

    default:
        n.name = "UNKNOWN";
        break;
        ;

    }
    return n;
}




void Sensornet::configureRadio( nodeID node, int network, int gateway, int frequency, char *key )
{
    thisNodeDesc = getNodeDescriptor( node );

    _node = (int)node;
    _network = network;
    _gateway = gateway;
    _frequency = frequency;

    radio.initialize(_frequency, _node, _network);
    radio.encrypt( key );
    markRadioPoweredUp();

    if ( _gateway == _node )
        _isGateway =  true;
    else
        _isGateway = false;

    messageSequence = 0;
}

boolean Sensornet::isGateway()
{

    return _isGateway;
}

void Sensornet::setCodebook( int codebook )
{
    currentCodebookIndex = codebook;
    currentCodebook = (sensorType *)codebookRegistry[codebook];
    compactedMessageBuffer.codebookID = codebook;
}


// Clears out the current in progress measurements (must always be reset) -
// does not specifically clear out the codebook itself

void Sensornet::newQuanta()
{
    quantaStartTime = millis();
    for ( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
        compactedMessageBuffer.reading[i] = SENSORNET_NOT_POPULATED;

    compactedMessageBuffer.type = 'C';
    compactedMessageBuffer.sequence = messageSequence;

    compactedMessageBuffer.timestamp = quantaStartTime;

    compressionSterile = true;
}

// Resets the engine (used after a flush and after a packet decode)

void Sensornet::resetCompression()
{
    // currentCodebook = null;
    for ( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
        compactedMessageBuffer.reading[i] = SENSORNET_NOT_POPULATED;
    compressionSterile = true;
}

int findIndexForSensor( sensorType *codebook, sensorType query )
{
    if ( codebook == null)
        return -2;

    for ( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
        if ( query == codebook[i] )
            return i;

    return -1;
}


int Sensornet::queueReading( sensorType sensor, float value )
{
    // Look up the index in the current codebook
    int index = findIndexForSensor( currentCodebook, sensor );

    if ( index < 0 )
        return -1;

    compactedMessageBuffer.reading[index] = value;

    compressionSterile = false;
    return 0;
}


inline const __FlashStringHelper *getSensorName( sensorType t )
{
    if ( t >= SENSORNET_SENSOR_LUT_SIZE )
        return null;

    return sensorLookup[t].name;
}

inline const __FlashStringHelper *getSensorUnits( sensorType t )
{
    if ( t >= SENSORNET_SENSOR_LUT_SIZE )
        return null;

    return sensorLookup[t].unit;
}

int Sensornet::getSensorIDforName( const char *n )
{
    for ( int i = 0 ; i < SENSORNET_SENSOR_LUT_SIZE ; i++ )
    {
        if ( strcmp_P( n, (const char *)sensorLookup[i].name ) == 0 )
            return i;
    }

    return -1;
}

int Sensornet::getSensorIDforName( String n )
{
    for ( int i = 0 ; i < SENSORNET_SENSOR_LUT_SIZE ; i++ )
    {
        if ( strcmp_P(  n.c_str(), (const char *)sensorLookup[i].name ) == 0 )
            return i;
    }

    return -1;
}

#define SENSORNET_COMPACTED_MAGIC 'C'
#define SENSORNET_LONGFORM_MAGIC 'R'

// Translates a packet into output to the serial line in a common format The
// sensornet output logging format is a CSV delimited sequence of fields C or
// R messages have this structure: Logline ID, Node, Millis of collection,
// Sensor, Reading, Units, Memo, RSSI during receipt, Node ID

int Sensornet::writePacketToSerial( nodeID origin, char *buffer, int len, int rssi )
{
    if ( len < 1 )
        return -1;

    char msgType = buffer[0];

    // Check if this is a compacted message? If so, pass to the handler

    if ( msgType == SENSORNET_COMPACTED_MAGIC )
    {
        debug_cbuf( buffer, len, false );
        writeCompressedPacketToSerial( origin, buffer, len, rssi );
        return 0;
    }

    // If its a longform packet, its already in the right format, just echo it

    if ( msgType == SENSORNET_LONGFORM_MAGIC )
    {
        Serial.print( buffer );
        Serial.print( COMMA );
        Serial.print( rssi );
        Serial.print( COMMA );
        Serial.print( origin, DEC);
        Serial.println();

        return 0;
    }

    // If neither, we have a snowflake.. Dump what we got for later analysis

    debug_cbuf( buffer, len, false );

    Serial.print( buffer );
    Serial.print( COMMA );
    Serial.print( rssi );
    Serial.print( COMMA );
    Serial.print( origin, DEC);
    Serial.println();

    return 0;
}

int Sensornet::writeCompressedPacketToSerial( nodeID origin, char *buffer, int len, int rssi )
{
    compactedMessage *msg = (compactedMessage *)buffer;
    nodeDescriptor sender = getNodeDescriptor( origin );

    // if( buffer == null )
    //     return -1;

    if ( msg->type != SENSORNET_COMPACTED_MAGIC )
        return -1;

    setCodebook( msg->codebookID );
    Serial.print( F("D: Compressed message on codebook: "));
    Serial.println( msg->codebookID, DEC );

    for ( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
    {
        Serial.print( F("D:  Checking codebook position="));
        Serial.print( i );
        Serial.print( F(" reading=")) ;
        Serial.print( msg->reading[i] );
        Serial.print( F(" codebook-idx=")) ;
        Serial.print( currentCodebook[i] );
        Serial.println();

        if ( msg->reading[i] == SENSORNET_NOT_POPULATED )
            continue;

        Serial.print( SENSORNET_COMPACTED_MAGIC );
        Serial.print( msg->sequence );
        Serial.print( F("-cbp") );
        Serial.print( i );
        Serial.print( F("-cb") );
        Serial.print( msg->codebookID );
        Serial.print( F("-cbi") );
        Serial.print( currentCodebook[i] );
        Serial.print( COMMA );
        Serial.print( sender.name );
        Serial.print( COMMA );
        Serial.print( msg->timestamp );
        Serial.print( COMMA );
        Serial.print( (const __FlashStringHelper *) getSensorName( currentCodebook[i] ) );
        // Serial.print( (const __FlashStringHelper *) F("yes") );
        //Serial.print( (const __FlashStringHelper *) sensorLookup[22].name );
        Serial.print( COMMA );
        Serial.print( msg->reading[i] );
        Serial.print( COMMA );
        Serial.print( getSensorUnits( currentCodebook[i] ) );
        Serial.print( COMMA );
        // memo goes here
        Serial.print( COMMA );
        Serial.print( rssi );
        Serial.print( COMMA );
        Serial.print( (int)origin );

        Serial.println();
    }

    resetCompression();

    return 0;
}

// A common entry point for all radio transmission

bool Sensornet::sendWithRetry(byte toAddress, const void *buffer, byte bufferSize, byte retries, byte retryWaitTime) //40ms roundtrip req for  61byte packets
{
    markRadioTXStart();
    boolean r = radio.sendWithRetry( _gateway, buffer, bufferSize, retries, retryWaitTime  );
    markRadioTXEnd();
    if ( r )
    {
        totalMessagesAcknowledged++;
        statcycle_totalMessagesAcknowledged++;
    }
    messageSequence++;
    totalMessagesSent++;
    statcycle_totalMessagesSent++;
}

// Flushes out the current packet, and resets the compression engine.

void Sensornet::flushQueue()
{

    if ( !compressionSterile )
    {
        debug_cbuf( (char *)&compactedMessageBuffer,  sizeof(compactedMessageBuffer), false );
        sendWithRetry( _gateway, (char *)&compactedMessageBuffer,  sizeof(compactedMessageBuffer), 3, 40  );
        totalCompactedMessagesSent++;
        statcycle_totalCompactedMessagesSent++;
    }
    resetCompression();

}


void Sensornet::sendReading( String sensor, float reading, String units )
{
    sendStructured( sensor, reading, units, F("") );
}

// Normal strncpy seems to try to pad the buffer with nulls, i don't want that
// since this seems like a waste of CPU cycles

int nopad_strncpy( char *dest, const char *src, int bufsize )
{
    int i;
    for ( i = 0 ; i < bufsize ; i++ )
    {
        dest[i] = src[i];
        if ( dest[i] == 0 )
            return i;
    }

    return i;

}

// Send structured transmits a sensor reading "upstream" based on a fixed set
// of rules designed to bring the reading in a consistent way to the gateway.
// It tries to be as effiecent as possible. If the calling node is remote, the
// library sends a packet to the gateway. The format of the packet depends on
// the context of the caller. If called with a set codebook and time quanta,
// the reading is placed into a compressed buffer that will be pooled with
// other readings and transmitted during flushQueue(); Otherwise, a long form
// CSV-like message is created and transmitted to the gateway. As a special
// case, if the caller itself is configured as a gateway, no data transmission
// is performed, and instead the reading is relayed to the serial port in
// SNCLF syntax.

void Sensornet::sendStructured( String sensor, float reading, String units, String memo )
{
    nodeDescriptor node = getNodeDescriptor( (nodeID) _node );

    Serial.print( F("D: send structured: Sending reading for "));
    Serial.print( sensor );
    Serial.print( F(" value ") );
    Serial.println( reading );

    if ( currentCodebook != null && !isGateway() )
    {
        // Check to see if this sensor has a known sensor ID
        int sensorID = getSensorIDforName( sensor );

        Serial.print( F("D: Looking up:") );
        Serial.print( sensor );
        Serial.print( F("Got:") );
        Serial.print( sensorID );
        Serial.println();

        if ( sensorID >= 0 )
        {
            // Looks like we could queue this since we were able to convert to
            // Sensor ID queueReading() will return negative number if it
            // couldn't compress it, and we'll fall back to the regular
            // sending process.

            Serial.println( F("D: Adding to compressed queue") );
            int result = queueReading( (sensorType) sensorID, reading );

            if ( result >= 0 ) // success
                return;

            // If queue reading failed, we'll fall back to the old way
        }
    }

    long unsigned int now = millis();

    // To be maximally efficient with RAM, this code avoids dynamically memory
    // and uses a fixed, library-wide buffer. Use of the String object
    // requires a lot of wierd mallocing

    {
        byte pos = 0;
        messageChar[pos++] = 'R';
        const char comma = ',';
        pos += snprintf( messageChar + pos, MAX_MESSAGE_LEN - pos, "%d", messageSequence );
        messageChar[pos++] = comma;
        pos += nopad_strncpy( messageChar + pos, node.name, MAX_MESSAGE_LEN - pos  );
        messageChar[pos++] = comma;
        pos += snprintf( messageChar + pos, MAX_MESSAGE_LEN - pos, "%ld", now );
        messageChar[pos++] = comma;
        pos += nopad_strncpy( messageChar + pos, sensor.c_str(), MAX_MESSAGE_LEN - pos );
        messageChar[pos++] = comma;
        dtostrf( reading, 0, 3, messageChar + pos );
        pos += strlen(messageChar + pos );
        messageChar[pos++] = comma;
        pos += nopad_strncpy( messageChar + pos, units.c_str(), MAX_MESSAGE_LEN - pos );
        messageChar[pos++] = comma;
        pos += nopad_strncpy( messageChar + pos, memo.c_str(), MAX_MESSAGE_LEN - pos );

        // Special case: Append the node ID and RSSI (which is by definition
        // 0) if we are the gateway so that the final message conforms to
        // SNCLF

        if ( isGateway() )
        {
            messageChar[pos++] = comma;
            pos += snprintf( messageChar + pos, MAX_MESSAGE_LEN - pos, "%d", 0 );
            messageChar[pos++] = comma;
            pos += snprintf( messageChar + pos, MAX_MESSAGE_LEN - pos, "%d", _node );

        }

        // pos += snprintf( messageChar + pos, MAX_MESSAGE_LEN-pos, "%ld", (long)int(reading) );
        // messageChar[pos++] = '.';
        // pos += snprintf( messageChar + pos, MAX_MESSAGE_LEN-pos, "%0ld", (long)int(reading)*100 );
        // messageChar[pos++] = comma;

        // String message = F("R");
        // message += messageSequence;
        // message += F(",");
        // message += node.name;
        // message += F(",");
        // message += now;
        // message += F(",");
        // message += sensor;
        // message += F(",");
        // message += reading;
        // message += ",";
        // message += units;
        // message += ",";
        // message += memo;
        // message.toCharArray( messageChar, MAX_MESSAGE_LEN );
    }

    Serial.print( F("D: >>>>") );
    Serial.println(messageChar);

    unsigned int l  = strlen(messageChar);

    // Our contract states that the message must be passed "upstream"
    // If we are a gateway reporting on itself, we actually need to
    // serial print this message, not tranmit it over the radio channel

    if ( isGateway() )
    {
        Serial.println( messageChar );
    }
    else
    {
        sendWithRetry( _gateway, messageChar, l, 3, 40  );
    }

    totalLongFormMessagesSent++;
    statcycle_totalLongFormMessagesSent++;
    messageSequence++;

}

// After a lot of research, it does appear LLAP has got a really killer
// implementation at least as good as LowPower and Narcoleptic - JG 2014


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This power-saving code was shamelessly stolen from the Jeelabs library with
// slight modification. see https://github.com/jcw/jeelib The watchdog timer
// is only about 10% accurate - varies between chips

#include <avr/sleep.h>
#include <util/atomic.h>

static volatile byte watchdogCounter;

void watchdogEvent()
{
    ++watchdogCounter;
}

ISR(WDT_vect)
{
    watchdogEvent();
}

void watchdogInterrupts (char mode)
{
    // correct for the fact that WDP3 is *not* in bit position 3!
    if (mode & bit(3))
        mode ^= bit(3) | bit(WDP3);
    // pre-calculate the WDTCSR value, can't do it inside the timed sequence
    // we only generate interrupts, no reset
    byte wdtcsr = mode >= 0 ? bit(WDIE) | mode : 0;
    MCUSR &= ~(1 << WDRF);
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
#ifndef WDTCSR
#define WDTCSR WDTCR
#endif
        WDTCSR |= (1 << WDCE) | (1 << WDE); // timed sequence
        WDTCSR = wdtcsr;
    }
}

/// @see http://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
void powerDown ()
{
    byte adcsraSave = ADCSRA;
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    // switch off analog comparator - not in Jeelabs' code
    ACSR = ACSR & 0x7F; // note if using it then we need to switch this back on when we wake.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        sleep_enable();
        // sleep_bod_disable(); // can't use this - not in my avr-libc version!
#ifdef BODSE
        MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
        MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
#endif
    }
    sleep_cpu();
    sleep_disable();
    // re-enable what we disabled
    ADCSRA = adcsraSave;
}

byte Sensornet::sleepForaWhile (word msecs)
{
    byte ok = 1;

    word msleft = msecs;
    // only slow down for periods longer than the watchdog granularity
    while (msleft >= 16)
    {
        char wdp = 0; // wdp 0..9 corresponds to roughly 16..8192 ms
        // calc wdp as log2(msleft/16), i.e. loop & inc while next value is ok
        for (word m = msleft; m >= 32; m >>= 1)
            if (++wdp >= 9)
                break;
        watchdogCounter = 0;
        watchdogInterrupts(wdp);
        powerDown();
        watchdogInterrupts(-1); // off
        // when interrupted, our best guess is that half the time has passed
        word halfms = 8 << wdp;
        msleft -= halfms;
        if (watchdogCounter == 0)
        {
            ok = 0; // lost some time, but got interrupted
            break;
        }
        msleft -= halfms;
    }
    // adjust the milli ticks, since we will have missed several
#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || defined (__AVR_ATtiny44__) || defined (__AVR_ATtiny45__)
    extern volatile unsigned long millis_timer_millis;
    millis_timer_millis += msecs - msleft;
#else
    extern volatile unsigned long timer0_millis;
    timer0_millis += msecs - msleft;
#endif

    timeSpentSleeping += msecs - msleft;
    statcycle_timeSpentSleeping += msecs - msleft;

    return ok; // true if we lost approx the time planned
}

void pin2_isr()
{
    sleep_disable();
    detachInterrupt(0);
}

void pin3_isr()
{
    sleep_disable();
    detachInterrupt(1);
}

void Sensornet::sleep(byte pinToWakeOn, byte direction, byte bPullup)   // full sleep wake on interrupt - pin is 2 or 3
{
    byte adcsraSave = ADCSRA;
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    // switch off analog comparator - not in Jeelabs' code
    ACSR = ACSR & 0x7F; // note if using it then we need to switch this back on when we wake.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    if (pinToWakeOn == 2)
    {
        pinMode(2, INPUT);
        if (bPullup) digitalWrite(2, HIGH);     // enable pullup
        attachInterrupt(0, pin2_isr, direction);
    }
    else
    {
        pinMode(3, INPUT);
        if (bPullup) digitalWrite(3, HIGH);     // enable pullup
        attachInterrupt(1, pin3_isr, direction);
    }
    cli();
    // sleep_bod_disable(); // can't use this - not in my avr-libc version!
#ifdef BODSE
    MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
    MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
#endif
    sei();
    sleep_cpu();  // and wait until we are woken
    sleep_disable();
    // re-enable what we disabled
    ADCSRA = adcsraSave;
}

// End of power-saving code.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////




void print_hex(int v, int num_places)
{
    int mask = 0, n, num_nibbles, digit;

    for (n = 1; n <= num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask; // truncate v to specified number of places

    num_nibbles = num_places / 4;
    if ((num_places % 4) != 0)
    {
        ++num_nibbles;
    }

    do
    {
        digit = ((v >> (num_nibbles - 1) * 4)) & 0x0f;
        Serial.print(digit, HEX);
    }
    while (--num_nibbles);

}

void print_binary(int v, int num_places)
{
    int mask = 0, n;

    for (n = 1; n <= num_places; n++)
    {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask;  // truncate v to specified number of places

    while (num_places)
    {

        if (v & (0x0001 << num_places - 1))
        {
            Serial.print("1");
        }
        else
        {
            Serial.print("0");
        }

        --num_places;
        if (((num_places % 4) == 0) && (num_places != 0))
        {
            Serial.print("-");
        }
    }
}


void debug_cbuf(char cbuf[], int idx, bool clear)
{
    if (idx == 0)
    {
        return;
    }

    Serial.println( F("D: Dumping buffer"));
    Serial.print( F("D: ASCII:"));
    //print printable ASCII
    for (int x = 0; x < idx; x++)
    {
        if ((int)cbuf[x] < 32 || (int)cbuf[x] > 126)
        {
            // Serial.print(" ");
            //Serial.print(cbuf[x], DEC);
        }
        else
        {
            //      Serial.print( " ");
            Serial.print( cbuf[x] );
        }
        Serial.print(F("."));
    }
    Serial.println();
    Serial.print( F("D: HEX:"));
    //print HEX
    for (int x = 0; x < idx; x++)
    {
        print_hex( cbuf[x], 8);
        Serial.print(" ");
    }
    Serial.println();

    // //print binary
    // for(int x = 0; x < *idx; x++) {
    //   print_binary( cbuf[x], 8);
    //   Serial.print(",");
    // }
    // Serial.println();


    /*
      Serial.print("size of cbuf: ");
      Serial.println(sizeof(cbuf), DEC);
      Serial.print("idx: ");
      Serial.println(*idx, DEC);
    */

    if (clear)
    {
        memset(cbuf, 0, sizeof(cbuf));
        idx = 0;
    }
}

