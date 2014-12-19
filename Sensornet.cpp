#include "Sensornet.h"
#include "Arduino.h"

#include <RFM69.h> 


#define SENSORNET_NOT_POPULATED -999999.99


unsigned long quantaStartTime;
int currentCodebookIndex;
sensorType *currentCodebook;
compactedMessage compactedMessageBuffer;

const char* COMMA = ",";




int codebookRegistry[][SN_CODEBOOK_MAX_SIZE] =
{
  { 0,1,2,3,4,5,6,7,8,9,10,22,23 },
  { SENSOR_TEST_A, SENSOR_TEST_B, HTU21D_RH, HTU21D_C },
  { 0,1,2,3,4,5,6,7,8,9,10,11,12 }
};


typedef struct sensorDescriptor 
{
  const __FlashStringHelper *name;
  const __FlashStringHelper *unit;
} sensorDescriptor;



//sensorType testCodebook[SN_CODEBOOK_MAX_SIZE] = { SENSOR_TEST_A, SENSOR_TEST_B, HTU21D_RH, HTU21D_C };
// int baseHewsCodebook[SN_CODEBOOK_MAX_SIZE] = { 0,1,2,3,4,5,6,7,8,9,10,11,12 };





#define SENSORNET_SENSOR_LUT_SIZE  25

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
  setSensorMap( 2,  F("HTU21D-C"),                      F("C")         );
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
  setSensorMap( 22, F("Sensor-Test-A"),                 F("unita")      );
  setSensorMap( 23, F("Sensor-Test-B"),                 F("unitb")      );
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








Sensornet::Sensornet() {
  ;
  currentCodebook  = null; 
  populateSL();
}


// const char* SENSORCONFIG[][3] =
// {
//   { HTU21D_RH, "HTU21D-RH", "%RH" },
//   { HTU21D_C, "HTU21D-C", "C" },
// };



 
//   ;
// }




nodeDescriptor getNodeDescriptor(nodeID id)
{

  nodeDescriptor n;

  switch( id )
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

  radio.initialize(_frequency,_node,_network);
  radio.encrypt( key );
 
  messageSequence = 0;

}



void Sensornet::setCodebook( int codebook )
{
  currentCodebookIndex = codebook;
  currentCodebook = (sensorType *)codebookRegistry[codebook];
  compactedMessageBuffer.codebookID = codebook;

}


void Sensornet::newQuanta()
{
  quantaStartTime = millis();
  for( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
    compactedMessageBuffer.reading[i] = SENSORNET_NOT_POPULATED;

  compactedMessageBuffer.type = 'C';
  compactedMessageBuffer.sequence = messageSequence;

}



int findIndexForSensor( sensorType *codebook, sensorType query )
{
  if( codebook == null)
    return -2;

  for( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
    if( query == codebook[i] )
        return i; 

  return -1;
}


int Sensornet::queueReading( sensorType sensor, float value )
{
  // Look up the index in the current codebook
  int index = findIndexForSensor( currentCodebook, sensor );
 
  if( index < 0 )
      return -1;

  compactedMessageBuffer.reading[index] = value;
  return 0;
}


inline const __FlashStringHelper *getSensorName( sensorType t )
{
  return sensorLookup[t].name;
}

inline const __FlashStringHelper *getSensorUnits( sensorType t )
{
  return sensorLookup[t].unit;
}

int Sensornet::getSensorIDforName( const char *n )
{
  for( int i = 0 ; i < SENSORNET_SENSOR_LUT_SIZE ; i++ )
  {
      if( strcmp_P( n, (const char *)sensorLookup[i].name ) == 0 )
          return i;
  }

  return -1;

}

 int Sensornet::getSensorIDforName( String n )
{
  for( int i = 0 ; i < SENSORNET_SENSOR_LUT_SIZE ; i++ )
    {
      if( strcmp_P(  n.c_str(), (const char *)sensorLookup[i].name ) == 0 )
          return i;
      }

return -1;

}


int Sensornet::writeCompressedPacketToSerial( nodeID origin, char *buffer )
{
    compactedMessage *msg = &compactedMessageBuffer;
    nodeDescriptor sender = getNodeDescriptor( origin );

   // if( buffer == null )
   //     return -1;

    if( msg->type != 'C' )
      return -1;

    setCodebook( msg->codebookID );

     for( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
     {
        if( msg->reading[i] == SENSORNET_NOT_POPULATED )
          continue;

        Serial.print( "C" );
        Serial.print( msg->sequence );
        Serial.print( "-cbp" );
        Serial.print( i );
            Serial.print( "-cb" );
                Serial.print( msg->codebookID );
            Serial.print( "-cbi" );
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
        Serial.println();
     }

     return 0;
}


// void queueReading

void Sensornet::flushQueue()
{
    radio.sendWithRetry( _gateway, (char *)&compactedMessageBuffer,  sizeof(compactedMessageBuffer), 3, 30  );  
    currentCodebook = null;

    messageSequence++;  
}

void Sensornet::sendReading( String sensor, float reading, String units )
{
    sendStructured( sensor, reading, units, F("") );
}


void Sensornet::sendStructured( String sensor, float reading, String units, String memo )
{
    nodeDescriptor node = getNodeDescriptor( (nodeID) _node );

 // Serial.println( "D: send structured: Sending reading for " + sensor + "value " + reading );

  if( currentCodebook != null )
  {


    // Check to see if this sensor fits into our current codebook
    int sensorID = getSensorIDforName( sensor );

   Serial.print( "D: Looking up " + sensor );
Serial.print( "Got:" );
   Serial.print( sensorID );
    Serial.println();

    if( sensorID >= 0 )
    {
        // Looks like we could queue this into the current codebook

       Serial.println( "D: Adding to compressed queue" );
      queueReading( (sensorType) sensorID, reading );


      return;
    }
  }

    long unsigned int now = millis();
    char messageChar[MAX_MESSAGE_LEN+1];
    { 
      String message = F("R");
      message += messageSequence;
      message += F(",");
      message += node.name;
      message += F(",");
      message += now;
      message += F(",");
      message += sensor;
      message += F(",");
      message += reading;
      message += ",";
      message += units;
      message += ",";
      message += memo;
      message.toCharArray( messageChar, MAX_MESSAGE_LEN );
  } 

    Serial.print( F(">>>>") );
    Serial.println(messageChar);

    unsigned int l  = strlen(messageChar);

    radio.sendWithRetry( _gateway, messageChar,  l, 3, 30  );  

    messageSequence++;

}


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This power-saving code was shamelessly stolen from the Jeelabs library with slight modification.
// see https://github.com/jcw/jeelib
// The watchdog timer is only about 10% accurate - varies between chips


#include <avr/sleep.h>
#include <util/atomic.h>

static volatile byte watchdogCounter;

void watchdogEvent() {
    ++watchdogCounter;
}

ISR(WDT_vect) { watchdogEvent(); }

void watchdogInterrupts (char mode) {
    // correct for the fact that WDP3 is *not* in bit position 3!
    if (mode & bit(3))
        mode ^= bit(3) | bit(WDP3);
    // pre-calculate the WDTCSR value, can't do it inside the timed sequence
    // we only generate interrupts, no reset
    byte wdtcsr = mode >= 0 ? bit(WDIE) | mode : 0;
    MCUSR &= ~(1<<WDRF);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
#ifndef WDTCSR
#define WDTCSR WDTCR
#endif
        WDTCSR |= (1<<WDCE) | (1<<WDE); // timed sequence
        WDTCSR = wdtcsr;
    }
}

/// @see http://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
void powerDown () {
    byte adcsraSave = ADCSRA;
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    // switch off analog comparator - not in Jeelabs' code
    ACSR = ACSR & 0x7F; // note if using it then we need to switch this back on when we wake.
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
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

byte Sensornet::sleepForaWhile (word msecs) {
    byte ok = 1;
    word msleft = msecs;
    // only slow down for periods longer than the watchdog granularity
    while (msleft >= 16) {
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
        if (watchdogCounter == 0) {
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

void Sensornet::sleep(byte pinToWakeOn, byte direction, byte bPullup)	// full sleep wake on interrupt - pin is 2 or 3
{
  byte adcsraSave = ADCSRA;
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  // switch off analog comparator - not in Jeelabs' code
  ACSR = ACSR & 0x7F; // note if using it then we need to switch this back on when we wake.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  if (pinToWakeOn == 2)
  {
	pinMode(2,INPUT);
    if (bPullup) digitalWrite(2,HIGH);		// enable pullup
	attachInterrupt(0, pin2_isr, direction);
  }
  else
  {
	pinMode(3,INPUT);
    if (bPullup) digitalWrite(3,HIGH);		// enable pullup
	attachInterrupt(1, pin3_isr, direction);
  }
  cli();
  // sleep_bod_disable(); // can't use this - not in my avr-libc version!
#ifdef BODSE
    MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
    MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
#endif
  sei();
  sleep_cpu();	// and wait until we are woken
  sleep_disable();
  // re-enable what we disabled
  ADCSRA = adcsraSave;
}

// End of power-saving code.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
