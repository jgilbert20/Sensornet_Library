#include "Sensornet.h"
#include "Arduino.h"

#include <RFM69.h> 

Sensornet::Sensornet() {
  ;
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


unsigned long quantaStartTime;
int currentCodebook;
compactedMessage compactedMessageBuffer;


void Sensornet::setCodebook( int codebook )
{
  currentCodebook = codebook;

}

#define SENSORNET_NOT_POPULATED -999999.99

void Sensornet::newQuanta()
{
  quantaStartTime = millis();
  for( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
    compactedMessageBuffer.reading[i] = SENSORNET_NOT_POPULATED;

  compactedMessageBuffer.type = 'C';
  compactedMessageBuffer.sequence = messageSequence;

}


sensorType coreCodebook[14] = { SENSOR_TEST_A, SENSOR_TEST_B, HTU21D_RH, HTU21D_C };

int findIndexForSensor( sensorType *codebook, sensorType query )
{
  for( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
    if( query == codebook[i] )
        return i; 

  return -1;
}

void Sensornet::queueReading( sensorType sensor, float value )
{
  // Look up the index in the current codebook
  int index = findIndexForSensor( coreCodebook, sensor );
  compactedMessageBuffer.reading[index] = value;
}

const char* COMMA = ",";


typedef struct sensorDescriptor 
{
  const char *name;
  const char *unit;
} sensorDescriptor;

const sensorDescriptor sensorLookup[] = 
{
  { "BATT-V",                        "volts" },
  { "HTU21D-RH",                     "%RH"},
  { "HTU21D-C",                      "C"},
  { "TSL2591-Lux",                   "lux" },
  { "TSL2591-Full",                  "raw" },
  { "TSL2591-IR",                    "raw" },
  { "LUX-FLT",                       "lux" },
  { "LUX-FLT-Broad",                 "raw" },
  { "LUX-FLT-Infra",                 "raw" },
  { "Dallas",                        "C" },

  { "BMP-Temp",                      "C" },
  { "BMP-Pressure",                  "Pa" },
  { "Current-A",                     "Arms" },
  { "Current-B",                     "Arms" },
  { "AM2315-Temp",                   "C" },
  { "AM2315-RH",                     "%RH" },
  { "MCP9808",                       "C" },
  { "SHT15-Temp",                    "C" },
  { "SHT15-RH",                      "%RH" },
  { "Therm0-Temp",                   "C" },
  { "Therm1-Temp",                   "C" },

  { "Sensor-Test-A",                 "unita"},
  { "Sensor-Test-B",                 "unitb"},

  { "Radio-Ack-SiBoot",              "%" },
  { "Radio-Ack-SiLast",              "%" },
  { "Radio-Ack-EWMA",                "%" },
  { "T-AvTx-SiBoot",                 "ms" },
  { "T-AvTx-SiLast",                 "ms" },
  { "T-AvTx-EWMA",                   "ms" },
  { "T-Loop-SiBoot",                 "ms" },
  { "T-Loop-SiLast",                 "ms" },
  { "T-DutyCycle-SiBoot",            "%" },
  { "T-DutyCycle-SiLast",            "%" },
  { "Total-Loops",                   "loops" },

  { "LUX-LOW",                       "lux" },
  { "LUX-Ratio",                     "%" },
  { "LUX-LOW-Broad",                 "raw" },
  { "LUX-LOW-Infra",                 "raw" },
  { "DUM",                           "na" },
  { "MDWIND",                        "raw" },
  { "Radio-BG-RSSI",                 "DB" },
  { "Radio-Recent-BG-RSSI",          "DB" },
  { "SI1145-Vis",                    "raw" },
  { "SI1145-IR",                     "raw" },
  { "SI1145-UvIndex",                "index" }
};

inline const char *getSensorName( sensorType t )
{
  return sensorLookup[t].name;
}

inline const char *getSensorUnits( sensorType t )
{
  return sensorLookup[t].unit;
}


int Sensornet::writeCompressedPacketToSerial( nodeID origin, char *buffer )
{
    compactedMessage *msg = &compactedMessageBuffer;
    nodeDescriptor sender = getNodeDescriptor( origin );

   // if( buffer == null )
   //     return -1;

    if( msg->type != 'C' )
      return -1;

     for( int i = 0 ; i < SN_CODEBOOK_MAX_SIZE ; i++ )
     {
        if( msg->reading[i] == SENSORNET_NOT_POPULATED )
          continue;

        Serial.print( "C" );
        Serial.print( msg->sequence );
        Serial.print( "-" );
        Serial.print( i );
        Serial.print( COMMA );
        Serial.print( sender.name );
        Serial.print( COMMA );
        Serial.print( msg->timestamp );
        Serial.print( COMMA );
        Serial.print( getSensorName( coreCodebook[i] ) );
        Serial.print( COMMA );
        Serial.print( msg->reading[i] );
        Serial.print( COMMA );
        Serial.print( getSensorUnits( coreCodebook[i] ) );
        Serial.print( COMMA ); 
        Serial.println();
     }

}


// void queueReading

void Sensornet::flushQueue()
{
    radio.sendWithRetry( _gateway, (char *)&compactedMessageBuffer,  sizeof(compactedMessageBuffer), 3, 30  );  

    messageSequence++;  
}

void Sensornet::sendReading( String sensor, float reading, String units )
{
    sendStructured( sensor, reading, units, F("") );
}


void Sensornet::sendStructured( String sensor, float reading, String units, String memo )
{
    nodeDescriptor node = getNodeDescriptor( (nodeID) _node );


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
