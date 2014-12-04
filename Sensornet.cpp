#include "Sensornet.h"
#include "Arduino.h"

#include <RFM69.h> 







Sensornet::Sensornet() {
  ;
}




void Sensornet::configureRadio( int node, int network, int gateway, int frequency, char *key )
{
  _node = node;
  _network = network;
  _gateway = gateway;
  _frequency = frequency;

  radio.initialize(_frequency,_node,_network);
  radio.encrypt( key );
  
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
