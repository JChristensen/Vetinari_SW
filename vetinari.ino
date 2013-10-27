//ATtiny84A Vetinari Clock Driver
//Developed with Arduino 1.0.5 and Arduino-Tiny, http://code.google.com/p/arduino-tiny/
//Set fuse bytes (L/H/E): 0x62/0xD6/0xFF (1MHz internal RC osc, 1.8V BOD, EESAVE)
//
//Jack Christensen 13Aug2013
//
//CC BY-SA
//ATtiny84A Vetinari Clock Driver by Jack Christensen is licensed under
//the Creative Commons Attribution-ShareAlike 3.0 Unported License.
//To view a copy of this license, visit //http://creativecommons.org/licenses/by-sa/3.0/
//or send a letter to Creative Commons, 171 Second Street, Suite 300,
//San Francisco, California, 94105, USA.
//
//Hardware: http://goo.gl/Rsk2Ki
//Software: http://goo.gl/oXYS8Y

//pin assignments (* = pullup on)
//PA0 * Not used (optional diagnostic LED)
//PA1 * mode button
//PA2   clock coil drive
//PA3   clock coil drive
//PA4   SCK/SCL (RTC, external pullup)
//PA5 * Not used (MISO used for ICSP only)
//PA6   MOSI/SDA (RTC, external pullup)
//PA7 * Not used
//PB0 * Not used (XTAL1)
//PB1 * Not used (XTAL2)
//PB2 * Interrupt from RTC
//PB3   RESET (external pullup)

#include <MCP79412RTC.h>              //http://github.com/JChristensen/MCP79412RTC
#include <Time.h>                     //http://playground.arduino.cc/Code/Time
#include <TinyWireM.h>                //http://github.com/JChristensen/TinyWireM
#include <avr/sleep.h>                //http://winavr.sourceforge.net
#include <util/delay.h>               //http://winavr.sourceforge.net

//fast port manipulation macros
#define COIL_1_ON PORTA |= _BV(PORTA2)
#define COIL_1_OFF PORTA &= ~_BV(PORTA2)
#define COIL_2_ON PORTA |= _BV(PORTA3)
#define COIL_2_OFF PORTA &= ~_BV(PORTA3)

const int COIL_DELAY = 30;            //milliseconds between coil polarity reversals
volatile boolean fRtcInt;             //RTC interrupt flag set by the ISR (RTC 1Hz interrupt)
volatile boolean fBtnInt;             //button interrupt flag set by the ISR

void setup(void)
{
    PORTA = _BV(PORTA0) | _BV(PORTA1) | _BV(PORTA5) | _BV(PORTA7);    //pullups on
    PORTB = _BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2);                  //pullups on
    DDRA |= _BV(DDA2) | _BV(DDA3);    //configure coil pins as outputs
    RTC.set(1376352000);              //start the RTC, set date/time to 13Aug2013 00:00:00 (an arbitrary time)
    RTC.vbaten(false);                //disable battery backup
    RTC.calibWrite(RTC.eepromRead(127));    //calibrate RTC using value previously stored in its EEPROM
    RTC.squareWave(SQWAVE_1_HZ);            //start the 1Hz interrupts from the RTC
    DDRA &= ~( _BV(DDA4) | _BV(DDA6) );     //done with the i2c bus, tri-state the pins
    PORTA &= ~( _BV(PORTA4) | _BV(PORTA6) );    //ensure pullups are off (have external pullups)
    ADCSRA &= ~_BV(ADEN);             //not using the ADC, so disable it

    PCMSK0 = _BV(PCINT1);             //enable pin change interrupt for mode button
    GIFR = _BV(PCIF0);                //clear interrupt flag
    GIMSK = _BV(PCIE0);               //enable pin change interrupt

    PCMSK1 = _BV(PCINT10);            //enable pin change interrupt for rtc
    GIFR = _BV(PCIF1);                //clear interrupt flag
    GIMSK |= _BV(PCIE1);              //enable pin change interrupt
}

void loop(void)
{
    static boolean fVetinari = true;  //flag to switch between Vetinari mode and normal clock mode
    static int8_t missedTicks = 0;

    if (fRtcInt) {                    //1Hz RTC interrupt?
        fRtcInt = 0;
        if (fVetinari) {
            if ( (rand() & 1) || (missedTicks >= 14) ) {
                clockTick(missedTicks);
                missedTicks = 0;
            }
            else {
                ++missedTicks;
            }
        }
        else {
            clockTick(0);
        }
        GIFR |= _BV(PCIF0);           //ensure button interrupt flag is clear
        GIMSK |= _BV(PCIE0);          //re-enable button interrupt
    }

    if (fBtnInt) {                    //mode button interrupt?
        fBtnInt = 0;
        fVetinari = !fVetinari;       //toggle clock mode
        while (!fRtcInt);             //wait for next tick
        fRtcInt = 0;
        clockTick(missedTicks);       //catch up if we toggled to normal mode
        missedTicks = 0;
        _delay_ms(COIL_DELAY);        //a little additional button debounce
    }

    gotoSleep();
}

//advance the clock by (nTicks + 1) seconds
void clockTick (int8_t nTicks)
{
    static boolean polarity = false;

    for(int8_t t=nTicks; t>=0; --t) {
        if (polarity = !polarity) {
            COIL_1_ON;
            _delay_ms(COIL_DELAY);
            COIL_2_ON;
            if (t != 0 ) _delay_ms(COIL_DELAY);
        }
        else {
            COIL_1_OFF;
            _delay_ms(COIL_DELAY);
            COIL_2_OFF;
            if (t != 0 ) _delay_ms(COIL_DELAY);
        }
    }
}

//1Hz interrupt from the RTC
//Using pin change interrupt, so it actually occurs twice
//per second, so we ignore every other one.
ISR(PCINT1_vect)
{
    static uint8_t intCount;

    if (++intCount & 1) {
        fRtcInt = true;
    }
}

//mode button switches between Vetinari mode and normal mode
ISR(PCINT0_vect)
{
    GIMSK &= ~_BV(PCIE0);             //no more button interrupts (until next clock tick)
    fBtnInt = true;
}

void gotoSleep(void)
{
    byte mcucr1, mcucr2;

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    cli();                            //stop interrupts to ensure the BOD timed sequence executes as required
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);    //turn off the brown-out detector
    mcucr2 = mcucr1 & ~_BV(BODSE);              //if the MCU does not have BOD disable capability, then this code has no effect
    MCUCR = mcucr1;
    MCUCR = mcucr2;
    sei();                            //ensure interrupts enabled so we can wake up again
    sleep_cpu();                      //go to sleep
    sleep_disable();                  //wake up here
}
