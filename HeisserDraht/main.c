/*
 * HeisserDraht.c
 *
 * Created: 04.12.2017 13:47:27
 * Author : Vincent Scharf
 */ 

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#define win   16
#define loose 17

#define led 1
#define seg 2

#define STATE_SLEEP 0
#define STATE_INIT 1
#define STATE_READY 2
#define STATE_PLAY 3

uint8_t STATE = -1;
uint8_t led_seg = led;
uint8_t contacts, seconds, minutes, hours, interruptFlag;
uint16_t units, compsunits, analog_voltage;
uint8_t analog_timeout, button_pressed, button_was_pressed, tick, button_timeout;

void init_osc() {
	// Change to 1MHz by changing clock prescaler to 8
	cli();										// Disable interrupts
	CLKPR = _BV(CLKPCE);						// Pre-scaler enable
	CLKPR = _BV(CLKPS1) |_BV(CLKPS0);			// Clock division factor 8 (0010)
}

void power_down() {
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	ADCSRA = 0;									// Power down ADC
	power_all_disable ();
	sleep_enable();
	sleep_cpu();
}

void sleep() {
	set_sleep_mode(SLEEP_MODE_IDLE);			// Set sleep-mode
	sleep_enable();
	sleep_mode();
}

/*Initialize ports:
	PA0, PA1, PA1, PA3 ... PA7 Output
	PA2 Input
	PB3 Output
*/
inline void init_ports()
{
	DDRA |= 0b11111011;							// PA2 as input, rest output
	DDRB |= _BV(PB3) | _BV(PB4);				// PB3 and PB4 as output, rest input

	PORTA = _BV(PA2);							// Enable PA2 pull-up resistor
	PORTB = _BV(PB6);							// Enable PB6 pull-up resistor
}

void LED_out(uint8_t out)
{
	PORTB &= ~_BV(PB3);
	PORTA &= 0b00000111;
	PORTA |= out<<3;
}

void seg_out(uint8_t out)
{
	PORTB |= _BV(PB3);
	PORTA &= 0b00000100;

	switch(out)
	{
		case 0:		PORTA |= 0b00000001; break;
		case 1:		PORTA |= 0b10011011; break;
		case 2:		PORTA |= 0b00100010; break;
		case 3:		PORTA |= 0b00001010; break;
		case 4:		PORTA |= 0b10011000; break;
		case 5:		PORTA |= 0b01001000; break;
		case 6:		PORTA |= 0b01000000; break;
		case 7:		PORTA |= 0b00011011; break;
		case 8:		PORTA |= 0b00000000; break;
		case 9:		PORTA |= 0b00001000; break;
		case 0xA:	PORTA |= 0b00010000; break;
		case 0xB:	PORTA |= 0b11000000; break;
		case 0xC:	PORTA |= 0b01100001; break;
		case 0xD:	PORTA |= 0b10000010; break;
		case 0xE:	PORTA |= 0b01100000; break;
		case 0xF:	PORTA |= 0b01110000; break;
		case win:	PORTA |= 0b11000011; break;
		case loose:	PORTA|= 0b01101010; break;
		default:	PORTA |= 0b00000001; break;
	}

}

int main(void)
{
	init_osc();									// Initialize Oscillator
	init_ports();								// Initialize the ports
	contacts = 0;
	interruptFlag = 0;
	analog_timeout = 0;
	PCMSK0 = 0;									// Sometimes needed because the register does not get erased when programming (god knows why)
	PCMSK1 = _BV(PCINT14);						// Set port toggle interrupt bit for PB6
	GIMSK = 0b00110000;
	//PCMSK1 = 0b01000000;
	//GIMSK   |= _BV(INT0);						// Enable INT0
	//MCUCR   |= _BV(ISC01);					// Trigger INT0 on falling edge (ground)
	//MCUCR	&= ~_BV(ISC00);
	OCR0A  |= 39;
	TCCR0A |= 1;								// Set timer mode to ctc
	TIMSK  |= _BV(OCIE0A);						// Set counter compare match interrupt bit
	sei();										// Enable interrupts globally
	TCCR0B |= _BV(CS02);						// Set pres-caler to 1/256
	//Timer starts
 
	/* 
		8MHz/1024 = 0.007813MHz --> 128us
		50Hz => T=20ms => 100Hz
		10ms/256us = 39.9
		39.9 = 13 => every 10th step set start of CompareReg to 1 every 30th step set it to 2 --> (100/10)*1 + (100%30)*1 = 13
	*/
	STATE = STATE_SLEEP;
	power_down();
	while (1) {
		switch (STATE) {
			case STATE_SLEEP: power_down(); break;
			case STATE_INIT: 
				//Set timer compare vector to 10000 microseconds (100Hz --> 50Hz per module)
				ADCSRA = 0b10001111;			/* Enable the ADC and its interrupt feature
												* and set the ACD clock pre-scalar to clk/128 */
				ADCSRB |= _BV(REFS2);
				ADMUX = 0b10000010;				/* Select internal 2.56V as Vref, left justify
												* data registers and select ADC2 as input channel */
				ADCSRA |= _BV(ADSC);			// Start Conversion
				STATE = STATE_READY;
				sleep();
				break;
			case STATE_READY: STATE = STATE_PLAY; break;
			case STATE_PLAY: sleep(); break;
		}
		// Empty like my head
	}
	return 0;
}

ISR(PCINT_vect)
{
	sleep_disable();							// Disable sleep
	if(STATE == STATE_SLEEP) {
		power_all_enable();						// Power everything back on
		STATE = STATE_INIT;
	}
	if(tick == 0){
		button_pressed = 1;
		tick = 1;
		} else if(tick == 1) {
		tick = 0;
	}
}

/*ADC Conversion Complete Interrupt Service Routine (ISR)*/
ISR(ADC_vect)
{
	sleep_disable();							// Disable sleep
	analog_voltage = ADCH;						// Output ADCH to variable
	ADCSRA |= 1<<ADSC;							// Start Conversion
}

ISR(TIMER0_COMPA_vect)
{
	if(button_timeout > 0) button_timeout++;			// Timeout calculation for beep
	if(button_pressed == 1) button_timeout++;			// Start timeout for beep
	/* BEGIN: Button pressed handling */
	if(button_pressed == 0  && button_timeout == 50) {	// If timeout strikes
		PORTB &= ~_BV(PB4);								// Speaker off
		button_timeout = 0;
	}
	if(button_pressed == 1) {							// If button was pressed
		PORTB |= _BV(PB4);								// Speaker on
		button_pressed = 0;
	}
	/* END */

	/* BEGIN: ADC Interrupt_Handling */
	if(analog_timeout == 0) {
		if(analog_voltage == 0.0) {
			contacts++;
			analog_timeout++;
		}
	} else if(analog_timeout == 25) {
		analog_timeout = 0;
	} else if(analog_timeout > 0) {
		analog_timeout++;
	}
	/* END */

	/* BEGIN: Timer handling */
	units++;									// Increase 10ms unit counter
	compsunits++;								// Increase compensation counter
	if(compsunits%2 == 0) { 
		OCR0A |= 1;
	} else if(compsunits%5 == 0) {				// Set
		OCR0A |=1;
		compsunits = 0;
	}
	if(units == 100) {							// Set second
		units = 0;
		seconds++;
	}
	if(seconds == 60)							// Set minute
	{
		seconds = 0;
		minutes++;
	}
	if(minutes == 60)							// Set hour
	{
		minutes = 0;
		hours++;
	}
	if(hours > 23)								// Reset
	{
		hours = 0;
	}
	/* END */
	
	/* BEGIN: LED and Segment Flashing */
	// Switch between led and 7-segment with 100Hz => 50Hz refreshrate each
	if(units%2 == 0) {
		led_seg = led;
	} else {
		led_seg = seg;
	}
	
	// Turn on LED
	if(led_seg == led)
	{
		LED_out(contacts%16);
		led_seg = seg;
	}
	// Turn on Segment
	else if(led_seg == seg)
	{
		seg_out(contacts%16);
		led_seg = led;
	}
	/* END */
}