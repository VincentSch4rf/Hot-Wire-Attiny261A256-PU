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
#define highscore 18

#define led 1
#define seg 2

#define STATE_SLEEP 0
#define STATE_INIT 1
#define STATE_READY 2
#define STATE_PLAY 3
#define STATE_WIN 4
#define STATE_LOOSE 5

uint8_t STATE = -1;
uint8_t led_seg = led;
uint8_t contacts, seconds, minutes, hours, interruptFlag;
uint16_t units, compsunits, analog_voltage;
uint8_t analog_timeout, button_pressed, button_was_pressed, tick, timeout, old_timeout, timeout_value, best;
uint16_t button_timeout, global_duration, note_count;

void init_osc() {
	// Change to 1MHz by changing clock prescaler to 8
	cli();										// Disable interrupts
	CLKPR = _BV(CLKPCE);						// Pre-scaler enable
	CLKPR = _BV(CLKPS1) |_BV(CLKPS0);			// Clock division factor 8 (0010)
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
		case loose:	PORTA |= 0b01101010; break;
		case highscore: PORTA |= 0b01101000; break;
		default:	PORTA |= 0b11111111; break;
	}

}

void power_down() {
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	ADCSRA = 0;									// Power down ADC
	power_all_disable ();
	sleep_enable();
	sleep_cpu();
	sleep_disable();							// Disable sleep
	ADCSRA = _BV(ADEN);							// Power on ADC
	power_all_enable();
}

void sleep() {
	set_sleep_mode(SLEEP_MODE_IDLE);			// Set sleep-mode
	sleep_enable();
	sleep_mode();
	sleep_disable();
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

/* NOTES */
const int Note_C1  = 478;
const int Note_CS1 = 451;
const int Note_D1  = 427;
const int Note_DS1 = 402;
const int Note_E1  = 375;
const int Note_F1  = 358;
const int Note_FS1 = 338;
const int Note_G1  = 319;
const int Note_GS1 = 301;
const int Note_A1  = 284;
const int Note_AS1 = 284;
const int Note_B1  = 268;
const int Note_H1  = 253;
const int Note_C2  = 239;
const int Note_CS2 = 225;
const int Note_D2  = 213;
const int Note_DS2 = 201;
const int Note_E2  = 190;
const int Note_F2  = 179;
const int Note_FS2 = 169;
const int Note_G2  = 159;
const int Note_GS2 = 150;
const int Note_A2  = 142;
const int Note_AS2 = 134;
const int Note_B2  = 134;
const int Note_H2  = 127;
const int Note_C3  = 119;
const int Note_D3  = 107;
const int Note_DS3 = 100;
const int Note_E3  = 95;
const int Note_F3  = 89;
const int Note_FS3 = 84;
const int Note_G3  = 80;
const int Note_GS3 = 75;
const int Note_A3  = 71;
const int Note_B3  = 67;
const int Note_H3  = 63;
const int Note_C4  = 60;
const int Note_D4  = 53;
const int Note_DS4 = 50;
/* END */

void tone(uint16_t note, uint16_t duration) {
	// Set up Timer/Counter1 for PWM output
	if(note != 0) {
		OCR1D  = 0;
		TCCR1B = 0;
		global_duration = duration;
		OCR1D  = note;
		TCCR1B = _BV(CS12) |_BV(CS10);								// 1:8 prescale
		TCCR1C = 1<<COM1A0S | 1<<COM1D0 | 1<<PWM1D;		// PWM D, clear on match
		TCCR1D |= _BV(WGM11);
	}
}

void notone() {
	TCCR1B = 0;
	TCCR1C = 0;
	TCCR1D = 0;
}

int main(void)
{
	init_osc();									// Initialize Oscillator
	init_ports();								// Initialize the ports
	best = 13;
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
	while (1) {
		switch (STATE) {
			case STATE_SLEEP:
				note_count = 0;
				contacts = 0;
				seconds = 0;
				interruptFlag = 0;
				analog_timeout = 0;
				timeout = 16;
				timeout_value = 4;
				power_down();
				break;
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
			case STATE_READY: sleep(); break;
			case STATE_PLAY: sleep(); break;
			case STATE_LOOSE: sleep(); break;
			case STATE_WIN:  sleep(); break;
		}
		// Empty like my head
	}
	return 0;
}

ISR(PCINT_vect)
{
	if(STATE == STATE_SLEEP) {
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
	analog_voltage = ADCH;						// Output ADCH to variable
	ADCSRA |= 1<<ADSC;							// Start Conversion
}

ISR(TIMER0_COMPA_vect)
{
	//if(button_timeout > 0) button_timeout++;			// Timeout calculation for beep
	//if(button_pressed == 1) button_timeout++;			// Start timeout for beep
	/* BEGIN: Button pressed handling */
	/*if(button_pressed == 0  && button_timeout == global_duration) {	// If timeout strikes
		//								// Speaker off
		button_timeout = 0;
		note_count++;
		button_pressed = 1;
	}*/
		//							// Speaker on
		//
	/* END */
	
	if(STATE == STATE_PLAY) {
		/* BEGIN BUTTON HANDLING */
		if(button_pressed == 1) {
			STATE = STATE_WIN;
		}
		/* END */
		
		/* BEGIN: ADC Interrupt_Handling */
		if(analog_timeout == 0) {
			if(analog_voltage == 0.0) {
				PORTB |= _BV(PB4);		// Speaker on
				old_timeout = timeout;
				timeout = 31;			// All LED´s on
				contacts++;
				analog_timeout++;
			}
		} else if(analog_timeout == 25) {
			PORTB &= ~_BV(PB4);			// Speaker off
			timeout = old_timeout;		// Back to timeout
			analog_timeout = 0;
			/* GAME HANDLING */
			if(contacts == 12) {
				STATE = STATE_LOOSE;
			}
			/* END */
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
		/* END */
	
		/* BEGIN: LED and Segment Flashing */
		// Switch between led and 7-segment with 100Hz => 50Hz refreshrate each
		// Turn on LED
		if(led_seg == led)
		{
			LED_out(timeout);
			led_seg = seg;
		}
		// Turn on Segment
		else if(led_seg == seg)
		{
			seg_out(contacts%16);
			led_seg = led;
		}
		/* END */
		
		/* GAME HANDLING */
		if(seconds == timeout_value) {
			if(timeout == 1) {
				seg_out(19);
				STATE = STATE_LOOSE;
			}
			timeout_value = (timeout_value+4);
			timeout = timeout/2;
		}
		/* END */
		
	} else if(STATE == STATE_READY) {
		seg_out(best);
		/* BEGIN BUTTON HANDLING */
		if(button_pressed == 1) {
			// Play "Star Trek" theme (somehow the tones are not clean do not know why, the calculation of the frequencies should be correct)
			if(button_timeout == global_duration) {
				switch(note_count) {
					case 0: tone(Note_F2, 70); break;
					case 1: tone(Note_DS3, 105); break;
					case 2: tone(Note_D3, 35); break;
					case 3: tone(Note_C3, 23); break;
					case 4: tone(Note_B2, 23); break;
					case 5: tone(Note_A2, 23); break;
					case 6: tone(Note_GS2, 245); break;
					/*case 7: tone(Note_FS2, 35); break;
					case 8: tone(Note_F2, 70); break;
					case 9: tone(Note_G3, 105); break;
					case 10: tone(Note_DS3, 35); break;
					case 11: tone(Note_D3, 23); break;
					case 12: tone(Note_C3, 23); break;
					case 13: tone(Note_B2, 23); break;
					case 14: tone(Note_A2, 245); break;*/
					case 7:
						notone();
						button_pressed = 0;
						STATE = STATE_PLAY;
						break;
				}
				button_timeout = 0;
				note_count++;
				if(STATE == STATE_PLAY) {
					note_count = 0;
					global_duration = 0;
				}
			} else {
				button_timeout++;
			}
		}
	} else if(STATE == STATE_LOOSE) {
		seg_out(loose);
		if(button_timeout == global_duration) {
			switch(note_count) {
			case 0: tone(Note_C4, 105); break;
			case 1: tone(Note_B3, 105); break;
			case 2: tone(Note_A3, 105); break;
			case 3: tone(Note_G2, 105); break;
			case 4:
				notone();
				LED_out(0);
				seg_out(19);
				STATE = STATE_SLEEP;
				break;
			}
			button_timeout = 0;
			note_count++;
		} else {
			button_timeout++;
		}
	} else if(STATE == STATE_WIN) {
		if(contacts < best) {
			best = contacts;
			seg_out(highscore);
		} else {
			seg_out(win);
		}
		if(button_timeout == global_duration) {
			switch(note_count) {
				case 0: tone(Note_C4, 105); break;
				case 1: tone(Note_C3, 105); break;
				case 2: tone(Note_C4, 105); break;
				case 3:
				notone();
				LED_out(0);
				seg_out(19);
				STATE = STATE_SLEEP;
				break;
			}
			button_timeout = 0;
			note_count++;
		} else {
			button_timeout++;
		}
	}
}