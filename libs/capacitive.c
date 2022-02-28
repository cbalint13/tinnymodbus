/*
 * capacitive.c
 *
 * Created: 25.02.2022 
 *  Author: Philipp Guth
 * 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "capacitive.h"

volatile unsigned long timer1_ovf_count = 0;

void capacitive_init() 
{
	CAP_PORT_DIR |=	(1<<CAP_OUT_PIN) |					// Set CAP_OUT_PIN as output
					(1<<CAP_IN_PIN);					// Set CAP_IN_PIN as output
	CAP_PORT &=		~(1<<CAP_OUT_PIN) &					// OUT_PIN low
					~(1<<CAP_IN_PIN);					// IN_PIN low
}

uint16_t get_capacitance(void)
{
		// Charge capacitance under test CAP_OUT_PIN and CAP_IN_PIN
		// Rising high edge on CAP_OUT_PIN
		
		CAP_PORT_DIR &= ~(1<<CAP_IN_PIN);				// Set pin CAP_IN_PIN as input
		ADMUX = (1<<MUX1);								// Enable ADC on CAP_IN_PIN
		CAP_PORT |= (1<<CAP_OUT_PIN);					// CAP_OUT_PIN high to charge cap 
		
		ADCSRA =	(1<<ADEN) |							// Enable ADC
					(1<<ADPS2) | (1<<ADPS1) |			// Set prescaler to 64 -> f_ADC=125kHz
					(1<<ADSC); 							// Start AD-conversion 
		while (ADCSRA & (1<<ADSC));						// Wait until conversion done
		int low  = ADCL;
		int high = ADCH;
		int val = (high << 8) | low;

		capacitive_init();		
	
	if (val < 800)
	{
		//Low value capacitor
		//short capacitance = (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);
		//return capacitance;
		return (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);
	}
	else
	{
		// Big capacitor, use RC charging method

		_delay_ms(1);										// Wait for full discharge

		// setup timer 2
		TCCR1 = 0x00;										// Timer in normal mode
		TCNT1 = 0x00;										// Clear timer value
		timer1_ovf_count = 0;								// Clear timer value
		
		TIMSK = (1<<TOIE1);									// enable interrupt on overflow 
		sei();												// enable global interrupts

		ADMUX =  (1<<MUX0);									// AD-MUX on CAP_OUT_PIN

		// Start charging the capacitor trough the internal pull-up resistor 
		CAP_PORT_DIR  &=  ~(1<<CAP_OUT_PIN);				// Set OUT_PIN as input
		CAP_PORT |= (1<<CAP_OUT_PIN);						// activate IN_PIN pull-up resistor -> starts charging cap through pullup resistor 
		TCCR1 = (1<<CS11) | (1<<CS10);						// Prescaler 4 - 2MHz - start timer2
		
		while ( !((CAP_PORT_PIN) & (1 << (CAP_OUT_PIN))) && // Wait until PIN_OUT goes high at around 2,5V
				(timer1_ovf_count < MAX_TIMER1_OVF) );		// or timeout
		TCCR1 &= ~( (1<<CS11) | (1<<CS10) ) ;				// stop timer2
		CAP_PORT &= ~(1<<CAP_OUT_PIN);						// stop charging by disabeling the pullup resistor
		// read value at OUT_PIN
		ADCSRA =	(1<<ADEN) |								// Enable ADC
					(1<<ADPS2) | (1<<ADPS1) |				// Set prescaler 
					(1<<ADSC); 								// Start AD-conversion 
		while(ADCSRA & (1<<ADSC));							// Wait until conversion done
		int low =	ADCL;
		int high =	ADCH;
		int val = (high << 8) | low;

		capacitive_init();

		float t_us = ((float)timer1_ovf_count + (float)TCNT1/256) * 128;

		// Calculate capacitance
		uint16_t capacitance = -(float)t_us / R_PULLUP / log(1.0 - (float)val / (float)MAX_ADC_VALUE);
		if (timer1_ovf_count >= MAX_TIMER1_OVF)				// return 0 if timeout
		{
			capacitance = 0;
		}

		return capacitance;
	}
}

uint16_t get_capacitance_avg(uint8_t n_measures)
{
	long sum_capacitance = 0;
	int i;
	for (i=0; i < n_measures; ++i)
	{
		sum_capacitance += get_capacitance();
	}
	return sum_capacitance / n_measures;
}

ISR (TIMER1_OVF_vect)
{
  /* Interrupt all 128us */
	timer1_ovf_count += 1;
}