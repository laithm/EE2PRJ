/*
 * Multifunctional-Robot.c
 *
 * Created: 18/02/2026 12:06:20
 * Author : 230025817
 */ 

#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void ADC_Init() {
	ADMUX = (1 << REFS0);                                // Vref: AVCC [cite: 225]
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Enable ADC, /64 prescaler
}

uint16_t ADC_Read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x07);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

void PWM_Timer1_Init() {
	/* Set PD5 (OC1A) and PD4 (OC1B) as output [cite: 103, 110] */
	DDRD |= (1 << PD5) | (1 << PD4);
	
	/* Timer 1: Fast PWM, 8-bit mode (WGM10=1, WGM12=1) */
	/* Non-inverting output on OC1A and OC1B */
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler 8
}

int main(void) {
	/* Set Direction pins PD2 and PD3 as output [cite: 87, 96] */
	DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD6) | (1 << PD7);
	
	ADC_Init();
	PWM_Timer1_Init();

	while(1) {
		// Simple Forward Logic
		PORTD |= (1 << PD2);  // AIN1 High
		PORTD &= ~(1 << PD3); // AIN2 Low
		PORTD |= (1 << PD6); //BIN1 High
		PORTD &= ~(1 << PD7); // BIN2 Low
		
		
		// Read Potentiometer on PA0 (Header J3) [cite: 13]
		// Map 10-bit ADC (0-1023) to 8-bit PWM (0-255)
		uint8_t speed = (ADC_Read(0) >> 2);
		
		OCR1A = speed; // Controls PWMA (PD5) [cite: 110]
		OCR1B = speed; // Controls PWMB (PD4)
		
		_delay_ms(10);
	}
}