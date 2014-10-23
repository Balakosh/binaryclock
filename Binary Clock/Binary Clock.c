/*
 * Binary_Clock.c
 *
 * Created: 20.10.2014 16:50:32
 *  Author: Akeman & Belenon
 */ 
#ifndef F_CPU
#define F_CPU 8000000
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//Variablen für die Zeit
volatile unsigned int sekunde = 0;
volatile unsigned int minute = 15;
volatile unsigned int stunde = 12;

volatile uint8_t statusReg = 0x00;
volatile uint8_t updateBit = 0;
volatile uint8_t button1Bit = 1;
volatile uint8_t button2Bit = 2;
volatile uint8_t button3Bit = 3;
volatile uint8_t button4Bit = 4;
volatile uint8_t button5Bit = 5;
volatile uint8_t button6Bit = 6;


void toggleLED(void){
	PORTA ^= 0xff;
	PORTC ^= 1<<2 | 1<<1 | 1;
	PORTC ^= 1<<2 | 1<<1 | 1;
}

void encodeLed(void){
	
	uint8_t sz = sekunde / 10;
	uint8_t se = sekunde % 10;
	
	uint8_t mz = minute / 10;
	uint8_t me = minute % 10;
	
	uint8_t hz = stunde / 10;
	uint8_t he = stunde % 10;
	
	// LED status bestimmen Sekunden Einer
	uint8_t se8 = se >> 3;
	se8 &= ~0b11111110;
	uint8_t se4 = se >> 2;
	se4 &= ~0b11111110;
	uint8_t se2 = se >> 1;
	se2 &= ~0b11111110;
	uint8_t se1 = se & ~0b11111110;
	
	// LED status bestimmen Sekunden Zehner
	uint8_t sz4 = sz >> 2;
	sz4 &= ~0b11111110;
	uint8_t sz2 = sz >> 1;
	sz2 &= ~0b11111110;
	uint8_t sz1 = sz & ~0b11111110;
	
	// LED status bestimmen Minuten Einer
	uint8_t me8 = me >> 3;
	me8 &= ~0b11111110;
	uint8_t me4 = me >> 2;
	me4 &= ~0b11111110;
	uint8_t me2 = me >> 1;
	me2 &= ~0b11111110;
	uint8_t me1 = me & ~0b11111110;
	
	// LED status bestimmen Minuten Zehner
	uint8_t mz4 = mz >> 2;
	mz4 &= ~0b11111110;
	uint8_t mz2 = mz >> 1;
	mz2 &= ~0b11111110;
	uint8_t mz1 = mz & ~0b11111110;
	
	// LED status bestimmen Stunden Einer
	uint8_t he8 = he >> 3;
	he8 &= ~0b11111110;
	uint8_t he4 = he >> 2;
	he4 &= ~0b11111110;
	uint8_t he2 = he >> 1;
	he2 &= ~0b11111110;
	uint8_t he1 = he & ~0b11111110;
	
	// LED status bestimmen Stunden Zehner
	uint8_t hz2 = hz >> 1;
	hz2 &= ~0b11111110;
	uint8_t hz1 = hz & ~0b11111110;
	
	// IC2 clocken
	PORTA = 0x00;
	PORTA |= se8<<4 | se4<<7 | me8<<5 | he8<<6;
	PORTC ^= 1<<2;
	PORTC ^= 1<<2;
	PORTA = 0x00;
	
	
	// IC3 clocken
	PORTA = 0x00;
	PORTA |= se2<<4 | sz4 | sz2<<5 | me4<<1 | mz4<<2 | he4<<3 | me2<<6 | hz2<<7;
	PORTC ^= 1<<1;
	PORTC ^= 1<<1;
	PORTA = 0x00;
	
	// IC4 clocken
	PORTA = 0x00;
	PORTA |= se1 | sz1<<1 | me1<<2 | mz1<<3 | he1<<4 | hz1<<5 | mz2<<6 | he2<<7;
	PORTC ^= 1;
	PORTC ^= 1;
	PORTA = 0x00;	
}


int main(void)
{
	DDRA = 0xff;
	DDRB = 0b11100000;
	DDRC = 0xff;
	DDRD = 0b10111111;
	
	PORTA = 0xff;
	PORTB = 0b00011111;
	PORTC = 0x00;
	PORTD = 0b11000000;
	
	// D-Latch Clocken (leeren/initialisieren)
	PORTA = 0x00;
	PORTC = 0b00000111;
	_delay_ms(100);
	PORTC = 0b00000000;
	
	//PORTC ^= 1<<2;
	//PORTC ^= 1<<2;
	//PORTC ^= 0b00000001;
	//PORTC ^= 0b00000001;
	

	
	
	sei();
	
	TCCR1B |= 1<<CS12 | 1<<WGM12;
	TIMSK |= 1<<OCIE1A;
	OCR1A = 31250-1;
	
    while(1)
    {
        //_delay_ms(1000);
		//toggleLED();
		if (bit_is_set(statusReg, updateBit) == 1)
		{
			encodeLed();
			statusReg &= ~(1<<updateBit);
		}
		
		
		// ========= Setting Hour + =============
		if ( !( PINB & (1<<PINB3) ) )	// When pulled to LOW by pressibng button
		{
			if (bit_is_clear(statusReg, button2Bit)) // When previously not pressed
			{
				if (stunde < 23)
				{
					stunde++;
				}
				else
				{
					stunde = 0;
				}
				statusReg |= 1<<button2Bit; // Button was pressed
			}
		}
		else
		{
			statusReg &= ~(1<<button2Bit); // Button was pressed
		}
		// ==================================
		
		
		// ========= Setting Hour - =============
		if ( !( PINB & (1<<PINB4) ) )	// When pulled to LOW by pressibng button
		{
			if (bit_is_clear(statusReg, button1Bit)) // When previously not pressed
			{
				if (stunde > 1)
				{
					stunde--;
				}
				else
				{
					stunde = 23;
				}
				statusReg |= 1<<button1Bit; // Button was pressed
			}
		}
		else
		{
			statusReg &= ~(1<<button1Bit); // Button was pressed
		}
		// ==================================
				

		// ========= Setting Minuten + =============
		if ( !( PINB & (1<<PINB1) ) )	// When pulled to LOW by pressibng button
		{
			if (bit_is_clear(statusReg, button4Bit)) // When previously not pressed
			{
				if (minute < 59)
				{
					minute++;
				}
				else
				{
					minute = 0;
				}
				sekunde = 0;
				TCNT1H = 0x00;
				TCNT1L = 0x00;
				statusReg |= 1<<button4Bit; // Button was pressed
			}
		}
		else
		{
			statusReg &= ~(1<<button4Bit); // Button was pressed
		}
		// ==================================


		// ========= Setting Minuten - =============
		if ( !( PINB & (1<<PINB2) ) )	// When pulled to LOW by pressibng button
		{
			if (bit_is_clear(statusReg, button3Bit)) // When previously not pressed
			{
				if (minute > 1)
				{
					minute--;
				}
				else
				{
					minute = 59;
				}
				sekunde = 0;
				TCNT1H = 0x00;
				TCNT1L = 0x00;
				statusReg |= 1<<button3Bit; // Button was pressed
			}
		}
		else
		{
			statusReg &= ~(1<<button3Bit); // Button was pressed
		}
		// ==================================
				
		encodeLed();
    }
}


ISR (TIMER1_COMPA_vect)
{
	sekunde++;
	if(sekunde == 60)
	{
		minute++;
		sekunde = 0;
	}
	if(minute == 60)
	{
		stunde++;
		minute = 0;
	}
	if(stunde == 24)
	{
		stunde = 0;
	}
	statusReg |= 1<<updateBit;
}