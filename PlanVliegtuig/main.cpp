/*
 * PlanVliegtuig.cpp
 *
 * Created: 29-11-2019 17:42:12
 * Author : Ch4oZ
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "LightweightRingBuff.h"
#include <stdbool.h>

uint8_t EEMEM deviceConfig = {
	0x14 // Address = 20
};

bool beacon_light = true;
bool beacon_light_bottom = true;
bool strobe_light = false;
int strobe_count = 0;

#define FOSC F_CPU // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define CMD_BUFFER_SIZE 12

#define RANDOMNESS 30

#define BEACON_BOTTOM_OFF() PORTB &= ~(1 << PINB0);  // = led off
#define BEACON_BOTTOM_ON() PORTB |=  (1 << PINB0);  // = led on
#define BEACON_TOP_OFF() PORTB &= ~(1 << PINB1);  // = led off
#define BEACON_TOP_ON() PORTB |=  (1 << PINB1);  // = led on

#define STROBE_WINGS_OFF() PORTB &= ~(1 << PINB3);  // = led off
#define STROBE_WINGS_ON() PORTB |=  (1 << PINB3);  // = led on
#define STROBE_TAIL_OFF() PORTB &= ~(1 << PINB4);  // = led off
#define STROBE_TAIL_ON() PORTB |=  (1 << PINB4);  // = led on

#define NAV_LIGHT_TAIL_OFF() PORTB &= ~(1 << PINB2);  // = led off
#define NAV_LIGHT_TAIL_ON() PORTB |=  (1 << PINB2);  // = led on
#define NAV_LIGHT_TAIL_TOGGLE() PORTB ^= (1 << PINB2); // Toggle

#define RWY_TURNOFF_LIGHT_LEFT_OFF() PORTC &= ~(1 << PINC0);  // = led off
#define RWY_TURNOFF_LIGHT_LEFT_ON() PORTC |=  (1 << PINC0);  // = led on
#define RWY_TURNOFF_LIGHT_LEFT_TOGGLE() PORTC ^= (1 << PINC0); // Toggle
#define RWY_TURNOFF_LIGHT_RIGHT_OFF() PORTC &= ~(1 << PINC1);  // = led off
#define RWY_TURNOFF_LIGHT_RIGHT_ON() PORTC |=  (1 << PINC1);  // = led on
#define RWY_TURNOFF_LIGHT_RIGHT_TOGGLE() PORTC ^= (1 << PINC1); // Toggle

#define NAV_LIGHT_WINGS_OFF() PORTD &= ~(1 << PIND2);  // = led off
#define NAV_LIGHT_WINGS_ON() PORTD |=  (1 << PIND2);  // = led on
#define NAV_LIGHT_WINGS_TOGGLE() PORTD ^= (1 << PIND2); // Toggle
#define LANDING_LIGHT_OFF() PORTD &= ~(1 << PIND3);  // = led off
#define LANDING_LIGHT_ON() PORTD |=  (1 << PIND3);  // = led on
#define LANDING_LIGHT_TOGGLE() PORTD ^= (1 << PIND3); // Toggle
#define LANDING_LIGHT_INBOUND_OFF() PORTD &= ~(1 << PIND4);  // = led off
#define LANDING_LIGHT_INBOUND_ON() PORTD |=  (1 << PIND4);  // = led on
#define LANDING_LIGHT_INBOUND_TOGGLE() PORTD ^= (1 << PIND4); // Toggle

#define TR_REAR_LIGHT_OFF() PORTD &= ~(1 << PIND5);  // = led off
#define TR_REAR_LIGHT_ON() PORTD |=  (1 << PIND5);  // = led on
#define TR_REAR_LIGHT_TOGGLE() PORTD ^= (1 << PIND5); // Toggle
#define TR_FRONT_LIGHT_OFF() PORTD &= ~(1 << PIND6);  // = led off
#define TR_FRONT_LIGHT_ON() PORTD |=  (1 << PIND6);  // = led on
#define TR_FRONT_LIGHT_TOGGLE() PORTD ^= (1 << PIND6); // Toggle
#define TR_INNER_LIGHT_OFF() PORTD &= ~(1 << PIND7);  // = led off
#define TR_INNER_LIGHT_ON() PORTD |=  (1 << PIND7);  // = led on
#define TR_INNER_LIGHT_TOGGLE() PORTD ^= (1 << PIND7); // Toggle

#define BEACON_LIGHTS_ON() { beacon_light = true; beacon_light_bottom = true ; } // = top on, bottom on
#define BEACON_LIGHTS_OFF() { beacon_light = false; beacon_light_bottom = false; } // = top off, bottom off
#define BEACON_LIGHTS_TOGGLE() { beacon_light = !beacon_light; (beacon_light == true ? beacon_light_bottom = true : false); }  // = Toggle
#define BEACON_LIGHTS_BOTTOM_ON() beacon_light_bottom = true ; // = bottom on, top on/off
#define BEACON_LIGHTS_BOTTOM_OFF() { beacon_light_bottom = false; beacon_light = false; } // = bottom off, top off
#define BEACON_LIGHTS_BOTTOM_TOGGLE() { beacon_light_bottom = !beacon_light_bottom; (beacon_light_bottom == false ? beacon_light = false : false); } // = Toggle


#define STROBE_LIGHTS_ON() strobe_light = true ; // = on
#define STROBE_LIGHTS_OFF() { strobe_light = false ; strobe_count = 0; STROBE_WINGS_OFF(); STROBE_TAIL_OFF();} // = off
#define STROBE_LIGHTS_TOGGLE() strobe_light = !strobe_light; // = Toggle

RingBuff_t Buffer;

// The inputted commands are never going to be
// more than 8 chars long.
// volatile so the ISR can alter them
unsigned char command_in[CMD_BUFFER_SIZE];
uint8_t data_count; // doesn't need volatile ?
unsigned char command_ready;
uint8_t ignore;
uint16_t lastCmdCount =0;


volatile uint8_t pwm_phase = 0;
volatile uint8_t tel = 0;

volatile uint8_t misschien =30;

unsigned char eAddress = 0;

void USART_Init(unsigned int ubrr)
{
	/* Set baud rate */	
	UBRR0H = (unsigned char)(ubrr>>8);	// Load upper 8-bits of the baud rate value into the high byte of the UBRR register
	UBRR0L = (unsigned char)ubrr;		// Load lower 8-bits of the baud rate value into the low byte of the UBRR register
	
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<RXCIE0); //|(1<<TXEN);
	
	/* Set frame format: 8data, 2stop bit, no parity */
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes
	
	data_count = 0;
	command_ready = false;
	ignore = true;
}

// Reset to 0, ready to go again
/*
void resetBuffer(void) {
	//for( int i = 0; i < CMD_BUFFER_SIZE;  ++i ) {
	//rx_buffer[i] = 0;
	//}
	command_in[0] = 0;
	data_count = 0;
}*/

/*
 * ISR RX complete
 * Receives a char from UART and stores it in ring buffer.
 */
ISR(USART_RX_vect) {
	//	unsigned char value;
	//	value = UDR;  // Fetch the received byte value into the variable "value"
	//	UDR = value;    //Put the value to UDR = send

	// Get data from the USART in register
	unsigned char temp = UDR0;
	
	if (RingBuffer_IsFull(&Buffer) == false) {
		RingBuffer_Insert(&Buffer, temp);
	}
}


uint8_t myRandomValue(uint8_t ibase, uint8_t irand) {
	return ibase +(rand() / (RAND_MAX / irand + 1));
}

ISR(TIMER2_OVF_vect)
{
	/*
	// begin pwm leds
	for (uint8_t i=0;i<N_LED;i++) {
		if( pwm_phase == 100)
		{
			if (ledData[i].ledPin >= 5 && ledData[i].ledPin <= 7) {
				PORTD &= ~(1<< ledData[i].ledPin); // LED uit
			} else {
				PORTB &= ~(1<< ledData[i].ledPin); // LED uit
			}
		}
		if( ledData[i].currentValue == pwm_phase && pwm_phase != 100)
		{
			if (ledData[i].ledPin >= 5 && ledData[i].ledPin <= 7) {
				PORTD |= (1<< ledData[i].ledPin); // LED aan
				} else {
				PORTB |= (1<< ledData[i].ledPin); // LED aan
			}
		}
	}
	if (pwm_phase == 100) {
		pwm_phase = 0;
	} else {
		pwm_phase++;
	}
	// end pwm leds
	*/
}

ISR(TIMER0_OVF_vect)
{			
	if (tel == 1 && beacon_light_bottom == true) {
		BEACON_BOTTOM_ON();
	}
	if (tel == 3) {
		BEACON_BOTTOM_OFF();
	}
	
	if (tel == 45 && beacon_light == true) {
		BEACON_TOP_ON();
	}
	if (tel == 47) {
		BEACON_TOP_OFF();
	}
	
	if (strobe_light == true) {
		if (++strobe_count == 90) {
			strobe_count = 0;
		}
		if (strobe_count == 1) {
			STROBE_WINGS_ON();
			STROBE_TAIL_ON();
		}
	}
	if (strobe_count == 2) {
		STROBE_WINGS_OFF();
		STROBE_TAIL_OFF();
	}
	
	if (++tel == 90) {

		/*
		PORTB ^= (1 << PINB3) | (1 << PINB4); // Toggle the LEDs		
		*/
		tel = 0;
	}	
}

/*
  Read random seed from eeprom and write a new random one.
*/
void initrand()
{
        uint32_t state;
        static uint32_t EEMEM sstate = 1;

        state = eeprom_read_dword(&sstate);

        // Check if it's unwritten EEPROM (first time). Use something funny
        // in that case.
        if (state == 0xffffffUL)
                state = 0xDEADBEEFUL;
        srand(state);
		
		state = !state;
        eeprom_write_dword(&sstate,rand());			
		
		misschien = RANDOMNESS;	// start lights on power up
}

void toggleRandom() {
	uint8_t iPin =  myRandomValue(0,20);
	
	switch (iPin) {
		case 1:
			NAV_LIGHT_WINGS_TOGGLE();
			break;
		case 2:
			LANDING_LIGHT_TOGGLE();
			break;
		case 3:
			LANDING_LIGHT_INBOUND_TOGGLE();
			break;
		case 4:
			TR_REAR_LIGHT_TOGGLE();
			break;
		case 5:
			TR_FRONT_LIGHT_TOGGLE();
			break;
		case 6:
			TR_INNER_LIGHT_TOGGLE();
			break;
		case 7:
			NAV_LIGHT_TAIL_TOGGLE();
			break;
		case 8:
			STROBE_LIGHTS_TOGGLE();
			break;
		case 9:
			BEACON_LIGHTS_TOGGLE();
			break;
		case 10:
			BEACON_LIGHTS_BOTTOM_TOGGLE();
			break;
		case 11:
			RWY_TURNOFF_LIGHT_LEFT_TOGGLE();
			break;
		case 12:
			RWY_TURNOFF_LIGHT_RIGHT_TOGGLE();
			break;
		default:
			break;
	}
}

int main(void)
{
	DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3) |(1<<DDB4); /* D8 t/m D12*/	
	DDRC |= (1<<DDC0) | (1<<DDC1) ; // A0 & A1 = output	
	DDRD |= (1<<DDD2) | (1<<DDD3) | (1<<DDD4) |(1<<DDD5) | (1<<DDD6) | (1<<DDD7) ; // D2 t/m D7 = output
	
	PORTB &= ~( (1<<PINB0) | (1<<PINB1) |(1<<PINB2) | (1<<PINB3)|(1<<PINB4) );
	PORTC &= ~( (1<<PINC0) | (1<<PINC1) );
	PORTD &= ~((1<<DDD2) | (1<<DDD3) | (1<<DDD4) |(1<<PIND5) | (1<<PIND6) | (1<<PIND7) );
	
	eAddress = eeprom_read_byte(&deviceConfig);	
	
	initrand();	
	
	// Initialize the buffer with the created storage array
	RingBuffer_InitBuffer(&Buffer);
		
	// Setup Timer 0
	
	TCCR0A = 0b00000000;   // Normal Mode
	TCCR0B =  (1<<CS00) | (1<<CS02);   // Div 1024 Prescaler
	TCNT0 = 0;            // Initial value
	
	// Enable interrupts as needed
	TIMSK0 |= (1<<TOIE0); //(1<<OCIE0A);      // Timer 0 Interrupt	
	
	// Setup Timer 2
	
	TCCR2A = 0b00000000;   // Normal Mode
	TCCR2B =  (1<<CS21) ;   // Div 1024 Prescaler
	TCNT2 = 0;            // Initial value

	// Enable interrupts as needed
	TIMSK2 |= (1<<TOIE2); //(1<<OCIE0A);      // Timer 2 Interrupt	
	
	USART_Init(MYUBRR);
	 		
	sei();               // Global Interrupts
	
	bool endOfCommand = false;
	
	uint16_t doNothingTime = 0;
	
    /* Replace with your application code */
    while (1) 
    {				 
		 // Print contents of the buffer one character at a time		 
		 while (RingBuffer_IsEmpty(&Buffer) == false && command_ready == false) {
			unsigned char c = RingBuffer_Remove(&Buffer);							
			
			if (data_count >= CMD_BUFFER_SIZE) {
				// too much data
				endOfCommand = false;
				data_count = 0;
				ignore = true;
			}					
				
			if (data_count == 0 && c == eAddress) { // 255 = Address for programming new Address
				// wrong address
				ignore = false;
			}
			
			command_in[data_count++] = c;	
			
			if (c == '\r') { // End of line!		
				if (endOfCommand == true) {
					if (ignore == false) {
						command_ready = true;
						// process command
					} else {						
						data_count  =0;
						ignore = true;						
					}					
					lastCmdCount = 8192;					
				}				
			} else {
				endOfCommand = false;
			}
			if (c == '\n') { // End of line!
				endOfCommand = true;				
			}
		 }
		
		if (command_ready == true)	{							
			if (command_in[0] == eAddress && data_count >= 3) { // Set LED values								
				uint8_t iLeds = command_in[1];
				uint8_t iLeds2 = command_in[2];

					if ( (iLeds & (1 << 0)) != 0) {
						TR_INNER_LIGHT_ON();
					} else {
						TR_INNER_LIGHT_OFF();
					}
					
					if ( (iLeds & (1 << 1)) != 0) {
						TR_FRONT_LIGHT_ON();
					} else {
						TR_FRONT_LIGHT_OFF();
					}
					
					if ( (iLeds & (1 << 2)) != 0) {
						TR_REAR_LIGHT_ON();
					} else {
						TR_REAR_LIGHT_OFF();
					}				
					
					if ( (iLeds & (1 << 3)) != 0) {
						NAV_LIGHT_WINGS_ON();
					} else {
						NAV_LIGHT_WINGS_OFF();
					}

					if ( (iLeds & (1 << 4)) != 0) {
						LANDING_LIGHT_ON();
					} else {
						LANDING_LIGHT_OFF();
					}
					
					if ( (iLeds & (1 << 5)) != 0) {
						LANDING_LIGHT_INBOUND_ON();
					} else {
						LANDING_LIGHT_INBOUND_OFF();
					}
					
					if ( (iLeds & (1 << 6)) != 0) {
						BEACON_LIGHTS_ON();
					} else {
						BEACON_LIGHTS_OFF();
					}
					
					if ( (iLeds & (1 << 7)) != 0) {
						NAV_LIGHT_TAIL_ON();
					} else {
						NAV_LIGHT_TAIL_OFF();
					}
					
					if ( (iLeds2 & (1 << 0)) != 0) {
						STROBE_LIGHTS_ON();
					} else {
						STROBE_LIGHTS_OFF();
					}
			}			
			data_count =0;						
			command_ready = false;
			ignore = true;
		}
		
		if (doNothingTime > 0) {
			doNothingTime--;
		} else {
			if (misschien ==  RANDOMNESS ) {		
				uint8_t iX = myRandomValue(0,4);		
				for (uint8_t i = 0;i<iX;i++) {
					toggleRandom();	
				}
				doNothingTime = myRandomValue(100,150)*100;
			}
			if (lastCmdCount >0) {
				lastCmdCount--;
				if (lastCmdCount == 0) {
					/*
					for (uint8_t i=0;i<N_LED;i++) {
						ledData[i].ledMode = 1; // Do random value stuff
					}
					*/
				}
			} else {
				doNothingTime = 10000;
				misschien =  myRandomValue(0,254);
			}
		}
    }
	return(0);
}