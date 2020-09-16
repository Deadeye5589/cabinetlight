/*        _         _             _       _                  _          _            _            _   _            _
*         /\ \      /\ \         /\ \     /\_\               / /\       /\ \         /\ \         /\_\/\_\ _      /\ \
*        /  \ \     \_\ \       /  \ \   / / /         _    / /  \      \_\ \       /  \ \       / / / / //\_\   /  \ \
*       / /\ \ \    /\__ \     / /\ \ \  \ \ \__      /\_\ / / /\ \__   /\__ \     / /\ \ \     /\ \/ \ \/ / /__/ /\ \ \
*      / / /\ \_\  / /_ \ \   / / /\ \ \  \ \___\    / / // / /\ \___\ / /_ \ \   / / /\ \ \   /  \____\__/ //___/ /\ \ \
*     / / /_/ / / / / /\ \ \ / / /  \ \_\  \__  /   / / / \ \ \ \/___// / /\ \ \ / / /  \ \_\ / /\/________/ \___\/ / / /
*    / / /__\/ / / / /  \/_// / /    \/_/  / / /   / / /   \ \ \     / / /  \/_// / /   / / // / /\/_// / /        / / /
*   / / /_____/ / / /      / / /          / / /   / / /_    \ \ \   / / /      / / /   / / // / /    / / /        / / /    _
*  / / /\ \ \  / / /      / / /________  / / /___/ / //_/\__/ / /  / / /      / / /___/ / // / /    / / /         \ \ \__/\_\
* / / /  \ \ \/_/ /      / / /_________\/ / /____\/ / \ \/___/ /  /_/ /      / / /____\/ / \/_/    / / /           \ \___\/ /
* \/_/    \_\/\_\/       \/____________/\/_________/   \_____\/   \_\/       \/_________/          \/_/             \/___/_/
*
*
* Main Project: Schranklicht Nano
* Based on Arduino Nano board with Atmega328p chipset
*
* Module: Main.c
* 16.09.2020
* V1.1 integrated LDR measurement 
*/

//Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

//Defines
#define TRANSIONTIME  10 //Since system tick is about 100ms, this will give us a 1 second fade in or fade out duration
#define HYSTERESIS 5	//Hysteresis for night time detection, will prevent flickering due to values near day / night border

//Enumerations
enum {idle, fadein, glow, fadeout}; //Status of light engine state machine

//Global variables
volatile uint8_t duration = 50;
volatile uint8_t brightnessmax = 127;
volatile uint8_t tick = 0;
volatile uint8_t status = 0;
volatile uint8_t run = 0;
volatile uint8_t lampmode = 0;
volatile uint8_t checkpin = 0;
volatile uint8_t debounce = 0;
volatile uint8_t night = 0;
volatile uint8_t switchingthreshold = 100;

//Function declarations
void init_ports(void);
void init_timers(void);
void init_interrupts(void);
void init_adc(void);
unsigned int adc_conversion(int channel);
_Bool is_night(void);

//Linearisation table for PWM brightness
unsigned int helligkeit[128]={0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4,
	4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10,
	10, 10, 10, 10, 11, 11, 11, 11, 12, 31, 32, 34, 35, 36, 38, 39, 41, 42, 44, 45, 47,
	49, 51, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 75, 77, 79, 82, 84, 87, 89, 92,
	94, 97, 100, 103, 105, 108, 111, 114, 117, 120, 124, 127, 130, 133, 137, 140, 144, 147,
	151, 155, 158, 162, 166, 170, 174, 178, 182, 186, 190, 194, 199, 203, 208, 212, 217,
	221, 226, 231, 236, 241, 246, 251, 255
};


// function name: init_ports()
// Overall setup of I/O ports
//
void init_ports(void){
	//Set pull-ups for inputs active = 1, disabled = 0 (default)
	PORTB = 0b00000100;
	PORTC = 0b00000000;
	PORTD = 0b00010000;

	//Set direction output = 1, input = 0 (default)
	DDRB = 0b11111011;
	DDRC = 0b11100000;
	DDRD = 0b11100011;
}



// function name: init_timers()
// configuration of timers for PWM output
// and on time duration
void init_timers(void){
	//PWM for LEDs on PD6
	OCR0A = 0;
	TCCR0A = (1<<COM0A1) | (1<<WGM02) | (1<<WGM00);						//Set Phase correct PWM non inverting, enable OC0A
	TCCR0B = (1<<CS02);													//256x Prescaler = 120Hz PWM

	//Timer for system tick
	ICR1 = 780;															//Tick occurs every 100ms = 10Hz
	TCCR1A = (1<<WGM11);												//Mode 14, Fast PWM Top at ICR1
	TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS12);			//Prescaler 1024
	TIMSK1 = (1<<TOIE1);												//Enable Timer Interrupt
}


ISR(TIMER1_OVF_vect){
	tick = 1;
}


// function name: init_interrupts()
// enables the internal interrupts for
// PIR Sensor input on Pin PD2 and PD3
// Lamp mode button input on PB2 (PCINT2)
void init_interrupts(void){
	//Set interrupts
	EICRA = 0b00001111; //Rising edge of INT1 and INT0 to trigger
	EIMSK = 0b00000011; //Enable Interrupts INT1 and INT0
	PCICR = 0b00000001; //Enable Pin change interrupt group 0
	PCMSK0 = 0b00000100; //Mask out all interrupts beside PCINT2
}


ISR(INT0_vect){
	if(status == idle && night){
		status = fadein;
	}
	run = duration;
}

ISR(INT1_vect){
	if(status == idle && night){
		status = fadein;
	}
	run = duration;
}

ISR(PCINT0_vect){
	if (!(PINB & (1<<PINB2)))	//Since we have a pin change interrupt we check for low level and only then do our stuff
	{
		checkpin = 1;
	}
}

// function name: init_adc()
// configuration of ADC to read in values
// of LDR on 1 channels PC0
void init_adc(void){
	ADMUX = (1<<REFS0) | (1<<ADLAR); //AVcc with external capacitor at AREF pin and justify left
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //Enable ADC and set prescaler to x128 = 125kHz
	DIDR0 = (1<<ADC0D);  //Digital Input Disable for ADC0
}


// function name: adc_conversion()
// call with ADC channel that shall be read out
// switches MUX register according to channel,
// waits for conversion to finish and
// returns ADC value at selected channel
// Will temporary stop interrupts
unsigned int adc_conversion(int channel){
	uint8_t temp;
	cli();
	temp = ADMUX & 0xF0;			//read current setting of ADMUX register and mask out lower bits
	temp |= channel & 0x0F;			//overwrite lower bits with channel setting
	ADMUX = temp;					//write combined values back into ADMUX register
	ADCSRA |= (1<<ADSC);			//Start ADC conversion
	while(!(ADCSRA&(1<<ADIF)));		//Wait for conversion to finish
	ADCSRA|=(1<<ADIF);				//Clear interrupt flag
	sei();
	return ADCH;
}

// function name: is_night ()
// Returns true if LDR is under threshold of sensitivity
// Hysteresis on ADC values to prevent flickering at border values
_Bool is_night(void)
{
	uint8_t LDR = 0;
	
	LDR = adc_conversion(0);
	
	if (switchingthreshold > HYSTERESIS )		//Hysteresis size
	{
		if (LDR > switchingthreshold)	//Only illuminate bed if it is dark enough
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return true;
	}
	
}

void togglelampmode(void){
	if (checkpin)								//Debounce the lampmode switch and check if button is still pressed after some time
	{
		debounce++;								//For this we use the timer tick which allows us to enter this functions round about each 100ms
		if (debounce > 5)						//So 2 debounce ticks are enough that the button must be pressed at least 200ms
		{
			checkpin = 0;
			debounce = 0;
			if (!(PINB & (1<<PINB2)))			//Is the lampmode switch still pressed, than switch lampmode on or off
			{
				if (lampmode && status == glow){
					lampmode = 0;
					run = 0;
				}
				else{
					lampmode = 1;
					run = duration;
					status = fadein;
				}
			}
		}
	}	
}

int main(void)
{
	//Local variables
	uint8_t brightness = 0;
	uint8_t stepwidth = 5;

	cli();
	init_ports();
	init_timers();
	init_interrupts();
	init_adc();
	sei();
	while (1)
	{
		night = is_night();
		if(tick){				//Execute the state machine every 100ms
			tick = 0;			//Reset the tick for the next 100ms
			
			//State machine for the left side
			if(status == fadein)		//If we are in idle fade in to maximum brightness
			{
				if(brightness <= brightnessmax){	//Fade in as long as the current brightness is beneath the global brightness
					brightness += stepwidth;	//Step width is calculated inside the setup routine
					if(brightness > 127)		//Limit brightness to the size for of our Linearization array
					brightness = 127;
					OCR0A = helligkeit[brightness];	//Set the PWM
				}
				if((brightness > brightnessmax) || (brightness == 127)) //The fade in is finished when we have reached the overall brightness or if we reach the end of the Linearization array
				status = glow;
			}	
			else if(status == glow){		//Now that we reached the desired brightness keep the lights on for the set duration
				if(run > 0){				//Decrease the run variable with each timer tick. A retrigger of the motion sensor resets the run variable and prolongs the on time
					if (!lampmode)			//Also we only do this if we are not in lampmode, in lampmode the lights will be on until we leaf lampmode by another press of the matching button
						{
							run--;
						}
				}
				else status = fadeout;		//The duration was long enough we can proceed to fade out the lights
			}
			else if(status == fadeout ){			//Now fadeout with each timer tick one step at a time
				if(brightness > stepwidth){			//Take care that the next subtraction will not lead to negative values or we are out of bound of the Linearization array
					brightness -= stepwidth;
					OCR0A = helligkeit[brightness];
				}
				else{									//Since the previous subtraction would have resulted in negative values we take care of this
					brightness = 0;						//and set the brightness of this side back to 0. This is also the right value to start the fade in next time
					OCR0A = helligkeit[brightness];
					status = idle;						//We can go back to idle and wait for the next motion sensor trigger event
				}
			}
		togglelampmode();
		}
	}
}
