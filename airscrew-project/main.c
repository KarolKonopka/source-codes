#define F_CPU 8000000
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "VL53L1X_api.h"

void i2cSetBitrate(uint16_t bitrateKHz);

void USART_Init(unsigned int ubbr);
void USART_Transmit_str(char *str);
void USART_Transmit_int(int val);
void USART_Transmit(unsigned char data);

volatile uint16_t logs[400] = {0};
volatile float height_setpoint = 1000;

ISR(INT0_vect) {
	for(int i=0; i<400; i++) {
		USART_Transmit_int(logs[i]);
		_delay_ms(1);
		USART_Transmit(0x0A);
		_delay_ms(1);
	}
	USART_Transmit(0x0D);
}

int main(void) {	
	float error = 0;
	float integral = 0;
	float derivative = 0;
	float last_error = 0;
	float control_value = 0;
	float cv_before_sat = 0;
	float integral_correction = 0;
	
	float Ts = 0.05;
	float Kp = 1.6; // 1.6
	float Ki = 0.75; // 0.75
	float Kd = 0.3; // 0.3
	float Kb = 0.1; // 0.1
	
	uint8_t state;
	uint16_t dev = 0x52;
	uint8_t dataReady;
	uint16_t height;
	uint16_t k = 0;
	
	USART_Init(MYUBRR);
	
	i2cSetBitrate(200);
	while(state == 0) {
		VL53L1X_BootState(dev, &state);
		_delay_ms(2);
	}
	VL53L1X_SensorInit(dev);
	VL53L1X_SetDistanceMode(dev, 2);
	VL53L1X_SetTimingBudgetInMs(dev, 50);
	VL53L1X_SetInterMeasurementInMs(dev, 50);
	
	DDRD |= (1<<PD3);
	PORTD |= (1<<PD3);
	
	DDRB |= (1<<PB1);
	TCCR1A |= (1<<WGM10)|(1<<WGM11)|(1<<COM1A1);
	TCCR1B |= (1<<WGM12)|(1<<CS11);
	OCR1A = 0;
	
	PORTD |= (1<<PD2);
	MCUCR |= (1<<ISC01);
	GICR |= (1<<INT0);
	
	for(int i = 0; i < 4; i++) {
		PORTD ^= (1<<PD3);
		_delay_ms(500);
	}
	
	VL53L1X_StartRanging(dev);
	sei();
	
	while(1) {
		while(dataReady == 0) VL53L1X_CheckForDataReady(dev, &dataReady);
		dataReady = 0;
		VL53L1X_GetDistance(dev, &height);
		VL53L1X_ClearInterrupt(dev);
		
		error = height_setpoint - height;
		
		if(abs(error) < 50) PORTD |= (1<<PD3);
		else PORTD &= ~(1<<PD3);
		
		integral = integral + error*Ts + Kb*integral_correction;
		derivative = (error-last_error)/Ts;
		
		cv_before_sat = Kp*error + Ki*integral + Kd*derivative;
		
		if(cv_before_sat > 1023) control_value = 1023;
		else if(cv_before_sat < 0) control_value = 0;
		else control_value = cv_before_sat;
		
		if(height < 100) {
			control_value = 0;
			OCR1A = (uint16_t)control_value;
			for(uint8_t i = 0; i < 6; i++) {
				PORTD ^= (1<<PD3);
				_delay_ms(500);
			}
			integral = 0;
		} else OCR1A = (uint16_t)control_value;
		
		integral_correction = control_value - cv_before_sat;
		last_error = error;
		
		if(k<400) {
			logs[k] = height;
			k++;
		}
	}
}

void i2cSetBitrate(uint16_t bitrateKHz) {
	uint8_t bitrate_div;

	bitrate_div = ((F_CPU/1000l)/bitrateKHz);
	if(bitrate_div >= 16)
		bitrate_div = (bitrate_div-16)/2;

	TWBR = bitrate_div;
}

void USART_Init(unsigned int ubbr) {
	UBRRH = (unsigned char)(ubbr>>8);
	UBRRL = (unsigned char)ubbr;
	UCSRB = (1<<TXEN);
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
}

void USART_Transmit_str(char * str) {
	while (*str) USART_Transmit(*str++);
}

void USART_Transmit_int(int val) {
	char bufor[17];
	USART_Transmit_str(itoa(val, bufor, 10));
}

void USART_Transmit(unsigned char data) {
	while( !(UCSRA & (1<<UDRE)) );
	UDR = data;
}