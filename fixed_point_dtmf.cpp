/*
	DTMF library for Arduino

	Copyright (c) 2014 Freedelity

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
 */

#include "fixed_point_dtmf.h"

#define N 136 // number of samples
static int32_t THRESHOLD = (int32_t)1000;
static int counter;
static const int16_t coeff[7] = {0x7294, 0x6FD2, 0x6CD3, 0x6999, 0x5A82, 0x51C5, 0x4856};

static const int dtmf_map[12] = {
  0x11,
  0x21,
  0x41,
  0x12,
  0x22,
  0x42,
  0x14,
  0x24,
  0x44,
  0x28,
  0x18,
  0x48
};

static const char dtmf_char[12] = {
  '1',
  '2',
  '3',
  '4',
  '5',
  '6',
  '7',
  '8',
  '9',
  '0',
  '*',
  '#'
};

char last_dtmf = 0;

static int16_t delay_0[7] = {0};
static int16_t delay_1[7] = {0};
static int16_t delay_2[7] = {0};
static int32_t prod[7] = {0};
static int samples[N];

void fid_dtmf_init(int32_t threshold)
{
	THRESHOLD = threshold;
	cli();
	// init ADC
	ADCSRA &= 0x10;	// make sure ADC is stopped and interrupt disabled
	ADCSRA |= 0x10;	// clear interrupt flag
	ADMUX = 0x40;	// ARef = AVcc, right-adjusted, and channel 0
	ADCSRA |= 0x80; // enable ADC
	ADCSRA |= 0x2F;	// enable interrupts and set prescaler to 128 (ADC frequency = 125kHz), and auto-trigger mode
	ADCSRB &= 0xF8; // free-running mode (trigger is interrupt flag)
	counter = 0;
	sei();
	
	// launch conversions
	ADCSRA |= 0x40;
}

/*
 * Implementation of the Goertzel algorithm using fixed-point arithmetic.
 * It is taken from http://www.ti.com/ww/cn/uprogram/share/ppt/c6000/Chapter17.ppt
 * and a bit derived to compute the several frequencies of DTMF.
 */
char fid_dtmf_digit()
{
	char digit = 0;
	static int i = 0;
	int16_t input;
	
	if( counter > 0 && i < counter )
	{
		input = (int16_t)(samples[i]-512);
		//input = input >> 4; // Scale down input to prevent overflow
		
		/*for(int j=0; j<7; ++j)
		{
			prod[j] = ((int32_t)delay_1[j] * coeff[j]) >> 14; // not shifted by 15 as the coeff is already divided by 2
			delay_0[j] = input + (int16_t)prod[j] - delay_2[j];
			delay_2[j] = delay_1[j];
			delay_1[j] = delay_0[j];
		}*/
		prod[0] = ((int32_t)delay_1[0] * coeff[0]) >> 14; // not shifted by 15 as the coeff is already divided by 2
		delay_0[0] = input + (int16_t)prod[0] - delay_2[0];
		delay_2[0] = delay_1[0];
		delay_1[0] = delay_0[0];
		prod[1] = ((int32_t)delay_1[1] * coeff[1]) >> 14;
		delay_0[1] = input + (int16_t)prod[1] - delay_2[1];
		delay_2[1] = delay_1[1];
		delay_1[1] = delay_0[1];
		prod[2] = ((int32_t)delay_1[2] * coeff[2]) >> 14;
		delay_0[2] = input + (int16_t)prod[2] - delay_2[2];
		delay_2[2] = delay_1[2];
		delay_1[2] = delay_0[2];
		prod[3] = ((int32_t)delay_1[3] * coeff[3]) >> 14;
		delay_0[3] = input + (int16_t)prod[3] - delay_2[3];
		delay_2[3] = delay_1[3];
		delay_1[3] = delay_0[3];
		prod[4] = ((int32_t)delay_1[4] * coeff[4]) >> 14;
		delay_0[4] = input + (int16_t)prod[4] - delay_2[4];
		delay_2[4] = delay_1[4];
		delay_1[4] = delay_0[4];
		prod[5] = ((int32_t)delay_1[5] * coeff[5]) >> 14;
		delay_0[5] = input + (int16_t)prod[5] - delay_2[5];
		delay_2[5] = delay_1[5];
		delay_1[5] = delay_0[5];
		prod[6] = ((int32_t)delay_1[6] * coeff[6]) >> 14;
		delay_0[6] = input + (int16_t)prod[6] - delay_2[6];
		delay_2[6] = delay_1[6];
		delay_1[6] = delay_0[6];
		
		if( ++i == N )
		{
			counter = 0;
			i=0;
			int32_t prod1, prod2, prod3, magnitude;
			int dtmf=0, bit=1, m;
			
			for(int j=0; j<7; ++j)
			{
				prod1 = (int32_t)delay_1[j] * delay_1[j];
				prod2 = (int32_t)delay_2[j] * delay_2[j];
				prod3 = ((int32_t)delay_1[j] * coeff[j])>>14;
				prod3 *= (int32_t)delay_2[j];
				
				magnitude = (prod1 + prod2 - prod3) >> 15;
				
				delay_1[j] = delay_2[j] = 0;
				
				if( magnitude > THRESHOLD )
				{
					dtmf |= bit;
				}
				bit <<= 1;
			}
			
			for(m=0; m<12; ++m)
			{
				if(dtmf_map[m] == dtmf)break;
			}
			
			if(m < 12)
			{
				// wait for the button to be released
				if(dtmf_char[m] != last_dtmf)
				{
					digit = dtmf_char[m];
					last_dtmf = digit;
				}
			}
			else
			{
				last_dtmf = 0;
			}
		}
		
	}
	
	return digit;
}

// Interrupt routine called when ADC is complete
ISR(ADC_vect)
{
	// ignore if we are still computing the previous block
	if( counter < N )
	{
		samples[counter] = ADCL | (ADCH << 8);
		counter++;
	}
}
