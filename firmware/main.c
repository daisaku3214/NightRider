/* Name: main.c
 * Author: <hayasi daisaku>
 * Copyright: <free>
 * License: <I'm forgot it>
 */
//======================================================================
//circut construction
/*/
 output			--->PORTCfrom1to4,PORTDfrom3
 in_nonpullup	--->PORTC0(ADC)
 in_pullup		--->PORTDfrom5to7
 pwm_output		--->PORTB1
 ADC_input		--->PORTC0
 NC				--->PORTC5,PORTD4,PORTB0,PORTBfrom2to5
 CLOCKs			--->PORTB6&7
 LED			--->PORTCfrom1to4 & PORTfrom0to3
 /*/
//======================================================================
//initial defining
#define F_CPU 20000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define MASKB 0b00000010/*PORTBoutputmask*/
#define PULLB 0b00000000/*PORTBpullupinput*/
#define MASKC 0b00011110/*PORTCoutputmask*/
#define PULLC 0b00000000/*PORTCpullupinput*/
#define MASKD 0b00001111/*PORTDoutputmask*/
#define PULLD 0b11100000/*PORTDpullupinput*/

#define TRYST 250//


#define ON	 0xff
#define OFF	 0x00

//======================================================================
//function prottype decaring
void LightLED(unsigned char data);
unsigned char ReadSW(unsigned char num);
void timer2int(void);
void timer2open(void);
void adcint(void);
void adcopen(void);
void adcclose(void);
void pwm1int(void);
void pwm1open(void);
void pwm1close(void);
void pwm1Dset(unsigned int bottom ,unsigned int count);
void timer0int(void);
void timer0open(void);
void SPstart(unsigned int sp_lengh , unsigned int bottom , unsigned int count);
unsigned int TRGset(unsigned char mode);
//======================================================================
//gloabal variable
volatile unsigned char T2prescaler;
volatile unsigned char stop;
volatile unsigned char ADdata;
volatile unsigned int PWM_INT;
volatile unsigned int PWM_DUTY;
volatile unsigned int SP_prescaler;
volatile unsigned char LED_PWM_duty;
volatile unsigned char LED_var;
volatile unsigned char LED_value[8];
//======================================================================
//main function
int main(void)
{
	cli();
	int i,j;
	unsigned char temp;
	unsigned char led;
	unsigned char SWcount=0;
	unsigned char SWflag=0;
	unsigned char NRmode = 0;
	unsigned char NRphase = 0;
	unsigned int NRspeed = 0;
	unsigned int NRcount = 0;
	unsigned char NRstate = 0; 
	unsigned char NRledR = 0;
	unsigned char NRledL = 0;
	unsigned int NRtrig = 2500;
	
	//******************************************************************
	//initializing PORT
	DDRB = 0b00000111;/*alloutput(without clock)*/
	DDRC = 0b00111110;/*input PC0*/
	DDRD = 0b00011111;/*input PD5PD6PD7*/
	
	PORTB = 0x00;/*all0*/
	PORTC = 0x00;/*all0*/
	PORTD = 0b11100000;/*PD5PD6PD7pullup*/
	//******************************************************************	
	//
	adcint();
	timer2int();
	timer0int();
  //	pwm1int();
	//******************************************************************	
	//all interrupt permitting
	stop = 1;
	timer2open();
	timer0open();
	sei();
  //	pwm1open();
  //	pwm1Dset(4000, 3000);
	//******************************************************************	
	i = 0;
	j = 0;
	temp = 0;
	adcopen();
	LED_PWM_duty = 0;
	SPstart(50,4000,1000);
	while (1) {
		if (stop) {
      ;
		}
		else {
      //**********************************************************
      //**********************************************************
      //
			stop = 1;
      //
      //**********************************************************
      //**********************************************************						
			adcopen();
			NRspeed = ((((unsigned int)ADdata)*100)/0xff)*10 + 10;
			//*********************************************************
			//Read swich
			if (SWflag) {
				if (ReadSW(1) + ReadSW(2) + ReadSW(3) < 3) {
					SWcount = 0;//pushed swicth 
				}
				else {
					if (SWcount > 20) {
						SWcount = 0;
						SWflag = 0;	//break the flag
					}
					else {
						SWcount++;
					}
          
				}
        
			}
			else {
				if (ReadSW(1) + ReadSW(2) + ReadSW(3) < 2) {
					SWflag = 1;//pushed swicth are more than 2
				}
				else {
					if (!ReadSW(1)) {
						if (SWcount > 20) {
							i = 0;
							SWcount = 0;
							SWflag = 1;
							NRcount = 0;
							NRphase = 0;
							SPstart(50,8000,2000);
							i = 0;
							j = 0;
							LED_PWM_duty = 0;
							//and do SW1 function
							if (NRstate) {
								NRstate = 0;
								NRtrig  = 2500;
							}
							else {
								NRstate = 1;
								NRtrig  = TRGset(NRmode);
							}
              
						}
						else {
							SWcount++;
						}
					}
					else if (!ReadSW(2)) {
						if (SWcount > 20) {
							i = 0;
							SWcount = 0;
							SWflag = 1;
							NRcount = 0;
							NRphase = 0;
							SPstart(50,6000,1500);
							//and do SW2 function
							if (NRmode == 0) {
								NRmode  = 9;
							}
							else {
								NRmode--;
							}
						}
						else {
							SWcount++;
						}
					}
					else if (!ReadSW(3)) {
						if (SWcount > 20) {
							i = 0;
							SWcount = 0;
							SWflag = 1;
							NRcount = 0;
							NRphase = 0;
							SPstart(50,5000,1250);
							//and do SW3 function
							if (NRmode == 9) {
								NRmode  = 0;
							}
							else {
								NRmode++;
							}
						}
						else {
							SWcount++;
						}
					}
					else {
						SWcount = 0;
					}
          
					
				}
			}
			//*********************************************************
			//intialize Night Rider
			
			//*********************************************************
			//drive Night Rider
			if (NRstate == 0) {
				if (i == 0) {
					if (NRphase == 0x8f) {
						i = 1;
					}
					else if (NRcount*NRspeed > TRYST) {
						NRphase++;
						NRcount = 0;
					}
					else {
						NRcount++;
					}	
				}
				else {
					if (NRphase == 0) {
						i = 0;
						if (j ==0)
							j = 1;
						else
							j = 0;
					}
					else if (NRcount*NRspeed > TRYST) {
						NRphase--;
						NRcount = 0;
					}
					else {
						NRcount++;
					}	
				}
				if (NRmode==0) {
					LED_var = 0xff;
				}
        else if(NRmode==1){
          if (j == 0) {
            LED_var = 0xf0;
          }
          else {
            LED_var = 0x0f;
          }
        }
        else if(NRmode==2){
          if (j == 0) {
            LED_var = 0b11001100;
          }
          else {
            LED_var = 0b00110011;
          }          
        }

				else{
					if (j == 0) {
            LED_var = 0b10101010;					
					}
					else {
						LED_var = 0b01010101;					
					}
				}
				LED_PWM_duty = NRphase;
        
			}
			else {
			
			}
		}
		
	}
  return 0;   /* never reached */
}
//======================================================================
//other functions 
/* lighting LED function */
void LightLED(unsigned char data)
{
	PORTC = PULLC|(MASKC & (data << 1));
	PORTD = PULLD|(MASKD & (data >> 4));
	
}
/*Reading Swicth*/
unsigned char ReadSW(unsigned char num)
{
	unsigned char temp;
  
	if (num <= 0) {
		return(0xff);
	}
	else if(num >= 4) {
		return(0xff);
	}
	else {
		temp = PIND;
		temp = temp <<(3-num);
		temp = temp >> (7);
		
		return(temp);
	}
}
/*Timer2Intialaize*/
void timer2int(void)
{
	TCCR2A = 0b00000010;//CTCTimermode
	TCCR2B = 0b00000011;//prescaler x32
}
/*StartTimer2*/
void timer2open(void)
{// intterrupting 200usec
	OCR2A = 125;//1.6 x 125 = 200[usec]
	TIMSK2 = 0b00000010;//OCIEA on
	TCNT2 = 0x00;		//counter intilize as 0x00
	T2prescaler = 0;
}
/*ADCIntialaize*/
void adcint(void)
{
	ADMUX = 0b00100000;//ARef 8bit useADC0
	ADCSRA = 0b10001111; //ADC enable and prescaler x 128
	ADCSRB = 0b00000000; //Free running mode
}
/*ADCsataring*/
void adcopen(void)
{
	ADCSRA |= 0b01000000;
}
/*ADCclosing*/
void adcclose(void)
{
	ADCSRA &= 0b10111111;	
}
void pwm1int(void)
{
	TCCR1A = 0b10000010;
	TCCR1B = 0b00011001;
	TCCR1C = 0;
}
void pwm1open()
{
	TCCR1A |= 0b10000000;
	ICR1 = 0xffff;
	TIMSK1 = 0b00000001;
}
void pwm1close(void)
{
	TCCR1A &= 0b01111111;
	TIMSK1 = 0;
}
void pwm1Dset(unsigned int bottom ,unsigned int count)
{
	if (bottom < count)
		count = bottom - 1;
	TCNT1 = 0xffff - bottom;
	OCR1A = 0xffff - count;
	PWM_INT = 0xffff - bottom;
	PWM_DUTY= 0xffff - count;
}
/*Timer0Intialaize*/
void timer0int(void)
{
	TCCR0A = 0b00000000;//NomalTimermode
	TCCR0B = 0b00000011;//prescaler x64
}
/*StartTimer0*/
void timer0open(void)
{// intterrupting 200usec
	OCR0A = LED_PWM_duty;
	TIMSK0 = 0b00000011;//OCIEA on OCIEB on OVF on
	TCNT0 = 0x00;		//counter intilize as 0x00
}
/*start speaker*/
void SPstart(unsigned int sp_lengh , unsigned int bottom , unsigned int count)
{
	pwm1close();
	OCR0B = 125;
	SP_prescaler = sp_lengh;
	TIMSK0 |= 0b00000100;
	pwm1int();
	pwm1open();
	pwm1Dset(bottom, count);
}
/*set trigger*/
unsigned int TRGset(unsigned char mode)
{
	unsigned int trg;
	switch (mode) {
		case 0:
			trg = 100;
			break;
		case 1:
			trg = 100;
			break;
		case 2:
			trg = 100;
			break;
		case 3:
			trg = 100;
			break;
		case 4:
			trg = 100;
			break;
		case 5:
			trg = 100;
			break;
		case 6:
			trg = 100;
			break;
		case 7:
			trg = 100;
			break;
		case 8:
			trg = 100;
			break;
		case 9:
			trg = 100;
			break;
		default:
			break;
	}
	return (trg);
}
/*Timer0 interrupt*/
ISR(TIMER0_COMPA_vect)
{
	//******************************************************************
	//SREG stacking
	unsigned char stack = SREG;
	//******************************************************************
	//intterrupt function
	LightLED(0x00);
	//******************************************************************
	//SREG reversing
	SREG = stack;
  //	return(0);
	//******************************************************************
}
/*Timer0 interrupt*/
ISR(TIMER0_COMPB_vect)
{
	//******************************************************************
	//SREG stacking
	unsigned char stack = SREG;
	//******************************************************************
	//intterrupt function
	/* stop speaker*/
	if (SP_prescaler ==0) {
		OCR0B = 0x0000;
		pwm1close();
		TIMSK0 &= 0b11111011;
	}
	else {
		SP_prescaler--;
	}
  
	//******************************************************************
	//SREG reversing
	SREG = stack;
  //	return(0);
	//******************************************************************
}
/*Timer0 interrupt*/
ISR(TIMER0_OVF_vect)
{
	//******************************************************************
	//SREG stacking
	unsigned char stack = SREG;
	TCNT0 = 0x00;		//counter intilize as 0x00
                  //******************************************************************
                  //intterrupt function
	OCR0A = LED_PWM_duty;
	if (LED_PWM_duty == 0) {
		LightLED(0x00);	
	}
	else {
		LightLED(LED_var);
	}
  
	//******************************************************************
	//SREG reversing
	SREG = stack;
  //	return(0);
	//******************************************************************
}
/*Timer2 interrupt*/
ISR(TIMER2_COMPA_vect)
{
	//******************************************************************
	//SREG stacking
	unsigned char stack = SREG;
	TCNT2 = 0x00;		//counter intilize as 0x00
                  //******************************************************************
                  //intterrupt function
	if(T2prescaler > 5){
		T2prescaler = 0;
		stop = 0;
    //		LightLED(0x0f);
    
	}
	else{
		T2prescaler++;
    //		LightLED(0x00);
	}
  
	//******************************************************************
	//SREG reversing
	SREG = stack;
  //	return(0);
	//******************************************************************
}
/*ADC interrupt*/
ISR(ADC_vect)
{
	//******************************************************************
	//SREG stacking
	unsigned char stack = SREG;
	//******************************************************************
	//intterrupt function
	adcclose();
  //	LightLED(0xff);
	ADdata = ADCH;
	//******************************************************************
	//SREG reversing
	SREG = stack;
  //	return(0);
	//******************************************************************
}
/*PWM interrupt*/
ISR(TIMER1_OVF_vect)
{
	//******************************************************************
	//SREG stacking
	unsigned char stack = SREG;
	//******************************************************************
	//intterrupt function
	TCNT1 = 0x0000;
	;
	OCR1A = PWM_DUTY;		//OCR1A renewing
	TCNT1 = PWM_INT;		//counter intilize 
                      //******************************************************************
                      //SREG reversing
	SREG = stack;
  //	return(0);
	//******************************************************************
}

