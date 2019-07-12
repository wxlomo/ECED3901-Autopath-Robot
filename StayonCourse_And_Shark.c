/*
 * ECED3901 Design Project PID and LiDAR obstacle detection
 * Created: 2019-07-07 8:48:47 AM
 * Author : Alexa Manderville
 */ 
//Definitions
#define F_CPU 14745600UL
#define AVcc 3.3 //Vref

//Standard Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

//---------------------------------------------------------------------------------------
//Definitions and Includes
//---------------------------------------------------------------------------------------
#define F_CPU 			14745600UL	//Crystal Clock freq.
#define Encoder_Notches 24u			//Custom encoders
#define tire_dia 		65u			//DF robot wheels in millimeters
#define max_RPM 		200.0		//DF robot limitation at 6V
#define min_RPM			10.0		//Robot limit based on encoders sense rate
#define timer1_period 	4.34027777778e-6f	//Prescaler of 64 used.

//PID related
#define K_P     		300			//Proportional gain, kp 300
#define K_I     		30			//Integral gain, ki 30
#define K_D     		100			//Differential gain, kd 100
#define TIME_INTERVAL   5			//PID time interval, Timer2 overflow ~18ms, TI 5
									//PID recalculates every 5x18ms ~90ms
#define MAX_PID_ERROR	200			//Max proportional error per update, 200RPM error 
#define MAX_PIDSUM_ERROR 2000		//Max PID integral error per update, 2000
									//Here we use 10 x Max_PID_Error, or 10x PID cycles
									//Bounds to ~1 sec of error build up.
#define offset_PWM		15			//% of duty cycle before motors start to move, 30
									//non-linear startup, OPENLOOP 0, PID 30
//Standard Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

//Custom Includes
#include "pid/uart.h"				//Header includes setup for UART
#include "pid/pid.h"				//AVR221 PID code, written by Atmega

//**************LiDAR includes********************//
//Custom Includes
#include "vl53l1x/uart.h"				//Header includes setup for UART
#include "vl53l1x/twimaster.h"
#include "vl53l1x/vl53l1_api.h"			//LiDAR API Header

//Setup LIDAR Device
static VL53L1_Dev_t                   dev;
static VL53L1_DEV                     Dev = &dev;
static int status;
static VL53L1_RangingMeasurementData_t RangingData;

//---------------------------------------------------------------------------------------
//Global Variables
//---------------------------------------------------------------------------------------
//Timer 1 functional variables
static unsigned int PINC_prev = 0x00;			//previous PINC values; 
												//used to detect rising/falling edges
//Encoder variables for tracking ticks
static unsigned long left_encoder_count = 0;	//Tick counter for left 
static unsigned long right_encoder_count = 0;	//Tick counter for right

//Timer 1 variables for tracking angular velocity
static unsigned long left_timer_counter = 0;	//Timer1 counts since last encoder trip
static unsigned long right_timer_counter = 0;	//Previous count for delta time measure
static float temp_counts = 0.0;					//Used for calcs.

//Measured Angular Velocity (RPM)
//Arrays used to get rid of outliers when measuring Angular Velocity (RPM) 
static float l_RPM[5] = {};				
static float r_RPM[5] = {};				
static float sort_RPM[5] = {};
static float left_RPM = 0.0;					
static float right_RPM = 0.0;						

//Desired Angular Velocity (RPM)
static float left_RPM_setpoint = 0.0;
static float right_RPM_setpoint = 0.0;
//static float temp = 0.0;

//PID Variables
int16_t ref_Left, measure_Left, inputValue_Left;
int16_t ref_Right, measure_Right, inputValue_Right;
struct PID_DATA pidDataLeft;		
struct PID_DATA pidDataRight;		

	
//---------------------------------------------------------------------------------------
//Prototype definitions
//---------------------------------------------------------------------------------------
ISR(PCINT2_vect);
ISR(TIMER1_OVF_vect);
ISR(TIMER2_OVF_vect);
int cmpfunc (const void * a, const void * b);
void Set_Input(int16_t inputValueL, int16_t inputValueR);
void go_forward(float velocity);

void adc_init(void)
{
	ADMUX = (1<<REFS0); 	//Use AVcc with external cap
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);  //Enable ADC, scaler
}
void IR1_adc_set_channel(unsigned int read_ADC_channel)
{
	read_ADC_channel &= 0x07;                   	//Valid input chn. 0-7
	ADMUX = (ADMUX & 0b11111000)|read_ADC_channel;  //Select desired channel
	ADCSRA |= (1<<ADSC);                        	//ADC Ctl & Status Reg. A
	while(ADCSRA & (1<<ADSC));                  	//Waits for conversion/read
}
void IR2_adc_set_channel(unsigned int read_ADC_channel)
{
	read_ADC_channel &= 0x07;                   	//Valid input chn. 0-7
	ADMUX = (ADMUX & 0b11111000)|read_ADC_channel;  //Select desired channel
	ADCSRA |= (1<<ADSC);                        	//ADC Ctl & Status Reg. A
	while(ADCSRA & (1<<ADSC));                  	//Waits for conversion/read
}
void Metal1_adc_set_channel(unsigned int read_ADC_channel)
{
	read_ADC_channel &= 0x07;                   	//Valid input chn. 0-7
	ADMUX = (ADMUX & 0b11111000)|read_ADC_channel;  //Select desired channel
	ADCSRA |= (1<<ADSC);                        	//ADC Ctl & Status Reg. A
	while(ADCSRA & (1<<ADSC));                  	//Waits for conversion/read
}
void Metal2_adc_set_channel(unsigned int read_ADC_channel)
{
	read_ADC_channel &= 0x07;                   	//Valid input chn. 0-7
	ADMUX = (ADMUX & 0b11111000)|read_ADC_channel;  //Select desired channel
	ADCSRA |= (1<<ADSC);                        	//ADC Ctl & Status Reg. A
	while(ADCSRA & (1<<ADSC));                  	//Waits for conversion/read
}
//LiDAR setup
void setup_lidar()
{
	uint8_t byteData;
	uint16_t wordData;
	
	i2c_init();
	
	Dev->I2cDevAddr = 0x52;
	
	VL53L1_software_reset(Dev);
	VL53L1_RdByte(Dev, 0x010F, &byteData);
	printf("VL53L1X Model_ID: %X\n", byteData);
	VL53L1_RdByte(Dev, 0x0110, &byteData);
	printf("VL53L1X Module_Type: %X\n", byteData);
	VL53L1_RdWord(Dev, 0x010F, &wordData);
	printf("VL53L1X: %X\n",wordData);
	printf("Autonomous Ranging Test\n");
	status = VL53L1_WaitDeviceBooted(Dev);
	status = VL53L1_DataInit(Dev);
	status = VL53L1_StaticInit(Dev);
	status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
	status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
	status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 50);
	VL53L1_UserRoi_t roiConfig;
	roiConfig.TopLeftX  = 1;
	roiConfig.TopLeftY  = 15;
	roiConfig.BotRightX = 15;
	roiConfig.BotRightY = 1;
	status = VL53L1_SetUserROI(Dev, &roiConfig);
	status = VL53L1_StartMeasurement(Dev);
	if(status)
	{
		printf("Status: %i\n",status);
		printf("VL53L1_StartMeasurement failed... Halting program execution.\n");
		while(1);	//... and kill progression
	} 

}
//---------------------------------------------------------------------------------------
//Interrupt service routines
//---------------------------------------------------------------------------------------

// Pin Change Interrupt ----------------------- ENCODER TICK AND ANGULAR VELOCITY UPDATES
ISR(PCINT2_vect)
{	
	int C2_change = (PINC & (1<<PC2)) ^ (PINC_prev & (1<<PC2));	//Did C2 change
	int C3_change = (PINC & (1<<PC3)) ^ (PINC_prev & (1<<PC3));	//and/or did C3 Change
			
	//use rising-edge only on C2 for left encoder
	if ((C2_change != 0x00) && (PINC & (1<<PC2)) )
	{
		left_encoder_count++;							//Increment Ticks 
		
		//Velocity Measurement
		temp_counts = (float)TCNT1+(float)left_timer_counter;//Period since last	
		//Calculate RPM from Timer1 value since last;  (Rotation/Notch) / Time in min
		left_RPM=(1.0/(float)Encoder_Notches)/(temp_counts*timer1_period/60);		
		sort_RPM[0] = l_RPM[0] = l_RPM[1];
		sort_RPM[1] = l_RPM[1] = l_RPM[2];
		sort_RPM[2] = l_RPM[2] = l_RPM[3];
		sort_RPM[3] = l_RPM[3] = l_RPM[4];
		sort_RPM[4] = l_RPM[4] = left_RPM;
		qsort(sort_RPM, 5, sizeof(float), cmpfunc);
		left_RPM = sort_RPM[2];	
		left_timer_counter = 0;							//Update prev. value
			
		right_timer_counter += TCNT1;
		//Reset Timer 1
		TCNT1 = 0;
	}
		
	//use rising-edge only on C3 for right encoder
	if ((C3_change != 0x00) && (PINC & (1<<PC3)) )
	{
		right_encoder_count++;							//Increment Ticks 
		
		//Velocity Measurement
		temp_counts = (float)TCNT1+(float)right_timer_counter;//Period since last		
		//Calculate RPM from Timer1 value since last;  (Rotation/Notch) / Time in min
		right_RPM=(1.0/(float)Encoder_Notches)/(temp_counts*timer1_period/60);		
		sort_RPM[0] = r_RPM[0] = r_RPM[1];
		sort_RPM[1] = r_RPM[1] = r_RPM[2];
		sort_RPM[2] = r_RPM[2] = r_RPM[3];
		sort_RPM[3] = r_RPM[3] = r_RPM[4];
		sort_RPM[4] = r_RPM[4] = right_RPM;
		qsort(sort_RPM, 5, sizeof(float), cmpfunc);
		right_RPM = sort_RPM[2];
		right_timer_counter = 0;						//Update prev. value	
		
		left_timer_counter += TCNT1;	
		//Reset Timer 1
		TCNT1 = 0;	
	}
	
	//Hard limit.  Force update RPM values if counters get out of hand, i.e. < min_RPM
	if (left_timer_counter > 65535)
	{		
		left_RPM = (1.0/(float)Encoder_Notches)/(left_timer_counter*timer1_period/60);
		l_RPM[0]=l_RPM[1]=l_RPM[2]=l_RPM[3]=l_RPM[4] = left_RPM;
	}
	if (right_timer_counter > 65535) 
	{
		right_RPM=(1.0/(float)Encoder_Notches)/(right_timer_counter*timer1_period/60);	
		r_RPM[0]=r_RPM[1]=r_RPM[2]=r_RPM[3]=r_RPM[4] = right_RPM;
	}
		
	//Update PINC to current reading
	PINC_prev = PINC;
}


//Timer1 Overflow Interrupt ---------------------- TIMER FOR ANGULAR VELOCITY MEASUREMENT
ISR(TIMER1_OVF_vect)
{		
	//Update RPM measurement on overflow...	been long time since last tick.. both wheels!
	//If no tick in 0.284 seconds; relatively no movement (<5 RPM) or critical failure!	
	left_timer_counter += 65535;
	right_timer_counter += 65535;
	left_RPM = (1.0/(float)Encoder_Notches)/(left_timer_counter*timer1_period/60);
	l_RPM[0]=l_RPM[1]=l_RPM[2]=l_RPM[3]=l_RPM[4] = left_RPM;
	right_RPM=(1.0/(float)Encoder_Notches)/(right_timer_counter*timer1_period/60);
	r_RPM[0]=r_RPM[1]=r_RPM[2]=r_RPM[3]=r_RPM[4] = right_RPM;
}


//Timer2 Overflow Interrupt -------------------------------- PID UPDATE CALCULATION TIMER
ISR(TIMER2_OVF_vect)
{		
	static uint16_t i = 0;		
	// Keep building time intervals until time to perform PID calc.
	if(i < TIME_INTERVAL)  {	i++;  }		
	// PID update time!!
	else
	{		
		i = 0; 					
		ref_Left = left_RPM_setpoint;
		ref_Right = right_RPM_setpoint;			
		measure_Left = left_RPM;
		measure_Right = right_RPM;
		if (ref_Left > min_RPM)
		{
			inputValue_Left = pid_Controller(ref_Left, measure_Left, &pidDataLeft);
		}
		else
		{
			inputValue_Left = -32767;			//This is zero for PID controller 
			pidDataLeft.sumError = 0;			//Prevent integral windup
		}
		if (ref_Right > min_RPM)
		{
			inputValue_Right = pid_Controller(ref_Right, measure_Right, &pidDataRight);
		}
		else
		{
			inputValue_Right = -32767;			//This is zero for PID controller 
			pidDataRight.sumError = 0;			//Prevent integral windup
		}		
		
		//inputValue_Left = ref_Left * (65535/200) - 32768;//uncomment for OPENLOOP
		//inputValue_Right = ref_Right * (65535/200) - 32768;//uncomment for OPENLOOP
		//Send new process value... PWM duty cycle
		Set_Input(inputValue_Left,inputValue_Right);		
	}
}


//---------------------------------------------------------------------------------------
//Function implementations 
//---------------------------------------------------------------------------------------
//cmpfunc used for sorting arrays
int cmpfunc (const void * a, const void * b)
{
	float fa = *(const float*) a;
	float fb = *(const float*) b;
	return (fa>fb) - (fa<fb);
}
	
//Set_Input function is used to send PWM to motors for vehicle control
void Set_Input(int16_t inputValueL, int16_t inputValueR)
{
	float tempL = 0.0; 
	float tempR = 0.0;
	
	//Need to translate inputValue from PID to duty-cycle on PWM; 
	//Negative is reverse, positive is forward normally, 
	//but we don't have quadrature-encoders (two optical channels/wheel)
	// 0 = motor off (no duty cycle and 32768  = motors full on (100% duty cycle)
	
	//Shift from negative to only positive output from PID
	tempL = (float)inputValueL+32768.0;
	tempR = (float)inputValueR+32768.0;
	
	//Magnitude scale, plus add offset, as non-linear control
	tempL = tempL/65535.0*100.0 + offset_PWM;		
	tempR = tempR/65535.0*100.0 + offset_PWM;

	//Limit magnitudes to ranges actually possible
	if (tempL > 100) {tempL=100.0;} else if (tempL<0.4) {tempL=0.4;}
	if (tempR > 100) {tempR=100.0;} else if (tempR<0.4) {tempR=0.4;}		
	
	//Send magnitude to TIMER0/PWM comparators for commanding the motors
	OCR0A = tempL/100.0*256.0-1;
	OCR0B = tempR/100.0*256.0-1;	
}

//Simple forward routine that trusts velocity PID
void go_forward(float velocity)
{
	//Vehicle Speed 65mm dia tire; 204.2mm/rotation; at 200RPM, top speed is ~680mm/sec
	//@25% duty cycle - ~1 RPS, @100 d.c.% - ~3 RPS, range of approx 60 - 180 RPM
	
	//Set direction forward, may need to adjust depending on team wiring/etc.
	PORTB |= (1 << PB6);
	PORTB &= ~(1 << PB5);
	
	//Set RPM
	left_RPM_setpoint = velocity;
	right_RPM_setpoint = velocity;
}

enum {LEFT,RIGHT};

int Lidar_Distance(){
	int temp_Lidar=7;
	status = VL53L1_WaitMeasurementDataReady(Dev);
	
	if(!status)
	{
		status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
		temp_Lidar = status;
		if(status==0)
		{
			#ifdef DEUBUG
			printf("Range: %i mm, ", RangingData.RangeMilliMeter);
			printf("Status: %u, ", RangingData.RangeStatus);
			printf("Signal Rate: %3.2lf, ", RangingData.SignalRateRtnMegaCps/65536.0);
			printf("Ambient Rate: %3.2lf\n",RangingData.AmbientRateRtnMegaCps/65336.0);
			#endif
		}
		status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
	}
	#ifdef DEBUG
	else{printf("Error waiting for data ready: %i\n",status);}
	#endif
	return RangingData.RangeStatus;
}

void Rotate_on_Spot(int Ticks,char Dir){
	int L_start=left_encoder_count;
	int R_start=right_encoder_count;
	if (Dir == RIGHT){
		PORTB &= ~(1 << PB5);
		PORTB &= ~(1 << PB6);
		right_RPM_setpoint=left_RPM_setpoint=12;
		}else{
		PORTB |= (1 << PB6);
		PORTB |= (1 << PB5);
		right_RPM_setpoint=left_RPM_setpoint=12;
	}
	
	while ((left_encoder_count-L_start)<Ticks && (right_encoder_count-R_start)<Ticks){
		#ifdef DEBUG
		printf("L_Ticks: %lu, R_Ticks: %lu, ",left_encoder_count,right_encoder_count);
		printf("L_set: %3.0f, R_set: %3.0f, ",left_RPM_setpoint,right_RPM_setpoint);
		printf("L_RPM: %3.2f, R_RPM: %3.2f\n",left_RPM,right_RPM);
		#endif
	}

	right_RPM_setpoint=left_RPM_setpoint=0;
}

enum{Start,Escape,Stop};

void Rotate_on_Wheel(int Ticks,char Dir){
	int L_start=left_encoder_count;
	int R_start=right_encoder_count;
	if (Dir == RIGHT){
		right_RPM_setpoint=0;
		left_RPM_setpoint=18;
		}else{
		right_RPM_setpoint=18;
		left_RPM_setpoint=0;
	}
	
	while ((left_encoder_count-L_start)<Ticks && (right_encoder_count-R_start)<Ticks){
		#ifdef DEBUG
		printf("L_Ticks: %lu, R_Ticks: %lu, ",left_encoder_count,right_encoder_count);
		printf("L_set: %3.0f, R_set: %3.0f, ",left_RPM_setpoint,right_RPM_setpoint);
		printf("L_RPM: %3.2f, R_RPM: %3.2f\n",left_RPM,right_RPM);
		#endif
	}

	right_RPM_setpoint=left_RPM_setpoint=0;
}

//---------------------------------------------------------------------------------------
//Main Program 
//---------------------------------------------------------------------------------------

int main(void)
{
	
	char State;
	//interrupt initializations 
	cli();								//Disable Interrupts
	init_uart();						//Initialize UART w/interrupts (baud/etc)
	setup_lidar();	
	//***********************LiDAR STUFF*************//
	PCICR = (1<<PCIE2);					//Pin Change Interrupt Control Reg.
										//PC2 is PCINT18 & PC3 is PCINT19, on PCIE2
	PCMSK2 = (1<<PCINT18)|(1<<PCINT19);	//Pin Change Mask Reg.
										//PCINT18 and PCINT19 to trigger interrupt
	TIMSK1 = (1 << TOIE1) ;				//Enable timer1 overflow interrupt(TOIE1)
	TIMSK2 = (1 << TOIE2);				//Enable timer2 overflow interrupt(TOIE2)
	
	sei();								//Enable Interrupts	
	
	//Timer0 setup, PWM to left and right motors
	DDRB |= (1<<PB3) | (1<<PB4);		//Data Dir. Reg. Port B, B3 B4 Out, PWM L R
	DDRB |= (1<<PB5) | (1<<PB6);		//Data Dir. Reg. Port B, B5 B6 Out, DIR L R
	TCNT0 = 0x00;						//Timer0, start at zero
	TCCR0A = 0xA3;						//Timer0 Ctrl Reg. A, non-invrt chan. A & B
										//and Mode 3: FastPWM + MAX limit
	TCCR0B = 0x02;						//Timer0 clk source, 8 prescaler & Mode 3
	OCR0A = 0;							//Timer0 Output Compare Register A, 8-bit
	OCR0B = 0;							//Timer0 Output Compare Register B, 8-bit
		
	//Timer1 setup; encoder sensor for position & angular velocity
	DDRC &= ~((1<<PC2)|(1<<PC3));		//Data Dir. Reg. PortC, C2 C3 In, Enc L R
	TCNT1 = 0x00;						//Timer1, start at zero
	TCCR1A = 0x00;						//Timer1 Control Reg. A, normal mode
	TCCR1B = ((1<<CS10)|(1<<CS11));		//Timer1 Control Reg. B, prescale:64, 
										//overflow ~0.284s, 16-bit counter	
	
	//Timer2 setup; PID update interval
	TCNT2 = 0x00;						//Timer2 start at zero
	TCCR2A = 0x00;						//Timer2 Control Reg. A, normal mode
	TCCR2B = ((1<<CS20)|(1<<CS21)|(1<<CS22));//Timer2 Control Reg. B, prescale 1024,
										//Overflow ~18ms, 14.4kHz, 8-bit counter (255)  		
		
	//PID initializations	
	pid_Init(K_P*SCALING_FACTOR, K_I*SCALING_FACTOR, K_D*SCALING_FACTOR, &pidDataLeft);
	pid_Init(K_P*SCALING_FACTOR, K_I*SCALING_FACTOR, K_D*SCALING_FACTOR, &pidDataRight);	
	pidDataLeft.maxError = pidDataRight.maxError = MAX_PID_ERROR;
	pidDataLeft.maxSumError = pidDataRight.maxSumError = MAX_PIDSUM_ERROR;	
	l_RPM[0]=l_RPM[1]=l_RPM[2]=l_RPM[3]=l_RPM[4] = 0;
	r_RPM[0]=r_RPM[1]=r_RPM[2]=r_RPM[3]=r_RPM[4] = 0;
		
	//Forward for this robot		
	PORTB |= (1 << PB6);
	PORTB &= ~(1 << PB5);		
					
	//Characterization variables
	left_RPM_setpoint=right_RPM_setpoint=40;//set RPM to 40
	
	//Metal and ADC
	int result; // variable to store the ADCW register value
	adc_init(); // initialize the registers for ADC conversions
	//IR variable d
	float IR_Left = 3901.0;
	float IR_Right = 3901.0; //for IR1 and IR2
	
	//Escape Algorithm variables
		State = Start;
		int temp_lidar;
		int Stop_Count =0;
		int Stop_Distance = 350;
		
	while (1)
	{	
				status = VL53L1_WaitMeasurementDataReady(Dev);
				if(!status)
				{
					status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
					if(status==0)
					{
						//printf("Range: %i mm, ", RangingData.RangeMilliMeter);
						//printf("Status: %u, ", RangingData.RangeStatus);
						//printf("Signal Rate: %3.2lf, ", RangingData.SignalRateRtnMegaCps/65536.0);
						//printf("Ambient Rate: %3.2lf\n",RangingData.AmbientRateRtnMegaCps/65336.0);
					}
					status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
				}
		
		//switch(State){
			//case Start:
			//for (int i=0;i<10;i++){
				//temp_lidar=Lidar_Distance();
			//}
			//if (RangingData.RangeMilliMeter<1050 && temp_lidar==0){ // originally 350
				//State = Escape;
				//}else{
				//Rotate_on_Spot(5,LEFT);
			//}
			//break;
			//
			//
			//case Escape:
			//Lidar_Distance();
			//if (RangingData.RangeMilliMeter<Stop_Distance && temp_lidar==0){
				//switch (Stop_Count){
					//case 0:
					//Rotate_on_Spot(13,LEFT);
					//Stop_Distance = 800;
					//Stop_Count++;
					//break;
					//case 1:
					//Rotate_on_Spot(10,RIGHT);
					//Stop_Count++;
					//Stop_Distance = 250;
					//
					//break;
					//default:
					//break;
				//}
				//
				//
				//}else {
				//go_forward(30);
			//}
			//break;
			//case Stop:
			//left_RPM_setpoint=right_RPM_setpoint=0;
			//break;
			//
		//}
		if (RangingData.RangeMilliMeter<250){
			right_RPM_setpoint=left_RPM_setpoint=0;
		}else{
				go_forward(30);
		}
	
		//*IR*//
		IR1_adc_set_channel(0); // choose the ADC channel 0
		IR_Left = ADC*AVcc/1023.0; // calculate the value of the converted voltage
		IR2_adc_set_channel(1); // choose the ADC channel 1
		IR_Right = ADC*AVcc/1023.0; // calculate the value of the converted voltage
		if(IR_Right > 0.2 && IR_Left > 0.2) {
			//printf("V: %3.2fV, Not at the edge\n", d1);
			left_RPM_setpoint=right_RPM_setpoint=0;//set RPM to 40
			while (1)
			{
			}

		}else if (IR_Right>0.1){
			Rotate_on_Spot(4,LEFT);
		}else if (IR_Left>0.1){
			Rotate_on_Spot(4,RIGHT);
		}
		//_delay_ms(500);
		
		//////*METAL*//
		//Metal1_adc_set_channel(7); // choose the ADC channel 7
		//result = (int) ADCW;
		//float Coil_Left = result*AVcc/1023.0; // convert the ADCW value to a voltage Metal1
		//Metal2_adc_set_channel(6); // choose the ADC channel 6
		//result = (int) ADCW;
		//float Coil_Right = result*AVcc/1023.0; //Metal2
		//
		//if (Coil_Right > 3){
			//Rotate_on_Wheel(4,RIGHT);
		//}else if (Coil_Left > 3){
			//Rotate_on_Wheel(4,LEFT);
		//}
		//
		
		//if(Coil_Right > 3 || Coil_Left >3 ) {
			//printf("Converted V: %3.2fV, Metal detected\n", Coil_Left);
			//left_RPM_setpoint=right_RPM_setpoint=0;//set RPM to 40
//
			//}else{
			//printf("Converted V: %3.2fV, Metal not detected\n", Coil_Left);
		//}
		//_delay_ms(1000); //one second delay to keep the terminal clean
		
		//*PID*//
		//Otherwise the robot continues to speak it's mind
		//else
		//{		
			//printf("L_Ticks: %lu, R_Ticks: %lu, ",left_encoder_count,right_encoder_count);
			//printf("L_set: %3.0f, R_set: %3.0f, ",left_RPM_setpoint,right_RPM_setpoint);		
			//printf("L_RPM: %3.2f, R_RPM: %3.2f\n",left_RPM,right_RPM);			
		//}	
		

		//else{printf("Error waiting for data ready: %i\n",status);}
	//	if (RangingData.RangeMilliMeter<100){
		//	left_RPM_setpoint=right_RPM_setpoint=0;//set RPM to 40
		//}
		//_delay_ms(50);
	}
}