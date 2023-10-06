// Program Author: Kenneth Mallabo
// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <iso646.h>
#include <stdint.h>
#include <pic16f877a.h>

#define _XTAL_FREQ  19660800        //THIS SHOULD BE CHANGED TO 19660800 Hz for the lab boards, 20000000 for proteus 

#include<pic.h>

// lcd initialise
#define RS RB4
#define EN RB5
#define D4 RB0
#define D5 RB1
#define D6 RB2
#define D7 RB3

#include "lcd.h";

// pwm define
#define START_PULSE_TIMER() T0CS=0 //echo wait PULSEIN timer start bit 
#define STOP_PULSE_TIMER() T0CS=1 //echo wait PULSEIN timer stop bit         
#define PORT_CONFIG() TRISC=0;TRISD=0 
#define INTERRUPT_ENABLE() GIE=1; PEIE=1; T0IF=0; T0IE=1 

//pwm variables
double PulseInTime=0;        //variable calculating the echo travel time 
double cmDistance;        //variable calculating the distance within limits 
unsigned int tDistance;        //temperary variable for holding the distance value 
unsigned int TimeCnt=0;        //timer0 overflow count 
char DArr[8];            // array holding the  calculated distance for display 

// lcd variables
int j;
int i;
int a;

long distance; // variable where the the reflection time of the ultrasound is stored

__CONFIG( FOSC_HS & WDTE_OFF & PWRTE_OFF & CP_OFF & BOREN_ON & LVP_OFF & CPD_OFF & WRT_OFF & DEBUG_OFF);

unsigned char counter,clk_cnt=0,clk_sec=0,seconds=0,up_date=0;
void setup_hardware (void)
{
        
   
    PORTA=0x00; 		// switch & ultrasonic	
     
    TRISB = 0x00 ;		// set all of PORTB for output   

    //TRISBbits.TRISB0=1;     	// But make least significant bit an input for the interrupt input if using that 
    TRISA=0xFE;			// set PORTA for inputs  originally FE 
     
    //TRISC0=TRISC1=TRISC2=0;
    ADCON1 =0xFE;      		// enable portA as analogue
	
    INTCONbits.GIE=1;           //Enable the interrupt system
    INTCONbits.INTE=1;          // 
    OPTION_REGbits.INTEDG=0;    // 0 or 1 depending on what 'edge' of signal at RBo 
    INTCONbits.PEIE=1;          // The peripheral register enable bit that applies to all 'other' peripherals
                                // that can not be processed through INTCON directly such as TIMER1 and use PIE/PIR
    
    //TIMER 0 SET_UP Comment out if using timer 1
    //INTCONbits.TMR0IE=0;
    //INTCONbits.TMR0IF=0;
    //INTCONbits.PEIE=1;			//not required for the basic TO but T1 would require this
    
    //TIMER 1 SET_UP Comment out if using timer 0
    PIE1bits.TMR1IE=1;
    PIR1bits.TMR1IF=0;
    T1CON=0B00000000;
    T1CONbits.T1CKPS0=1;        // THE BITS FOR PRESCLAER SET TO 11    
    T1CONbits.T1CKPS1=1;        // which  is    8
    T1CONbits.TMR1CS=0;         // THIS BIT ENSURES TIMER CLOCK IS SYSTEMS Fosc/4 

    OPTION_REGbits.PS0=1;		//option register set 
    OPTION_REGbits.PS1=1;
    OPTION_REGbits.PS2=1;
    OPTION_REGbits.PSA=0;
    OPTION_REGbits.T0CS=0;
    
 
	    //the following lines relate to the external interrupt if it was being used
    //OPTION_REGbits.INTEDG=0;  // The interrupting signal is caused by a 0 to 1 transition (edge triggered)
                                 // so the INT EDGE select bit is set to 1. A '0' in this option bit would allow a 1 to 0 EDGE 
                                 // to cause the interrupt. 
   //INTCONbits.INTF=0;      // Clear the external interrupt flag as a precaution
   //INTCONbits.INTE=1;      // set the ENABLE bit to activate external interrupt
        
   //INTCONbits.GIE=0		// I have disabled interrupts
   //;       			// THE GLOBAL INTERRUPT ENABLE/MASK BIT MUST BE SET to allow  any CPU interrupt
   
                                // NOW BEFORE 'TURNING ON' TIMER 1 MODULE TO START COUNTING...
                                // LOAD A VALUE TO START COUNT FROM (NOT JUST  0000 TO FFFF).
                                // ASSUMING THAT Fosc= 20MHz which is divided by 4 and pre-scaler 8....
                                // (4/20e6)*8 = 1.6 microseconds i.e the period of clock signals applied to TIMER 1.    
    //T1CONbits.TMR1ON;         // So for 1 second this would mean 1sec/1e-6 = 625,000 counts..obviously multiple overflows
                                // of a 16 bit count (max single overflow of 65,536). So either use a lower frequncy XTAL...
                                // or as before, with TIMER 0 programs, 'count off' a number of interrupts. Clearly you require
                                // an integer value (no fractional/decimal) and must remember the max value of a single count is 65,536.)
                                // dividing 625,000 by 10 gives 62,500 which is an integer value less than 65,536.
      TMR1L=0xDC;               // SO THAT MEANS YOU LOAD  -62,500 (2'S COMPLEMENT) WHICH IS hex  0BDC.
      TMR1H=0x0B;       
      //T1CONbits.TMR1ON=1;         //And now turn on TIMER 1 TO START COUNT
      
      //**************************************************************************
        //***************THE FOLLOWING IS FOR THE TIMER 1 GATE CONTROL.....there are a lot of registers that need to be carefully checked
      //T1CONbits.T1GINV=1;       //This is in the T1 CONTROL REG and if '1' then TIMER 1 COUNTS WHEN INPUT AT GATE =1
      
      //T1CONbits.TMR1GE=1;       // This 'gate enables'  i.e. the timer count is controlled by the gate at pin PORT B 5
      
      //Also, comparator C2 control register has a bit which must be set to 1 for the T1 Gate  input to be from the pin (T1G)  
      //CM2CON1bits.T1GSS=1;
      
      //**************************************************************************************************************  
}


// Interrupt function  Remember there is only 1 function invoked by an 'interrupt' even though there are various possible sources
// of interrupt. This example has only 1 used, the timer 0 (often it and external interrupt are used)
//The following line is a generic name for ISR routine
//void __interrupt() interruptServiceRoutine(void) 
//{			
 //          }
	
 //But below it is named for the timer since we are only using that  
// Interrupt function
//	NOTE:Comment out the interrupt function if using timer 0, otherwise don't if using timer 1

void __interrupt()timer_isr(void) 
{
   if(PIR1bits.TMR1IF==1)     // THIS IF CLAUSE IS TRUE IF THE INTERRUPT CAME FROM TIMER 1
   {
      TMR1L=0xDC;               //Reset the timer for the next count
      TMR1H=0x0B;
      PIR1bits.TMR1IF=0;        //Reset the flag in the PIR register
      
      clk_cnt++;
    
        if(clk_cnt==7)   // originally 20?-->   // Check for the 7th interrupt? NOTE!!!!
        {                           // Yes it was set to 20 to 'slow down' the demo to 2 seconds for observation purposes
            clk_cnt=0;              //Reset
            clk_sec++;              // count off another second
            up_date=1;              // message flag to main routine 
        }
  } 
                                // The simple purpose of the external Interrupt is to reset SECONDS to zero
    else                        // If it was not the TIMER that interrupted
        {   INTCONbits.INTF=0;        // It must be EXTERNAL INTERRUPT SO CLEAR THAT FLAG 
            T1CONbits.TMR1ON=0;      // Turn off timer 1 for a moment...possibly could be done first
      
            clk_sec=0;                // Clear seconds
            clk_cnt=0;                // and reset this to ensure it starts from a true zero condition 
            up_date=1;              // message flag to main routine as we want this 'zero' to be displayed
      
            TMR1L=0xDC;               //Reset the timer for the next count
            TMR1H=0x0B;
            T1CONbits.TMR1ON=1;       //And turn on the TIMER 1 again to continue
    }                           // You could just have cleared the TIMER flag and cleared seconds...but the output will probably
}   				// never show zero or only very momentarily...this is a 'full reset' on interrupt switch press


/*******************************************************************************************
  Function    : Delay10Us
  Description : Delay function for 10 micro-sec 
*******************************************************************************************/  
void Delay10Us()
{
    int dCnt;
    for(dCnt=0;dCnt<3;dCnt++);    
}
/*******************************************************************************************
  Function    : Delay_100MicroSec
  Description :Delay function for 100 micro-sec  
*******************************************************************************************/ 
void Delay_100MicroSec()
{
    int dCnt;             // delay count   
    for(dCnt=1;dCnt<35;dCnt++);     // 100Us approx   
}
 
/*******************************************************************************************
  Function    : DelayMs
  Description :  Delay function for n milli-sec, n=1,2,3.... 
*******************************************************************************************/  
void DelayMs(unsigned char Cnt)
{
    int dCnt;
    while(Cnt>0)
    {
        for(dCnt=0;dCnt<10;dCnt++)
        Delay_100MicroSec();
        Cnt--;
    }    
}
/*******************************************************************************************
  Function    : TIMER0_INIT
  Description :Timer0 initialization
*******************************************************************************************/
void TIMER0_INIT()
{
    PSA =1;            // set timer0 minimum prescale   
    TMR0=0;            // timer0 count register   
    T0CS=1;         // timer0 start bit   
}
/*******************************************************************************************
  Function    : TriggerPulse
  Description :Generating pulse for sensor
*******************************************************************************************/
void TriggerPulse()
{
    RA0=0;
    TRISA0=0;        // pulse signal line made O/P for trigger   
    Delay10Us();
    RA0=1;            // H_to_L transition. ie, 
                //Trigger Pulse to the Ultrasonic range finder module  
    Delay10Us();
    RA0=0;
    TimeCnt=0;         // clear timer overflow count
    PulseInTime=0; 
    TMR0=0; 
}    
/*******************************************************************************************
  Function    : PulseInMode
  Description :Calculating & displaying distance
*******************************************************************************************/
void PulseInMode()
{    
    while(!RA2);        // wait for PulseIn signal to go HIGH   
    START_PULSE_TIMER();    // start echo PulseIn timer T0CS=0  
    while(RA2);         // wait for PulseIn signal to go LOW   
    STOP_PULSE_TIMER();    // stop echo PulseIn timer T0CS=1  
    PulseInTime= TMR0;    // timer calculations [return time in Us]   
    PulseInTime= (double)( ( (PulseInTime* 0.2) + (TimeCnt * 51.2)) );    
    cmDistance = (double) PulseInTime/58;      // timer value in Us
                                        // divided by 58 gives the cm distance  
    tDistance =(unsigned int)cmDistance;    // display of the calculated distance       
    // lcd header
   Lcd_Clear();
   char str[20];
   
   Lcd_Clear();
   sprintf(str, "Distance = %f", cmDistance);
   Lcd_Set_Cursor(1,1);
   Lcd_Write_String(str);
 
}

/*******************************************************************************************
  Function    : interrupt isr
  Description :interrupt service   
  NOTE: Comment this function out if using timer 1!
*******************************************************************************************/
/*
void interrupt isr()
{    
 
    if(T0IF==1)        // timer overflow interrupt   
    {    
        T0IF=0;        // clear timer interrupt flag   
        TimeCnt++;    // timer overflow count increment   
    }
}
*/
void main(void) 

{
   setup_hardware() ;	//initialise the various registers used	
   seconds  = 0;   
   Lcd_Init();		    // lcd header
   PORT_CONFIG();           // PORT configurations   
   TIMER0_INIT();           // Timer initialisation   
   INTERRUPT_ENABLE();         // Interrupt enable       
   
   Lcd_Clear();
   char str[20];
   Lcd_Clear();
   sprintf(str, "Distance = %f", cmDistance);
   Lcd_Set_Cursor(1,1);
   Lcd_Write_String(str);
   
   while(1)	
   {
	/****pulse width modulation technique source: https://www.rhydolabz.com/wiki/?p=895
	****comment or uncomment code for either technique***/
	
	//while(RA1==0)	// comment this loop out if using real hardware
	//{
	    // do nothing while the switch is off
	//}
        TriggerPulse();       // Trigger mode                        
        PulseInMode();       // PulseIn mode   
        DelayMs(250);       //wait before next cycle   
        DelayMs(250);      //wait before next cycle
		 
      while(RA1==1); 	// comment this loop out if using real hardware
   
      tDistance = 0;
    }
    
      /***timer 1 technique & delay timing technique
      ****comment or uncomment code for either technique***/
      /*			
      clk_sec=0; 	
      distance = 0;
      Lcd_Clear();
      char str[80];
      sprintf(str, "Distance = %d", clk_sec);
      Lcd_Set_Cursor(1,1);
      Lcd_Write_String(str);

      //while(RA1==0)	// comment this loop out if using real hardware
      //{
	 // do nothing while swich is off
      //}  //WAIT FOR SWITCH PRESS
      RA0=1;			//PULSE TRIGGER FOR 10 US
      __delay_us(10); 
      RA0=0;      		// TURN OFF PULSE TRIGGER
      
      while(RA2==0);		//WAIT FOR RA2 TO GO HIGH
      while(RA2==1)      
      {
	    TMR1IF=1;		// turn on counter for timer1
      }
      TMR1IF=0;			// turn off counter for timer1      
      Lcd_Clear();
      sprintf(str, "Distance = %d", clk_sec);
      Lcd_Set_Cursor(1,1);
      Lcd_Write_String(str);
      __delay_us(10000);  
		 
      //while(RA1==1) 	// comment this loop out if using real hardware
      //{
	    // do nothing else while swich is on
      //}    
      
     }
    */ 
 }
