//
// MyWalletReader code
//
// This is the starter code for the WalletReader device using a PIC16F1823 microprocessor
//
#define ZX_CLK_8MHZ		/* define for 8MHz, commented out is 16MHz */

#include <xc.h>

// procressor configuration bits
#pragma config FOSC = INTOSC	// internal oscillator
#pragma config PLLEN = OFF	// 4x PLL disabled
#pragma config CLKOUTEN = OFF	// no clock output
#pragma config FCMEN = ON	// enable fail-safe clock monitoring
#pragma config IESO = OFF	// disable oscillator switchover
#pragma config WDTE = OFF	// enable watchdog timer
#pragma config PWRTE = ON	// enable power up timer
#pragma config BOREN = OFF	// allow brown out detection // set OFF from ON
#pragma config BORV = LO	// brown out voltage trip point
#pragma config MCLRE = ON	// reset on MCLR pin not RE3
#pragma config STVREN = ON	// stack error will cause a reset
#pragma config LVP = OFF	// disable low voltage programming
#pragma config CP = ON		// enable code protection
#pragma config CPD = ON		// enable data protection
#pragma config WRT = ALL	// enable write protection of entire device

//#define Clock_8MHz
//#define Baud_9600
//#pragma config = 0x29A4
//#pragma config reg2 = 0x3EFF
//#pragma origin 4
//#include "math16.h"
//#include "math24f.h"

#define _XTAL_FREQ  8000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions

#define PAY         PORTAbits.RA5  //PIN 3		(input)
#define COLLECT     PORTAbits.RA4  //PIN 5		(input)
#define LED         PORTCbits.RC5  //PIN 9		(digital output used for LED)
#define SWITCH      PORTCbits.RC0  //PIN 10		(digital output used for Relay)
#define LEFT_RF     PORTAbits.RA2  //PIN 2		(input) used to detect RF ON
#define LEFT_Code   PORTCbits.RC2  //PIN 6		(output) used to be A.4
#define RIGHT_Code  PORTCbits.RC3  //PIN 8		(output) used for group transmission signals
#define RIGHT_RF    PORTCbits.RC1  //PIN 11		(input) use to detect RF ON

unsigned int Nsample,Rsample,Lsample;
unsigned int State, Nevent, Hevent, Revent, Levent;
//unsigned int RandomNumber;

void boot_up(void);
void uart_init(void);
unsigned int Read_ADC_Value(void);
//void configure_transmitter(void);
void transmit_PNcode(void);
void transmit_MASTERcode(void);
void transmit_code(void);
void LED_Flash(unsigned int);
void send_packet(void);
void switch_action_ALL(void);
void switch_action_IR(void);
void switch_action_CAP(void);
//void send_packet_PROX(void);
void Change_State(void);
void Left_RF_Module(void);
void Right_RF_Module(void);
void Oscillator_HIGH(void);
void Oscillator_LOW(void);
void RandomDelay(void);

void main()
{
   unsigned int j;
   unsigned int SUM, MSL, AnalogValue; //val=280;
   
   boot_up();
//   uart_init();
      
   for(j = 0; j < 10; j++)			//startup flashing LED
    {
        LED = 1;
        __delay_ms(100);
        LED = 0;
        __delay_ms(100);
    }
   
    __delay_ms(1000);         //force a delay of 1 sec (for power to get high))

   State = 0;   // initialize as the ground state

   while(1)
   {
       switch (State)
        {
            case 0:
//                transmit_code();		// transmit code
                switch_action_ALL();	// perform switch action (for ALL functions IR/CAP/TOUCH)
                break;
                
            case 1:
//                transmit_code();		// transmit code
                switch_action_IR();		// perform switch action (IR only MASTER MODE)
                break;

            case 2:
//                transmit_code();		// transmit code
                switch_action_CAP();	// perform switch action (for CAP/TOUCH SLAVE MODE)
                break;
        }
        Change_State();             // change state as required with a HOVER detected
        __delay_ms(20);             // delay 4 msec (40)
//        RandomDelay();
   } 
}

void LED_Flash(unsigned int x)
{
    unsigned int j;
    
	for(j = 0; j < x; j++)			//flash x times
    {
        LED = 1;
        __delay_ms(200);
        LED = 0;
        __delay_ms(200);
    }
}

void uart_init(void)
{
	TXSTA = 0;
	RCSTA = 0;
	BAUDCON = 0;

	RCSTAbits.SPEN = 1;		/* enable serial port */

	/*
	 * configure for 115200N81.
	 * We use a 16-bit BRG with BRGH=1, so the baudrate is (Fosc/baudrate/4) - 1.
	 * at 8MHz, the baudrate error is 2.12%. At 16MHz it is 0.79%.
	 */
	TXSTAbits.BRGH = 1;
	BAUDCONbits.BRG16 = 1;

#ifdef ZX_CLK_8MHZ
	SPBRGH = 0;
//	SPBRGL = 16;
	SPBRGL = 207;       // for 9600 baud, default for BLE module //
#else
	SPBRGH = 0;
	SPBRGL = 34;
#endif

	TXSTAbits.TXEN = 1;		/* enable transmitter */
}

void boot_up(void)
{
//    OSCCON = 0b11110000; //Setup internal oscillator for 32MHz (with 4x PLL)
    OSCCON = 0b01110000; //Setup internal oscillator for 32MHz (4X PLL removed for 8 mhz)
//    while(OSCCON.2 == 0); //Wait for frequency to stabilize
//
    ANSELA = 0;
    ANSELC = 0;
    
	LATA = 0b00000000;
    LATC = 0b00000000;
    
    PORTA = 0b00000000;
    PORTC = 0b00010100;
    TRISA = 0b00111110;  //0 = Output, 1 = Input (RX is an input)
    TRISC = 0b00100000;  //0 = Output, 1 = Input (RX is an input)
    
//	APFCON = 0b.1110.0110;	//Set alternate Pin function register (page 113)
	APFCON = 0b10000100;	//Set alternate Pin function register (page 113)
//	MCLRE = 0;
//    WPUA = 0b00010000;
//    WPUC = 0b00100000;

	LED = 0;    // on
	SWITCH = 0; // off
    LEFT_Code = 1;   // set high in the OFF state
    RIGHT_Code = 1;  // set high in the OFF state
    LEFT_RF = 0;
    RIGHT_RF = 0;
    Nsample = 0;
    Rsample = 0;
    Lsample = 0;
//    Hsample = 0;
    Nevent = 0;      // LED switch event
    Revent = 0;      // RF right switch event
    Levent = 0;      // RF left switch event
    Hevent = 0;      // hover switch event

    ADRESH = 0b00000000; //Turn pins to Digital instead of Analog
    ADRESL = 0b00000000; //Turn pins to Digital instead of Analog
//    ANSEL = 0b.0000.0000; //Turn pins to Digital instead of Analog
//    ANSELH = 0b.0000.0000; //Turn high pins to Digital instead of Analog
//    CMCON = 0b.0000.0111; //Turn off comparator on RA port
    INTCON = 0b00000000; //Turn off interrupts
    T1CON = 0b00000000; //Turn off timers
    T2CON = 0b00000000; //Turn off timers
    SSP1CON1 = 0b00000000; //Turn off slave select
    SSP1CON3 = 0b00000000; //Turn off other SSP1 stuff
    SSP1STAT = 0b00000000; //Turn off SSP1 status register
    CCP1CON = 0b00000000; //Turn off CCP (Compare/Capture/PWM), and turn on PWM
//    CCP1CON = 0b.0000.0000; //Turn off CCP (Compare/Capture/PWM), and turn on PWM

//clear RA2 (CCP1) for output and PWM1 on pin 5
    TRISCbits.TRISC2 = 0;	//clear RA2 (CCP1) for output and PWM1 on pin 5
    TRISCbits.TRISC4 = 0;	//clear RA2 (CCP1) for output and PWM1 on pin 5
    
// 2MHz
//    T2CON = 0b00000100; // prescaler + turn on TMR2;
//    PR2 = 0b00000011;
//    CCPR1L = 0b00000010;  // set duty MSB
//    CCP1CON = 0b00001100; // duty lowest bits + PWM mode 
    
// 4MHz
    T2CON = 0b00000100; // prescaler + turn on TMR2;
    PR2 = 0b00000001;
    CCPR1L = 0b00000001;  // set duty MSB
    CCP1CON = 0b00001100; // duty lowest bits + PWM mode
    
// 1MHz  (based on 8MHZ clock)
//    T2CON = 0b00000100; // prescaler + turn on TMR2;
//    PR2 = 0b00000001;
//    CCPR1L = 0b00000001;  // set duty MSB
//    CCP1CON = 0b00001100; // duty lowest bits + PWM mode
    
    STR1B = 0;   // Start the PWM steering as off for LEFT_RF
    STR1D = 0;   // Start the PWM steering as off for RIGHT_RF
	FVRCON = 0b00000000;
}

void switch_action_ALL()
{
// detect the trigger
//
    if (PAY == 0 || COLLECT == 0)         // was 1100 was 2400 (recently use 1800) 
	{
            Nsample++;
            Nevent = 0;
	}
	else
	{
//            Nsample = 0;
            Nevent = 1;
	}
//
// Detect an RIGHT RF pulse
//
    if (RIGHT_RF == 1)
	{
            Rsample++;
            Revent = 0;
	}
	else
	{
            Revent = 1;
	}
//
// Detect an LEFT RF pulse
//
    if (LEFT_RF == 1)
	{
            Lsample++;
            Levent = 0;
	}
	else
	{
            Levent = 1;
	}

        if (Nsample >= 3 && Nevent == 1)
        {
            if (LED == 1)
            {
                SWITCH = 0;
                NOP();
                LED = 0;
//                while(TXIF == 0) ; TXREG = G_HOVER_ON;
            }
            else
            {
                SWITCH = 1;
                NOP();
                LED = 1;
//                while(TXIF == 0) ; TXREG = G_HOVER_OFF;
            }
            Nsample = 0;
            Nevent = 0;
        }

        if (Nsample >= 180)
        {
            Hevent = 1;     // hover event detected
            Nsample = 0;    // count reset
            return;
        }
//
// RIGHT RF pulse detected and switch 
//    
        if (Rsample >= 1 && Revent == 1)
        {
            if (LED == 1)
            {
                SWITCH = 0;
                NOP();
                LED = 0;
                __delay_ms(10);
            }
            else
            {
                SWITCH = 1;
                NOP();
                LED = 1;
                __delay_ms(10);
            }
            Rsample = 0;
            Revent = 0;
//
// Send the LEFT RF pulse for 100 msec on condition of a MASTER switch event
//
            Left_RF_Module();
        }
//
// RF pulse detected and switch 
//    
        if (Lsample >= 1 && Levent == 1)
        {
            if (LED == 1)
            {
                SWITCH = 0;
                NOP();
                LED = 0;
                __delay_ms(10);
            }
            else
            {
                SWITCH = 1;
                NOP();
                LED = 1;
                __delay_ms(10);
            }
            Lsample = 0;
            Levent = 0;
//
// Send the RF pulse for 100 msec on condition of a MASTER switch event
//
            Right_RF_Module();
        }
}

void switch_action_IR()
{
//
// detect the trigger
//
    if (COLLECT == 0)
	{
            Nsample++;
            Nevent = 0; 
	}
	else
	{
//            Nsample = 0;
            Nevent = 1;
	}                          

        if (Nsample >= 3 && Nevent == 1)
        {
            if (LED == 1)
            {
                SWITCH = 0;
                NOP();
                LED = 0;
                __delay_ms(10);
            }
            else
            {
                SWITCH = 1;
                NOP();
                LED = 1;
                __delay_ms(10);
            }
            Nsample = 0;
            Nevent = 0;
//
// Send the RF pulse for 100 msec, to both sides staggered
//
            Left_RF_Module();
            Right_RF_Module();
        }

        if (Nsample >= 180)
        {
            Hevent = 1;     // hover event detected
            Nsample = 0;    // count reset
            return;
        }
}

void switch_action_CAP()
{

// sum up PROX and touch events

	if (COLLECT == 0)
	{
            Nsample++;
            Nevent = 0;
	}
	else
	{
//            Nsample = 0;
            Nevent = 1;
	}
//
// Detect an RIGHT RF pulse
//
    if (RIGHT_RF == 1)
	{
            Rsample++;
            Revent = 0;
	}
	else
	{
            Revent = 1;
	}
//
// Detect an LEFT RF pulse
//
    if (LEFT_RF == 1)
	{
            Lsample++;
            Levent = 0;
	}
	else
	{
            Levent = 1;
	}

        if (Nsample >= 3 && Nevent == 1)
//        if (Nsample >= 10)
        {
            if (LED == 1)
            {
                SWITCH = 0;
                NOP();
                LED = 0;
                __delay_ms(10);
//                while(TXIF == 0) ; TXREG = G_HOVER_ON;
            }
            else
            {
                SWITCH = 1;
                NOP();
                LED = 1;
                __delay_ms(10);
//                while(TXIF == 0) ; TXREG = G_HOVER_OFF;
            }
            Nsample = 0;
            Nevent = 0;
        }

        if (Nsample >= 180)
        {
            Hevent = 1;     // hover event detected
            Nsample = 0;    // count reset
            return;
        }
//
// RIGHT RF pulse detected and switch 
//    
        if (Rsample >= 1 && Revent == 1)
        {
            if (LED == 1)
            {
                SWITCH = 0;
                NOP();
                LED = 0;
                __delay_ms(10);
            }
            else
            {
                SWITCH = 1;
                NOP();
                LED = 1;
                __delay_ms(10);
            }
            Rsample = 0;
            Revent = 0;
//
// Send the LEFT RF pulse for 100 msec on condition of a MASTER switch event
//
            Left_RF_Module();
        }
//
// RF pulse detected and switch 
//    
        if (Lsample >= 1 && Levent == 1)
        {
            if (LED == 1)
            {
                SWITCH = 0;
                NOP();
                LED = 0;
                __delay_ms(10);
            }
            else
            {
                SWITCH = 1;
                NOP();
                LED = 1;
                __delay_ms(10);
            }
            Lsample = 0;
            Levent = 0;
//
// Send the RF pulse for 100 msec on condition of a MASTER switch event
//
            Right_RF_Module();
        }
}

void Change_State(void)
{
   if (Hevent==1)       // change state based on hover event
	{
      State++;
      if (State>2) State = 0;
      LED_Flash(State+1);
      Hevent = 0;       // reset hover back to switch mode
	}
//   else
//   {
//   }
}

void Left_RF_Module(void)
{
    LEFT_Code = 0;
    NOP();
    Oscillator_HIGH();NOP();
    STR1B = 1;
    NOP();
    __delay_ms(400);    // 100 msec delay
    NOP();
    STR1B = 0;
    NOP();
    Oscillator_LOW();
    NOP();
    LEFT_Code = 1;
    NOP();
}

void Right_RF_Module(void)
{
    RIGHT_Code = 0;     // Send to the opposite RF antenna
    NOP();
    Oscillator_HIGH();  //Select internal oscillator for 32MHz (with 4x PLL)
    NOP();
    STR1D = 1;
    NOP();
    __delay_ms(400);    // 100 msec delay (400 for a 32MHz ticker))
    NOP();
    STR1D = 0;
    NOP();
    Oscillator_LOW();   //Return internal oscillator to 8MHz (with 4x PLL off)
    NOP();
    RIGHT_Code = 1;
    NOP();
}

void Oscillator_HIGH(void)
{
    OSCCON = 0b11110000; //Select internal oscillator for 32MHz (with 4x PLL)
}

void Oscillator_LOW(void)
{
    OSCCON = 0b01110000; //Return internal oscillator to 8MHz (with 4x PLL off)
}