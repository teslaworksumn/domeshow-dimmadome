/* 
 * File: domeshow_lib.h   
 * Author: Ian Smith
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef DOME_LIB
#define	DOME_LIB

#include <xc.h> // include processor files - each processor file is guarded.  

void uart_init()
{
    //Set up DMX Receive
    TXSTA1bits.SYNC = 0;        //Asynchronous
    RCSTA1bits.CREN = 1;        //Enable receiver
    TXSTA1bits.TXEN = 0;        //Disable serial port send
    RCSTA1bits.SPEN = 1;        //Enable serial port receive
    RCSTA1bits.RX9 = 0;         //8-bit receive (DMX has two stop bits)
    TRISCbits.TRISC7 = 1;       //Enable input
    TRISCbits.TRISC6 = 0;
    INTCONbits.GIE = 1;         //Enable global interrupts
    INTCONbits.PEIE = 1;        //Enable peripheral interrupts
    PIE1bits.RC1IE = 1;         //Enable interrupts
    BAUDCON1bits.BRG16 = 0;     //Enable 8-bit baudrate
    TXSTA1bits.BRGH = 0;
    SPBRG1 = 2;                 //Baud=Fosc/(64*(SPBRG1+1)) Set baudrate 250kbps (2)
}

void pin_init()
{
    TRISCbits.TRISC1 = 0;
    //TRISCbits.TRISC7 = 1;       //Input for DMX Receive. Done in uart_init.
    TRISCbits.TRISC6 = 0;
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB4 = 0;
    TRISCbits.TRISC3 = 0;
    
    RC3 = 0;                        //This for troubleshooting?
}

void timer_init()
{
    //set up clock speed (pls get us 48MHz)
    OSCCONbits.IRCF = 0b111;    //8Mhz internal oscillator
    OSCCONbits.SCS = 0b00;      //System clock is primary clock source
    OSCTUNEbits.PLLEN = 1;      //Enable PLL
    
    //Set up output
    CCP4CONbits.CCP4M = 0b1100;
    CCP5CONbits.CCP5M = 0b1100;
    CCP6CONbits.CCP6M = 0b1100; //This is multiplexed with the programmer (PGD)
    CCP7CONbits.CCP7M = 0b1100; //This is multiplexed with the programmer (PGC)
    CCP8CONbits.CCP8M = 0b1100;
    CCP9CONbits.CCP9M = 0b1100;
    //CCP10CONbits.CCP10M = 0b1100; //Disable for DMX Receive
    
    //Set up timer and PWM Frequencies
    //PWM Duty cycle is CCPRxL (low byte) and CCPRxH (high byte)
    T2CON = 0b00000100;         //Enable TMR2 with prescaler = 1
    //TODO: Fix these?
    PR2 = 254;                  //PWM period = (PR2+1) * prescaler * Tcy * 4
    CCPR1L = 25;                //pulse width = CCPR1L * prescaler * Tcy * 4
}


void writeColor(int r, int g, int b) {
    CCPR4L = 255 - r;
    CCPR5L = 255 - g;
    CCPR6L = 255 - b;
    CCPR7L = 255 - r;
    CCPR8L = 255 - g;
    CCPR9L = 255 - b;
}

uint32_t packColor(int r, int g, int b) {
	return (((long) r << 16) | ((long) g << 8) | ((long) b));
}

int getR(uint32_t x) {
	return (int) (x >> 16);
}

int getG(uint32_t x) {
	return (int) (x >> 8);
}

int getB(uint32_t x) {
	return (int) (x);
}

void writePackedColor(uint32_t x) {
	writeColor(getR(x), getG(x), getB(x));
}

uint32_t Wheel(int WheelPos) {
	WheelPos = 255 - WheelPos;
	if(WheelPos < 85) {
		return packColor(255 - WheelPos * 3, 0, WheelPos * 3);
	}
	if(WheelPos < 170) {
		WheelPos -= 85;
		return packColor(0, WheelPos * 3, 255 - WheelPos * 3);
	}
	WheelPos -= 170;
	return packColor(WheelPos * 3, 255 - WheelPos * 3, 0);
}



#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

