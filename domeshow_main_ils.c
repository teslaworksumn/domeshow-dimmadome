/*
 * File:   domeshow_main.c
 * Author: Dan
 *
 * Created on January 24, 2017, 12:59 PM
 */

// PWM Duty cycle is CCPRxL (low byte) and CCPRxH (high byte)

#include <xc.h>
#include <stdint.h>
#include <pic18f47j13.h>
#define _XTAL_FREQ 32000000                  // Fosc  frequency for _delay

void setup(void)
{
    //Set up DMX Receive
    RCSTA1bits.SPEN = 1; //Enable Serial Port Receive
    TRISCbits.TRISC7 = 1; //Enable input
    PIE1bits.RC1IE = 1; //Enable interrupts
    
    CCP4CONbits.CCP4M = 0b1100;
    CCP5CONbits.CCP5M = 0b1100;
    CCP6CONbits.CCP6M = 0b1100;
    CCP7CONbits.CCP7M = 0b1100;
    CCP8CONbits.CCP8M = 0b1100;
    CCP9CONbits.CCP9M = 0b1100;
    //CCP10CONbits.CCP10M = 0b1100;
    
    TRISCbits.TRISC1 = 0;
    //TRISCbits.TRISC7 = 0;
    TRISCbits.TRISC6 = 0;
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB4 = 0;
    
    T2CON = 0b00000100;     // Enable TMR2 with prescaler = 1
    PR2 = 249;   // PWM period = (PR2+1) * prescaler * Tcy = 1ms
    CCPR1L = 25; // pulse width = CCPR1L * prescaler * Tcy = 100us
}

void writeColor(int r, int g, int b) {
    CCPR5L = 250 - ((int) ((((float) r) / 255.0) * 250.0));
    CCPR4L = 250 - ((int) ((((float) g) / 255.0) * 250.0));
    CCPR9L = 250 - ((int) ((((float) b) / 255.0) * 250.0));
}

void wheel(int WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        writeColor(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        writeColor(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    writeColor(WheelPos * 3, 255 - WheelPos * 3, 0);
    return;
}

void interrupt ISR() {
    if(PIR1bits.RC1IF == 1) {
        PIR1bits.RC1IF = 0;
    }
}

void main(void)
{
    int DMX_ch0, DMX_ch1, DMX_ch2, DMX_ch3, 
            DMX_ch4, DMX_ch5, DMX_ch6;
    int n = 0;
    
    setup();
    int i = 0;
    while(1)
    {
        
        CCPR4L = 0;
        CCPR5L = 50;
        CCPR6L = 100;
        CCPR7L = 150;
        CCPR8L = 200;
        CCPR9L = 230;
        CCPR10L = 255;
        
        __delay_ms(5);
    }
    return;
}
