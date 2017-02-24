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
    RCSTA1bits.SPEN = 1;        //Enable Serial Port Receive
    TRISCbits.TRISC7 = 1;       //Enable input
    PIE1bits.RC1IE = 1;         //Enable interrupts
    
    //Set up output
    CCP4CONbits.CCP4M = 0b1100;
    CCP5CONbits.CCP5M = 0b1100;
    CCP6CONbits.CCP6M = 0b1100; //This is multiplexed with the programmer (PGD)
    CCP7CONbits.CCP7M = 0b1100; //This is multiplexed with the programmer (PGC)
    CCP8CONbits.CCP8M = 0b1100;
    CCP9CONbits.CCP9M = 0b1100;
    //CCP10CONbits.CCP10M = 0b1100; //Disable for DMX Receive
    
    TRISCbits.TRISC1 = 0;
    //TRISCbits.TRISC7 = 0; //Disable for DMX Receive
    TRISCbits.TRISC6 = 0;
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB4 = 0;
    
    //Setup timer and PWM Frequencies
    T2CON = 0b00000100;     // Enable TMR2 with prescaler = 1
    PR2 = 249;   // PWM period = (PR2+1) * prescaler * Tcy = 1ms
    CCPR1L = 25; // pulse width = CCPR1L * prescaler * Tcy = 100us
}

void interrupt ISR() {
    if(PIR1bits.RC1IF == 1) {   //If interrupt on UEART receive
        PIR1bits.RC1IF = 0;     //Reset it to 0.
    }
}

void main(void)
{
    int DMX_ch0, DMX_ch1, DMX_ch2, DMX_ch3, 
            DMX_ch4, DMX_ch5, DMX_ch6;
    
    setup();

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
