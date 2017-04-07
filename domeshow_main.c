/*
 * File:   domeshow_main.c
 * Author: Dan Sunderman, Ian Smith
 *
 * Created on January 24, 2017, 12:59 PM
 * 
 * Some implementation ideas based on Phosphorus by John Bisgaard.
 * 
 */

// PIC18F47J13 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config WDTEN = OFF       // Watchdog Timer (Enabled)
#pragma config PLLDIV = 1       // 96MHz PLL Prescaler Selection (PLLSEL=0) (Divide by 3 (12 MHz oscillator input))
#pragma config CFGPLLEN = ON    // PLL Enable Configuration Bit (PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset (Enabled)
//#pragma config XINST = ON       // Extended Instruction Set (Enabled)

// CONFIG1H
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = INTOSCPLL  // Oscillator (INTOSCPLL)
#pragma config SOSCSEL = HIGH   // T1OSC/SOSC Power Selection Bits (High Power T1OSC/SOSC circuit selected)
#pragma config CLKOEC = OFF      // EC Clock Out Enable Bit  (CLKO output disabled on the RA6 pin)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor (Enabled)
#pragma config IESO = OFF        // Internal External Oscillator Switch Over Mode (Disabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Postscaler (1:32768)

// CONFIG3L
#pragma config DSWDTOSC = INTOSCREF// DSWDT Clock Select (DSWDT uses INTRC)
#pragma config RTCOSC = T1OSCREF// RTCC Clock Select (RTCC uses T1OSC/T1CKI)
#pragma config DSBOREN = ON     // Deep Sleep BOR (Enabled)
#pragma config DSWDTEN = ON     // Deep Sleep Watchdog Timer (Enabled)
#pragma config DSWDTPS = G2     // Deep Sleep Watchdog Postscaler (1:2,147,483,648 (25.7 days))

// CONFIG3H
#pragma config IOL1WAY = ON     // IOLOCK One-Way Set Enable bit (The IOLOCK bit (PPSCON<0>) can be set once)
#pragma config ADCSEL = BIT10   // ADC 10 or 12 Bit Select (10 - Bit ADC Enabled)
#pragma config PLLSEL = PLL96   // PLL Selection Bit (Selects 96MHz PLL)
#pragma config MSSP7B_EN = MSK7 // MSSP address masking (7 Bit address masking mode)

// CONFIG4L
#pragma config WPFP = PAGE_127  // Write/Erase Protect Page Start/End Location (Write Protect Program Flash Page 127)
#pragma config WPCFG = OFF      // Write/Erase Protect Configuration Region  (Configuration Words page not erase/write-protected)

// CONFIG4H
#pragma config WPDIS = OFF      // Write Protect Disable bit (WPFP<6:0>/WPEND region ignored)
#pragma config WPEND = PAGE_WPFP// Write/Erase Protect Region Select bit (valid when WPDIS = 0) (Pages WPFP<6:0> through Configuration Words erase/write protected)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <pic18f27j13.h>
#include "domeshow_lib.h"

#define TRANSMIT

#define _XTAL_FREQ 32000000     //Fosc frequency for _delay

#define BOARD_NUM 0             //Board/Chip number
#define BOARD_CHANNELS 6        //Channels per chip

volatile int board = BOARD_NUM; //Figure out a better way to set this
volatile int startChannel = board * BOARD_CHANNELS;
volatile uint8_t channelValues[255];
volatile uint16_t channel;       //Initialize channel to 0. Read from 0 -> 512

void setup(void)
{
    //Clock: Page 42 of datasheet
    
    //Set up DMX Receive
    TXSTA1bits.SYNC = 0;        //Asynchronous
    RCSTA1bits.CREN = 1;        //Enable receiver
    TXSTA1bits.TXEN = 0;        //Disable serial port send
    RCSTA1bits.SPEN = 1;        //Enable serial port receive
    RCSTA1bits.RX9 = 1;         //9-bit receive (DMX has two stop bits)
    TRISCbits.TRISC7 = 1;       //Enable input
    TRISCbits.TRISC6 = 0;
    INTCONbits.GIE = 1;         //Enable global interrupts
    INTCONbits.PEIE = 1;        //Enable peripheral interrupts
    PIE1bits.RC1IE = 1;         //Enable interrupts
    BAUDCON1bits.BRG16 = 0;     //Enable 8-bit baudrate
    SPBRG1 = 2;                 //Baud=Fosc/(64*(SPBRG1+1)) Set baudrate 250kbps
                                //What is Fosc? 48MHz
                                //Haha nobody knows what's happening here.
    
    //52
    
    /*
     * CCP4:R1
     * CCP5:G1
     * CCP6:B1
     * CCP7:R2
     * CCP8:G2
     * CCP9:B2
     */

    //Set up output
    CCP4CONbits.CCP4M = 0b1100;
    CCP5CONbits.CCP5M = 0b1100;
    CCP6CONbits.CCP6M = 0b1100; //This is multiplexed with the programmer (PGD)
    CCP7CONbits.CCP7M = 0b1100; //This is multiplexed with the programmer (PGC)
    CCP8CONbits.CCP8M = 0b1100;
    CCP9CONbits.CCP9M = 0b1100;
    //CCP10CONbits.CCP10M = 0b1100; //Disable for DMX Receive
    
    TRISCbits.TRISC1 = 0;
    //TRISCbits.TRISC7 = 1;       //Input for DMX Receive. Done above.
    TRISCbits.TRISC6 = 0;
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB6 = 0;
    TRISBbits.TRISB5 = 0;
    TRISBbits.TRISB4 = 0;
    TRISCbits.TRISC3 = 0;
    
    //Set up timer and PWM Frequencies
    //PWM Duty cycle is CCPRxL (low byte) and CCPRxH (high byte)
    T2CON = 0b00000100;         //Enable TMR2 with prescaler = 1
    //TODO: Fix these?
    PR2 = 254;                  //PWM period = (PR2+1) * prescaler * Tcy * 4
    CCPR1L = 25;                //pulse width = CCPR1L * prescaler * Tcy * 4
    
    RC3 = 0;
    
    
#ifdef TRANSMIT
    TRISCbits.TRISC6 = 0;
    TXSTA1bits.TXEN = 1;        //Enable serial port send
    
#endif
}

void interrupt RC1IFInterrupt(void) {
    char dmxByte;
    char stopByte;
    
    if(PIR1bits.RC1IF == 1) {   //If interrupt on EUART receive
        PIR1bits.RC1IF == 0;  //Reset EUART interrupt flag
        if(RCSTA1bits.FERR) {
            // Frame error; start of packet
            channel = 0;        //Start over
            return;
        }
        
        // Normal byte; channel data
        dmxByte = RCREG1;   //Read data
        stopByte = RX91;    //Read stop bit
        
        channelValues[channel] = dmxByte;  //Keep track of DMX data
        channel++;  //Then go to the next channel
        char txByte = channel % 255;
        TXREG=txByte;
    }
}

void cycle() {
    while(1) {
        int i = 0;
        for(; i < 255; i++){
            writePackedColor(Wheel(i));
            __delay_ms(5);
        }
    }
}

void write() {
    //Do the actual writing to the ports.
    //TODO: Fix scaling
    //Test
    /*channelValues[0] = 90;
    channelValues[1] = 90;
    channelValues[2] = 90;
    channelValues[3] = 90;
    channelValues[4] = 90;
    channelValues[5] = 90;
     * */
    
    /*CCPR4L = 255-i;
    CCPR5L = 255-i;
    CCPR6L = 255-i;
    CCPR7L = 255-i;
    CCPR8L = 255-i;
    CCPR9L = 255-i;*/
    
//    CCPR4L = 255-channelValues[0];      //RP7
//    CCPR5L = 255-channelValues[1];      //RP8
//    CCPR6L = 255-channelValues[2];      //RP9
//    CCPR7L = 255-channelValues[3];      //RP10
//    CCPR8L = 255-channelValues[4];      //RP12
//    CCPR9L = 255-channelValues[5];      //RP17
    
    CCPR4L = channelValues[startChannel+0];      //RP7
    CCPR5L = channelValues[startChannel+1];      //RP8
    CCPR6L = channelValues[startChannel+2];      //RP9
    CCPR7L = channelValues[startChannel+3];      //RP10
    CCPR8L = channelValues[startChannel+4];      //RP12
    CCPR9L = channelValues[startChannel+5];      //RP17
}

void main(void)
{
    setup();
    //Start with everything off
    /*CCPR4L = 255;
    CCPR5L = 255;
    CCPR6L = 255;
    CCPR7L = 255;
    CCPR8L = 255;
    CCPR9L = 255;*/
    
    while(1)
    {
        write();
//        i = (i+1)%256;
//        write(i);
//        
//        __delay_ms(5);
        //TXREG = 'a';
        //RC3 = 1;
        //cycle();
        
//        CCPR4L -= 5;
//        CCPR5L = 50;
//        CCPR6L = 100;
//        CCPR7L = 150;
//        CCPR8L = 200;
//        CCPR9L = 230;
//        write();
        
//        __delay_ms(5);
    }
    return;
}
