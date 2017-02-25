/*
 * File:   domeshow_main.c
 * Author: Dan Sunderman, Ian Smith
 *
 * Created on January 24, 2017, 12:59 PM
 * 
 * Some implementation ideas based on Phosphorus by John Bisgaard.
 * 
 */

#include <xc.h>
#include <stdint.h>
#include <pic18f47j13.h>

#define _XTAL_FREQ 32000000     //Fosc frequency for _delay

#define BOARD_NUM 0
#define BOARD_CHANNELS 6

volatile int board = BOARD_NUM; //Figure out a better way to set this
volatile int channel = 0;       //Initialize channel to 0. Read from 0 -> 512
volatile int localChannel = 0;  //Keep track of which channel on the board
volatile char dmxByte;          //This will hold the raw DMX read
volatile int channelValues[BOARD_CHANNELS];

void setup(void)
{
    //Set up DMX Receive
    RCSTA1bits.SPEN = 1;        //Enable Serial Port Receive
    TRISCbits.TRISC7 = 1;       //Enable input
    PIE1bits.RC1IE = 1;         //Enable interrupts
    BAUDCON1bits.BRG16 = 1;     //Enable 16-bit baudrate
    SPBRG1 = 3;                 //250k=16000000/(16*(3+1)) Set baudrate 250kHz
    
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
    
    //Set up timer and PWM Frequencies
    //PWM Duty cycle is CCPRxL (low byte) and CCPRxH (high byte)
    T2CON = 0b00000100;         //Enable TMR2 with prescaler = 1
    PR2 = 254;                  //PWM period = (PR2+1) * prescaler * Tcy = 1ms
    CCPR1L = 25;                //pulse width = CCPR1L * prescaler * Tcy = 100us
}

void interrupt ISR() {
    if(PIR1bits.RC1IF == 1) {   //If interrupt on UEART receive
        PIR1bits.RC1IF = 0;     //Reset it to 0.
        if(RCSTA1bits.FERR) {
            channel = 0;        //If end of signal
            localChannel = 0;   //Keep track of which channel in the board
            dmxByte = RCREG1;   //Start reading again
        }
        else {
            channel++;          //Next channel
            dmxByte = RCREG1;   //Read data
        }
        //If we're in the right range of values for this board
        if((channel >= (16*board)) && (localChannel < BOARD_NUM)) {
            channelValues[localChannel] = dmxByte;  //Keep track of DMX data
            localChannel++;                         //The go to the next channel
        }
    }
}

void write() {
    //Do the actual writing to the ports.
    //Since
    CCPR4H = channelValues[0];
    CCPR5H = channelValues[1];
    CCPR6H = channelValues[2];
    CCPR7H = channelValues[3];
    CCPR8H = channelValues[4];
    CCPR9H = channelValues[5];
}

void main(void)
{
    
    setup();

    //Start everything as off
    CCPR4H = 0;
    CCPR5H = 0;
    CCPR6H = 0;
    CCPR7H = 0;
    CCPR8H = 0;
    CCPR9H = 0;
    
    while(1)
    {
        
        CCPR4L = 0;
        CCPR5L = 50;
        CCPR6L = 100;
        CCPR7L = 150;
        CCPR8L = 200;
        CCPR9L = 230;
        
        __delay_ms(5);
    }
    return;
}
