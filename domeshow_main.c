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

#pragma config XINST = OFF      //Allows use of MPLab simulator for debugging

#define _XTAL_FREQ 32000000     //Fosc frequency for _delay

#define BOARD_NUM 0             //Board/Chip number
#define BOARD_CHANNELS 6        //Channels per chip

volatile int board = BOARD_NUM; //Figure out a better way to set this
volatile int channel = 0;       //Initialize channel to 0. Read from 0 -> 512
volatile int localChannel = 0;  //Keep track of which channel on the board
volatile char dmxByte;          //This will hold the raw DMX read
volatile uint8_t channelValues[BOARD_CHANNELS];

void delay() {
    asm("repeat #11999");
    asm("nop");
}


void setup(void)
{
    //Clock: Page 42 of datasheet
    
    //Set up DMX Receive
    TXSTA1bits.TXEN = 0;        //Disable serial port send
    RCSTA1bits.SPEN = 1;        //Enable serial port receive
    TRISCbits.TRISC7 = 1;       //Enable input
    PIE1bits.RC1IE = 1;         //Enable interrupts
    BAUDCON1bits.BRG16 = 1;     //Enable 16-bit baudrate
    SPBRG1 = 1;                 //Baud=Fosc/(16*(SPBRG1+1)) Set baudrate 250kbps
                                //What is Fosc? 8MHz?
    
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
    
    //Set up timer and PWM Frequencies
    //PWM Duty cycle is CCPRxL (low byte) and CCPRxH (high byte)
    T2CON = 0b00000100;         //Enable TMR2 with prescaler = 1
    //TODO: Fix these?
    PR2 = 254;                  //PWM period = (PR2+1) * prescaler * Tcy * 4
    CCPR1L = 25;                //pulse width = CCPR1L * prescaler * Tcy * 4
}

//Copied the attribute from Kia
//Previously just had interrupt
void __attribute__((__interrupt__)) ISR() {
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
        if((channel >= (BOARD_CHANNELS * board)) && (localChannel < BOARD_CHANNELS)) {
            channelValues[localChannel] = dmxByte;  //Keep track of DMX data
            localChannel++;                         //The go to the next channel
        }
    }
}

void write() {
    //Do the actual writing to the ports.
    //TODO: Fix scaling
    
    //Test
    channelValues[0] = 90;
    channelValues[1] = 90;
    channelValues[2] = 90;
    channelValues[3] = 90;
    channelValues[4] = 90;
    channelValues[5] = 90;
    
    CCPR4L = channelValues[0];      //RP7
    CCPR5L = channelValues[1];      //RP8
    CCPR6L = channelValues[2];      //RP9
    CCPR7L = channelValues[3];      //RP10
    CCPR8L = channelValues[4];      //RP12
    CCPR9L = channelValues[5];      //RP17
}


void writeColor(int r, int g, int b) {
    CCPR5L = 255 - r;
    CCPR4L = 255 - g;
    CCPR10L = 255 -b;
}

void drawFrame(int frame) {
    int r = frame;
    int b = 255 - frame;
    int g = 0;
    writeColor(r, g, b);
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

void cycle() {
    while(1) {
        int i = 0;
        for(; i < 255; i++){
            writePackedColor(Wheel(i));
            __delay_ms(5);
        }
    }
}







void main(void)
{
    
    setup();

    //Start with everything off
    CCPR4L = 255;
    CCPR5L = 255;
    CCPR6L = 255;
    CCPR7L = 255;
    CCPR8L = 255;
    CCPR9L = 255;
    
    int i = 0;
    
    while(1)
    {
        
        cycle();
        
//        CCPR4L -= 5;
//        CCPR5L = 50;
//        CCPR6L = 100;
//        CCPR7L = 150;
//        CCPR8L = 200;
//        CCPR9L = 230;
//        write();
        
        __delay_ms(5);
    }
    return;
}
