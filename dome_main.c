/*
 * File:   dome_main.c
 * Author: Ian
 *
 * Created on April 6, 2017, 9:09 PM
 */

// PIC18F47J13 Configuration Bit Settings

// 'C' source line config statements -- THIS CONFIG WORKS

// CONFIG1L
#pragma config WDTEN = ON       // Watchdog Timer (Enabled)
#pragma config PLLDIV = 2       // THIS IS CORRECT DON'T CHANGE (2)
#pragma config CFGPLLEN = OFF    // PLL Enable Configuration Bit (PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset (Enabled)
//#pragma config XINST = ON       // Not supported in free XC8

// CONFIG1H
#pragma config CP0 = OFF        // Code Protect (Program memory is not code-protected)

// CONFIG2L
#pragma config OSC = INTOSCPLLO  // Oscillator (INTOSCPLL)
#pragma config SOSCSEL = HIGH   // T1OSC/SOSC Power Selection Bits (High Power T1OSC/SOSC circuit selected)
#pragma config CLKOEC = ON      // EC Clock Out Enable Bit  (CLKO output enabled on the RA6 pin)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor (Enabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

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


#include "xc.h"
#include <stdint.h>
#include <pic18f27j13.h>
#include "domeshow_lib.h"
#include "dmxconfig.h"
#include "dmx.h"

#define _XTAL_FREQ 48000000     //Fosc frequency for _delay

#define CHIP_ADDRESS 0
#define CHIP_CHANNELS 6
//#define BAUDPIN_OVERRIDE

uint8_t channelValues[CHIP_CHANNELS];
extern volatile uint8_t DMX_RxData[DMX_RX_BUFFER_SIZE];


void setup(void) {
    timer_init();

//    Currently not overriding UART and pin setup. These get covered(?) in dmx_init.
#define BAUDPIN_OVERRIDE
    pin_init();
    uart_init();
    
    dmx_init();
    dmx_set_start(0x00);         //Check this with ENTTEC USB DMX Pro Library
    dmx_set_address(0x01);
    
    TXSTA1bits.TXEN = 1;
    
    return;
}

void write_UART(char * s, int length)
{
    int i;
    for(i=0; i<length; i++)
    {
        TXREG=s[i];
    }
}

void interrupt RC1IFInterrupt() {
//    dmx_timer_interrupt();     //Don't think we need this, but leaving it in in case
    dmx_rx_interrupt();
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
    //Not sure what's up with the ordering here...
    CCPR4L = DMX_RxData[3];      //RP7
    CCPR5L = DMX_RxData[1];      //RP8
    CCPR6L = DMX_RxData[2];      //RP9
    CCPR7L = DMX_RxData[0];      //RP10
    CCPR8L = DMX_RxData[4];      //RP12
    CCPR9L = DMX_RxData[5];      //RP17
}

int main(void) {
    setup();
    
    while(1)
     {
        //if(dmx_new_data())
        //{
        //    dmx_read(CHIP_ADDRESS,channelValues,CHIP_CHANNELS); // Read the data into channelValues
            write();
        //}

//      Can add a routine here to turn lights off after a specific time with no signal, etc.     

     }
    
    return 0;
}
