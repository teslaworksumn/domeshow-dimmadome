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
#include "crc16_xmodem.h"
//#include "dmxconfig.h"
//#include "dmx.h"

#define _XTAL_FREQ 48000000     //Fosc frequency for _delay

#define CHIP_ADDRESS 0
#define CHIP_CHANNELS 6
#define TOTAL_CHANNELS 120
#define RX_BUFFER_SIZE 0x02ff // 1024
//#define BAUDPIN_OVERRIDE

// State of the domeshow RX
typedef enum {
    DSCOM_STATE_READY,
    DSCOM_STATE_PRE_PROCESSING,
    DSCOM_STATE_PROCESSING
} DSCOM_RX_STATE;

uint8_t channelValues[TOTAL_CHANNELS];
DSCOM_RX_STATE dscom_rx_state;
volatile uint8_t rxData[RX_BUFFER_SIZE];
uint16_t head = 0;
volatile uint16_t tail = 0;
unsigned char crc_start = 0;
unsigned char crc_end = 0;
uint8_t num_magic_found = 0;
uint8_t magic[4] = {0xDE, 0xAD, 0xBE, 0xEF};

void setup(void) {
    
    timer_init();
    pin_init();
    uart_init();
        
    TXSTA1bits.TXEN = 1;
    return;
}

void interrupt isr() {
    uint8_t rxByte;

    if(RC1IE & RC1IF)
    {
        RC1IF=0; // Clear interrupt flag

        // Check for framing error
        if(RCSTA1bits.FERR)
        {
            toggle();
            TXREG1 = 0x97;
            rxByte = RCREG1; // Clear framing error
        } else {
            rxByte = RCREG1;
            rxData[tail] = rxByte;
            tail = (tail + 1) & RX_BUFFER_SIZE;
        }
    }
}

uint16_t get_tail() {
    RC1IE = 0; // Disable interrupts to read value
    uint16_t t = tail;
    RC1IE = 1; // Re-enable interrupts
    return t;
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
    CCPR4L = channelValues[3];      //RP7
    CCPR5L = channelValues[1];      //RP8
    CCPR6L = channelValues[2];      //RP9
    CCPR7L = channelValues[0];      //RP10
    CCPR8L = channelValues[4];      //RP12
    CCPR9L = channelValues[5];      //RP17
}

/*
 * Returns the number of bytes in the ring buffer that can be used currently.
 * Intentionally off by one to account for race conditions with interrupts
 * (giving a one byte buffer between head and tail)
 */
int bytes_available() {
    int t = get_tail();
    int available = t - head;
    if (t < head) { // Looped around ring buffer
        available = RX_BUFFER_SIZE - head + t;
    }
    return available;
}

/*
 * Reads in the next byte from the ring buffer and properly increments head.
 * Assumes that there is at least one byte to read
 */
uint8_t read_byte() {
    uint8_t byte = rxData[head];
    head = (head + 1) & RX_BUFFER_SIZE;
    return byte;
}

/*
 * Reads the next two bytes from the ring buffer as a uint16.
 * Assumes that there are at least two bytes to read
 */
uint16_t read_two_bytes() {
    uint16_t highByte = rxData[head] << 8;
    head = (head + 1) & RX_BUFFER_SIZE;
    uint16_t lowByte = rxData[head];
    head = (head + 1) & RX_BUFFER_SIZE;
    return highByte | lowByte;
}

void read_packet(uint16_t length) {
    unsigned int i = 0;
    while (i < length) {
        channelValues[i] = rxData[head];
        head = (head + 1) & RX_BUFFER_SIZE;
        i++;
    }
}

int main(void) {

    setup();
    
    uint8_t rxByte;
    uint16_t length;
    int num_bytes;
    
    while(1)
    {
        __delay_ms(5);
        switch (dscom_rx_state) {
            case DSCOM_STATE_READY:
                // Wait for magic bytes
                num_bytes = bytes_available();
                if (num_bytes > 0) {
                    RC3=1;
                    rxByte = read_byte();
                    if (rxByte == magic[num_magic_found]) {
                        num_magic_found++;
                        // If all magic found, move on
                        if (num_magic_found == 4) {
                            dscom_rx_state = DSCOM_STATE_PRE_PROCESSING;
                            num_magic_found = 0;
                        }
                    } else {
                        // Not a magic sequence. Start over
                        num_magic_found = 0;
                    }
                }
                break;
            case DSCOM_STATE_PRE_PROCESSING:
                toggle();
                // Decode length (two bytes)
                if (bytes_available() >= 2) {
                    length = read_two_bytes();
                    // Check for invalid length goes here
                    dscom_rx_state = DSCOM_STATE_PROCESSING;
                }
                break;
            case DSCOM_STATE_PROCESSING:
                // Decode packet (most of the data)
                if (bytes_available() >= length) {
                    // Load data into "ready" array
                    read_packet(length);
                    
                    // Verify using XMODEM 16 CRC
                    uint16_t readCrc = read_two_bytes();
                    uint16_t calculatedCrc = crc16xmodem(channelValues, length);
                    if (readCrc == calculatedCrc) {
                        // Valid packet (no corruption)
                        write();
                    }
                    DSCOM_STATE_READY;
                }
                break;
        }
        // Can add a routine here to turn lights off after a specific time with no signal, etc.     
    }
    
    return 0;
}
