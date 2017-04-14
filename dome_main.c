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
#pragma config XINST = OFF       // Not supported in free XC8

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

#include <stdint.h>
#include <xc.h>
#include "domeshow_lib.h"
#include "crc16_xmodem.h"

#define _XTAL_FREQ 48000000     //Fosc frequency for _delay

#define CHIP_ADDRESS 0
#define CHIP_CHANNELS 6
#define TOTAL_CHANNELS 144
#define RX_BUFFER_SIZE 0x03ff // 1024

// State of the domeshow RX
typedef enum {
    DSCOM_STATE_READY,
    DSCOM_STATE_PRE_PROCESSING,
    DSCOM_STATE_PROCESSING
} DSCOM_RX_STATE_t;

uint8_t startChannel = CHIP_ADDRESS * CHIP_CHANNELS;
uint8_t channelValues[TOTAL_CHANNELS];
DSCOM_RX_STATE_t dscom_rx_state = DSCOM_STATE_READY;
volatile uint8_t rxData[RX_BUFFER_SIZE];
uint16_t head = 0;
volatile uint16_t tail;
unsigned char crc_start = 0;
unsigned char crc_end = 0;
uint8_t num_magic_found = 0;
uint8_t magic[4] = {0xDE, 0xAD, 0xBE, 0xEF};

void setup(void) {
    
    timer_init();
    pin_init();
    uart_init();
        
    return;
}

__interrupt(high_priority) void isr() {
    uint8_t rxByte;
    uint16_t t;

    if(RC1IE & RC1IF)
    {
        RC1IF=0; // Clear interrupt flag

        // Check for framing error
        if(RCSTA1bits.FERR)
        {
            rxByte = RCREG1; // Clear framing error
        } else if (RCSTA1bits.OERR) {
            // clear error
        } else {
            rxByte = RCREG1;
            t = tail;
            rxData[t] = rxByte;
            t = (t + 1) & RX_BUFFER_SIZE;
            tail = t;
        }
    }
}

uint16_t get_tail() {
    RC1IE = 0; // Disable interrupts to read value
    uint16_t t = tail;
    RC1IE = 1; // Re-enable interrupts
    return t;
}

__inline void write() {
    //Not sure what's up with the ordering here...
    CCPR4L = channelValues[startChannel + 3];      //RP7
    CCPR5L = channelValues[startChannel + 1];      //RP8
    CCPR6L = channelValues[startChannel + 2];      //RP9
    CCPR7L = channelValues[startChannel + 0];      //RP10
    CCPR8L = channelValues[startChannel + 4];      //RP12
    CCPR9L = channelValues[startChannel + 5];      //RP17
}

/*
 * Returns the number of bytes in the ring buffer that can be used currently.
 * Intentionally off by one to account for race conditions with interrupts
 * (giving a one byte buffer between head and tail)
 */
uint16_t bytes_available() {
    uint16_t t = get_tail();
    uint16_t available = t - head;
    if (t < head) { // Looped around ring buffer
        available = RX_BUFFER_SIZE - head + t;
    }
    return available;
}

/*
 * Reads in the next byte from the ring buffer and properly increments head.
 * Assumes that there is at least one byte to read
 */
__inline uint8_t read_byte() {
    RC1IE = 0; // Disable interrupts to read value
    uint8_t byte = rxData[head];
    RC1IE = 1; // Re-enable interrupts
    head = (head + 1) & RX_BUFFER_SIZE;
    return byte;
}

/*
 * Reads the next two bytes from the ring buffer as a uint16.
 * Assumes that there are at least two bytes to read
 */
__inline uint16_t read_two_bytes() {
    uint16_t highByte = read_byte();
    uint16_t lowByte = read_byte();
    return (highByte << 8) | lowByte;
}

void read_packet(uint16_t length) {
    unsigned int i = 0;
    while (i < length) {
        channelValues[i] = read_byte();
        i++;
    }
}

int main(void) {

    setup();
    
    uint8_t rxByte;
    uint16_t length;
    uint16_t num_bytes;
    
    while(1)
    {
        switch (dscom_rx_state) {
            case DSCOM_STATE_READY:
                // Wait for magic bytes
                num_bytes = bytes_available();
                if (num_bytes > 0) {
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
                    dscom_rx_state = DSCOM_STATE_READY;
                }
                break;
            default:
                break;
        }
    }
    
    return 0;
}
