#ifndef _DMX_H
#define _DMX_H

/*****************************************************************************
*                           dmx.h
*
* Simple Library for a DMX Reciever/device
*
*
* Hardware:
*  Requires a EUSART
*
*
* CODE OWNERSHIP AND DISCLAIMER OF LIABILITY
*
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in
* all derivatives hereto.  You may use this code, and any derivatives created
* by any person or entity by or on your behalf, exclusively with Microchips
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT
* LIMITED TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND
* FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH
* MICROCHIPS PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
* APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY), STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL, PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
* CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE
* BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS
* CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY
* TO HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
* Author            Date            Ver     Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Michael Pearce    19 June 2012    1.00    Rewrite of code
*
* Michael Pearce    16 October 2012 1.01    Change dmx_set_command() to
*                                           dmx_set_start() to match standard
*
* Michael Pearce    23 October 2012 1.02    Added dmx_get_address() for rx
*                                           Added dmx_tx_done()
*                                           Added dmx_tx_enable(enable)
*
* Michael Pearce   5 November 2012  1.03    Added in time functions
*                                           Renamed some parameters
*
* Michael Pearce   6 March 2013     1.04    Added dmx rx timeout function
*                                           Fix missing Second flag update
******************************************************************************/
#include <stdint.h>
#include "dmxconfig.h"


// To allow for inline in the future for the interrupt functions
#ifndef inline 
#define inline
#endif

// Common

/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_init(<FONT COLOR=BLUE>void</FONT>)</B>
*
*  Initialises the DMX512A Library
*
*  Please add near the beginning of you main() function before your main loop
*/
extern void dmx_init(void);

/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_interrupt(<FONT COLOR=BLUE>void</FONT>)</B>
*
* This processes all the DMX512 related Interrupts.
*
* Please add this to your interrupt function.
*/
extern void dmx_interrupt(void);


/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_set_start(<FONT COLOR=BLUE>uint8_t start</FONT>)</B>
*
* This selects the DMX512A start code.
*
* For DMX_CONTROLLER this is the start code that is transmitted.
*
* For DMX_RECEIVER this is the start code that will be responded to.
*
* The dmx_init() function defaults this value to '0' which is the most
* common start code that is used.
*
* @param cmd - uint8_t - Sets the start code to use.
*/
extern void dmx_set_start(uint8_t start);



#ifdef DMX_TIMER

/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_timer_interrupt(<FONT COLOR=BLUE>void</FONT>)</B>
*
* DMX Timer interrupt. Called as part of dmx_interrupt() but can call seperately for optimization
*
*/
extern void dmx_timer_interrupt(void);

/**
* <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_timer_ms(<FONT COLOR=BLUE>void</FONT>)</B>
*
* DMX Timer ms flag. Set every ms.
*
* @returns 0 if flag not set or  1 if flag set (clears the flag)
*/
extern uint8_t dmx_timer_ms(void);

/**
* <B><FONT COLOR=BLUE>uint16_t</FONT> dmx_ms_count(<FONT COLOR=BLUE>void</FONT>)</B>
*
* @returns current ms count
*/
extern uint16_t dmx_ms_count(void);

/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_ms_clear(<FONT COLOR=BLUE>void</FONT>)</B>
*
* Clears the current ms count
*/
extern void     dmx_ms_clear(void);

/**
* <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_timer_sec(<FONT COLOR=BLUE>void</FONT>)</B>
*
* @returns 1 if a one second timeout has occured or 0 if not.
*/
extern uint8_t  dmx_timer_sec(void);

/**
* <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_sec_count(<FONT COLOR=BLUE>void</FONT>)</B>
*
* @returns current second count
*/
extern uint8_t  dmx_sec_count(void);

/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_sec_clear(<FONT COLOR=BLUE>void</FONT>)</B>
* 
* Clears the current second count.
*/
extern void     dmx_sec_clear(void);


#endif


// Controller Specific
#ifdef DMX_CONTROLLER
/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_tx_interrupt(<FONT COLOR=BLUE>void</FONT>)</B>
*
* DMX Transmit interrupt. Called as part of dmx_interrupt() but can call seperately for optimization
*
*/
extern void dmx_tx_interrupt(void);


/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_tx_set_start(<FONT COLOR=BLUE>uint8_t cmd</FONT>)</B>
*
* This selects the DMX512A start code for the controller
*
* The dmx_init() function defaults this value to '0' which is the most
* common start code that is used.
*
* @param cmd - uint8_t - Sets the start code to use.
*/
extern void dmx_tx_set_start(uint8_t cmd);

/**
* <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_tx_get_start(<FONT COLOR=BLUE>void</FONT>)</B>
*
* Returns the DMX512A start code in use
*
* @return Returns current start code in use
*/
extern uint8_t dmx_tx_get_start(void);


/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_write_byte(<FONT COLOR=BLUE>uint16_t addr, uint8_t data</FONT>)</B>
*
* This writes a single byte to the specific address in the TX buffer.
*
* @param addr - uint16_t - range 1 to DMX_TX_BUFFER_SIZE - a 0 is invalid
* @param data - uint8_t - data to insert in the buffer at location addr
*/
extern void dmx_write_byte(uint16_t addr, uint8_t data);



/**
 * <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_tx_read_byte(<FONT COLOR=BLUE>uint16_t addr</FONT>)</B>
 *
 * This Reads a byte from the tx buffer - usefull for manipulation of data in situe.
 */
extern uint8_t dmx_tx_read_byte(uint16_t addr);


/**
 * <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_tx_done(<FONT COLOR=BLUE>void</FONT>)</B>
 *
 * Indicates when a packet has been sent and starting a new one. Self clears
 *
 * @return Returns 1 if new data sent 0 if not
 *
 */
extern uint8_t dmx_tx_done(void);


/**
 * <B><FONT COLOR=BLUE>void</FONT> dmx_tx_enable(<FONT COLOR=BLUE>uint8_t</FONT>)</B>
 *
 * Enables or disable the transmission sequence. i.e. Turns output on/off
 *
 * @param enable - uint8_t - 1 = turn TX on, 0 = turn TX Off
 *
 */
extern void dmx_tx_enable(uint8_t enable);

/**
 * <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_tx_get_enable(<FONT COLOR=BLUE>void</FONT>)</B>
 *
 * @return Returns 1 if enabled, 0 if disabled
 *
 */
extern uint8_t dmx_tx_get_enable(void);

/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_write(<FONT COLOR=BLUE> uint16_t addr, uint8_t *data, uint8_t num</FONT>)</B>
*
* Copies an array of data to the output buffer.
*
*  This buffer write is performed autonomously - i.e. interrupts disabled
*
* @param addr - uint16_t - address in th ebuffer to start writing to
* @param *data - uint8_t - pointer to the data to copy to the buffer
* @param num - uint8_t - number of bytes to copy to the buffer. Limited range.
*/
extern void dmx_write( uint16_t addr, uint8_t *data, uint8_t num);




#endif // END DMX_CONTROLLER SPECIFICS


// Reciever Specific
#ifdef DMX_RECEIVER

/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_rx_interrupt(<FONT COLOR=BLUE>void</FONT>)</B>
*
* DMX Receive interrupt. Called as part of dmx_interrupt() but can call seperately for optimization
*
*/
extern inline void dmx_rx_interrupt(void);


/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_rx_set_start(<FONT COLOR=BLUE>uint8_t cmd</FONT>)</B>
*
* This selects the DMX512A start code for the receiver
*
* The dmx_init() function defaults this value to '0' which is the most
* common start code that is used.
*
* @param cmd - uint8_t - Sets the start code to use.
*/
extern void dmx_rx_set_start(uint8_t cmd);

/**
* <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_rx_get_start(<FONT COLOR=BLUE>void</FONT>)</B>
*
* Returns the DMX512A start code in use
*
* @return Returns current start code in use
*/
extern uint8_t dmx_rx_get_start(void);



/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_set_address(<FONT COLOR=BLUE>uint16_t base</FONT>)</B>
*
* Sets the base address to start reading data from.
* Valid address range for DMX512 is 1 to 512  (But will work beyond the DMX range for custom protocol)
* An address of "0" will not respond to DMX data causing the DMX 1.2 second timeout to occur effectively disabling DMX.
*
* @param base - uint16_t - base address to use.
*/
extern inline void dmx_set_address(uint16_t base);


/**
* <B><FONT COLOR=BLUE>uint16_t</FONT> dmx_get_address(<FONT COLOR=BLUE>void</FONT>)</B>
*
* Gets the base address to start reading data from.
*
* @return Returns Base address in use.
*/
extern inline uint16_t dmx_get_address(void);

/**
* <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_new_data(<FONT COLOR=BLUE>void</FONT>)</B>
*
* Indicates if new data has been recieved
*
* @return Returns 0 for no new data, or the number of bytes of new data received
*/
extern uint8_t dmx_new_data(void);

/**
* <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_read_byte(<FONT COLOR=BLUE>uint8_t offset</FONT>)</B>
*
* Indicates if new data has been recieved
*
* @param  offset - uint8_t - sets the offset into the RX buffer to read from
* @return Returns the byte read from the RX Buffer at the offset address
*/
extern uint8_t dmx_read_byte(uint8_t offset);

/**
* <B><FONT COLOR=BLUE>void</FONT> dmx_read( <FONT COLOR=BLUE>uint8_t offset, uint8_t *data, uint8_t num</FONT>)</B>
*
* Indicates if new data has been recieved
*
* @param offset - uint8_t - sets the offset into the RX buffer to read from
* @param *data - uint8_t - Pointer to where to copy the data to
* @param num - uint8_t - Number of bytes to copy. Limited to buffer range.
*
* @return Number of Bytes copied.
*/
extern uint8_t dmx_read( uint8_t offset, uint8_t *data, uint8_t num);



/**
* <B><FONT COLOR=BLUE>uint8_t</FONT> dmx_rx_timeout( <FONT COLOR=BLUE>void</FONT>)</B>
*
* Indicates if a data timeout has occured
* Changing the DMX_RX_TIMEOUT_MS changes the timeout limit (ms counter)
*
* @return 1 if timeout in effect, 0 if everything is still running
*/
extern uint8_t dmx_rx_timeout(void);

#endif // END DMX_RECEIVER SPECIFICS


// Back Channel Specific
#ifdef DMX_BACKCHANNEL
 #warning DMX512 Back Channel not implimented. It will be available in the future
#endif // END DMX_BACKCHANNEL SPECIFICS



#endif
