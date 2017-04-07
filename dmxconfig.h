#ifndef _DMXCONFIG_H
#define _DMXCONFIG_H
/*****************************************************************************
*                           dmxconfig.h
*
* Simple Library for a DMX Reciever/device
* Alter settings in this file to select the options you need and
* to define pins etc that are used.
* Please read comment at the beginning of each section carefully
*
* By default this is configured for the Lighting Communications Motherboard
* with the DMX512A Interface board.
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
******************************************************************************/
/**
 * <B>Microchip Technology Inc DMX512A Library</B>
 * 
 * This library has been developed to make it easy to impliment the
 * basic DMX512A Controller and Reciever functions on a PIC Microcontroller.
 *
 * Please adjust the settings in dmxconfig.h to match your required
 * hardware and DMX512A configuration.
 *
 * Currently this library has only been tested on PIC16F1xxx parts but should
 * be easy to port to the full range of PIC Microcontroller products that
 * have a suitable EUSUART. This will be done in the future.
 *
 */
#define DMX


/**
* <B>MODES OF OPERATION</B>
*
*  <B>DMX_CONTROLLER</B> - Transmits DMX512A data to recievers\n
*  <B>DMX_RECEIVER</B> - Reads the DMX512A data\n
* 
*  <B>DMX_BACKCHANNEL</B> - back channel to send data back to a Controller\n
*  Note: the back channel should only be selected for DMX_RECEIVER
*
*
**/
//#define DMX_CONTROLLER          // MUST have the 1 to work
#define DMX_RECEIVER              // MUST have the 1 to work
//#define DMX_BACKCHANNEL


/**
* <B>DMX_TIMER</B>
* 
* This enables the timer interrupt features
* If you disable this function you MUST write your own function to provide
* the appropitate timing flags required by the DMX library.
* 
* The best option is to leave it as is unless you really need to use the timer
* for something else! In that case, please ensure you include what is required 
* to keep the DMX running! <BR>
* i.e. copy then modify the existing timer interrupt section.
* 
*/
#define DMX_TIMER



/*****************************************************************************
*                     CONTROLLER SPECIFIC OPTIONS
*****************************************************************************/
#ifdef DMX_CONTROLLER
#define DMX_USE_TX 1
/**
 * <B>DMX_TX_BUFFER_SIZE</B>
 * 
 *  Select the size of the transmit buffer\n
 *  This is the Maximum number of channels that will be sent.\n
 *  DMX512a requires a minimum of 20 and a maximum of 512\n
 *  This depends on how much RAM your PIC has available.\n
 * 
 */
#define DMX_TX_BUFFER_SIZE 128

#else
 #define DMX_USE_TX 0

#endif // End of DMX Controller specifics

/*****************************************************************************
*                     RECEIVER SPECIFIC OPTIONS
*****************************************************************************/
#ifdef DMX_RECEIVER
#define DMX_USE_RX 1
/**
 * <B>DMX_RX_BUFFER_SIZE</B>
 * 
 *  Select the size of the receive buffer.\n
 *  This is the number of DMX512 Channels you will recieve\n
 *  For example a RGBW fixture may use 4 channels where a scanner may need 10\n
 * 
 */
#define DMX_RX_BUFFER_SIZE 6


/**
 * <B>DMX_RX_TIMEOUT_MS</B>
 *
 *  Selects the ms between breaks that sets the timeout flag.<BR>
 *  This is usually just over 1 second and is common for DMX Fixtures
 *  to go into a blackout or default mode if this occurs (i.e. signal lost)
 */
#define DMX_RX_TIMEOUT_MS 1200

#else
 #define DMX_USE_RX 0
#endif  // End of Receiver Specifics



/*****************************************************************************
*                     BACKCHANNEL SPECIFIC OPTIONS
*
*****************************************************************************/
#ifdef DMX_BACKCHANNEL

/**
 * <B>DMX_BC_BUFFER_SIZE</B>
 *
 *  Select the size of the back channel buffer.\n
 *  This is the number of DMX512 Channels you will send as a back channel response\n
 *
 *  <B>NOTE: BACKCHANNEL IS NOT CURRENTLY IMPLIMENTED</B>
 */
#define DMX_BC_BUFFER_SIZE 6


#endif // End of Back channel specific defines



/*****************************************************************************
*                          HARDWARE SETTINGS
*
*****************************************************************************/


// Direction pin is used on the demo board so that both Transmit and recieve
// can be demonstrated using a single SN75176 transciever.
//#define DMX_USE_DIR_PIN         // Define if using a direction pin
//Don't know why this is necessary for our purposes.
#define DMX_DIR_PIN     LATC0   // LAT for the direction pin
#define DMX_DIR_TRIS    TRISC0  // TRIS for the direction pin
#define DMX_DIR_RX      1       // Pin setting for RX mode (Based on SN75176)
#define DMX_DIR_TX      0       // Pin Setting for TX mode (Based on SN75176)
//I reversed these... think they were wrong????

#define DMX_RX_PIN      LATC7   // RX pin used
#define DMX_RX_TRIS     TRISC7

#define DMX_TX_PIN      LATC6   // TX  pin used
#define DMX_TX_TRIS     TRISC6

// Select the EUSART that will be used
#define DMX_BAUDCON     BAUDCON1
#define DMX_RCSTA       RCSTA1
#define DMX_FERR        RCSTA1bits.FERR
#define DMX_OERR        RCSTA1bits.OERR
#define DMX_TXSTA       TXSTA1
#define DMX_TXEN        TXSTA1bits.TXEN     // TX Enable Bit
#define DMX_TRMT        TXSTA1bits.TRMT     // TX Shift reg status bit
#define DMX_RCREG       RCREG1  // RX Data register
//This used to be RCREG2???? Not sure why.
#define DMX_SPBRGL      SPBRG1
#define DMX_SPBRGH      SPBRGH1
#define DMX_TXREG       TXREG1  // TX data register

#define DMX_RCIE        RC1IE
#define DMX_RCIF        RC1IF
#define DMX_TXIE        TXIE
#define DMX_TXIF        TXIF


// Calculate the Preload values for the EUSART Configuration
// Using BRG16=1 & BRGH=0
// BRGH:BRGL = (FOSC / BAUD / 16) - 1
//           = (FOSC / 250000 / 16) -1
// For 32MHZ = (32M / 250K / 16) -1 = 7 = 0x0007
// #### NEED TO AUTO CALCULATE THIS - MANUAL FOR NOW BASED on 32MHz ####
#define DMX_SPBRGH_LOAD     0
#define DMX_SPBRGL_LOAD     11       
//We're using 48MHz. Initially this ^ was 7.
#define DMX_BRG16           1       // BRG16=1 & BRGH=0 = Fosc/16 - 16bit H:L
#define DMX_BRGH            0       // BRG16=1 & BRGH=1 = Fosc/4  - 16bit H:L

#define DMX_BAUDCON_LOAD    0b00000000 | (DMX_BRG16<<3)
// Use 9 bit mode to give the 2 stop bits required - set bit 9 to '1'
#define DMX_STA_LOAD        0b01000001 | (DMX_USE_TX <<5) | (DMX_BRGH << 2 )
#define DMX_RCSTA_LOAD      0b10000000 | (DMX_USE_RX <<4)


// Timer hardware options and load values

#define DMX_TMRIE       TMR1IE
#define DMX_TMRIF       TMR1IF
#define DMX_TMR         TMR1
#define DMX_TCON        T1CON
#define DMX_TCON_LOAD   0b00110001          // Fosc/4, 1:8 pre

// 32MHz / 4 / 8 = 1MHz 1/1MHz = 1us

// 1ms = 0xFFFF - 1000 = 0xFc17
#define DMX_TMR_LOAD_1MS     0xFc17  // Load value for 1ms

// 20us = 0xFFFF - 20 = 0xFFEB
#define DMX_TMR_LOAD_MAB     0xFFEB  // Load value for MAB      ( 20us)

// 180us = 0xFFFF - 180 = 0xFF4B
#define DMX_TMR_LOAD_BREAK   0xFF4B  // Load Value for BREAK    (180us)

// 800us = 0xFFFF - 800 = 0xFCDF  - Adjust to fine tune the 1ms total
#define DMX_TMR_LOAD_FILL   0xFCDF   // Load value to total 1ms (800us)


#endif    // END OF FILE

