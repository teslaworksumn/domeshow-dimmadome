/*****************************************************************************
*                           dmx.c
*
* Simple Library for a DMX Reciever or Transmitter
*
*
* Hardware:
*       EUSART
*       TIMER
*
*
* CODE OWNERSHIP AND DISCLAIMER OF LIABILITY
*
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in
* all derivatives hereto.  You may use this code, and any derivatives created
* by any person or entity by or on your behalf, exclusively with Microchip�s
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
* Michael Pearce    20 June 2012    1.00    Rewrite of code
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
*
* Michael Pearce   17 March 2013    1.05    Fixed addressing to be 1 to 512
*                                           Address of 0 is now "Off" for RX
*                                           Write to addr 0 is ignored for TX
******************************************************************************/
#include <htc.h>
#include "dmx.h"


//============== VARIABLES ===============

#ifdef DMX_RECEIVER
/** DMX_RxData<BR> Receive Data Buffer  */
volatile uint8_t DMX_RxData[DMX_RX_BUFFER_SIZE];
/** DMX_RxChannel<BR> Base Address / Channel to start reading from */
uint16_t         DMX_RxChannel=0;
/** DMX_RxStart<BR> The Start Code for a valid data stream */
uint8_t          DMX_RxStart=0;
/** DMX_AddrCount<BR> Address counter used in the interrupt routine */
volatile uint16_t DMX_RxAddrCount=0;
/** *DMX_RxDataPtr<BR> Pointer to the receive data buffer. Used in interrupt */
volatile uint8_t *DMX_RxDataPtr;
/** DMX_RxState<BR> Current state in the receive state machine */
volatile uint8_t DMX_RxState=0;
/** DMX_RxTimer<BR> Counts ms since last overflow - used for 1 second timeout */
volatile uint16_t DMX_RxTimer=0;

#endif

#ifdef DMX_CONTROLLER
/** DMX_TxData<BR> Transmit Data Buffer  */
volatile uint8_t   DMX_TxData[DMX_TX_BUFFER_SIZE];
/** DMX_TxCount<BR> Counts how many bytes sent. Used in interrupt  */
uint16_t         DMX_TxCount=0;
/** DMX_TxStart<BR> Start code to transmit  */
uint8_t          DMX_TxStart=0;
/** *DMX_TxByte<BR> Pointer to data to send  */
volatile uint8_t *DMX_TxByte;
/** DMX_TxState<BR> State for the Transmit state machine  */
volatile uint8_t DMX_TxState=0;

 enum
 {
    TX_BREAK,
    TX_MAB,
    TX_START,
    TX_DATA
 };


#endif


#ifdef DMX_TIMER

volatile uint8_t DMX_TimerState=0;

typedef struct 
{
    union
    {
        struct
        {
            unsigned int MS    :1;
            unsigned int SEC   :1;
            unsigned int MIN   :1;
            unsigned int HR    :1;

            unsigned int BREAK :1;  // Start BREAK Timer
            unsigned int MAB   :1;  // Start MAB Timer
            unsigned int spare :2;
        };
        uint8_t flags;
    };

    uint16_t MS_Count;
    uint8_t  SEC_Count;
    uint8_t  MIN_Count;
    uint8_t  HR_Count;


}DMX_TIMER_DATA;

volatile DMX_TIMER_DATA DMX_Timer;

#endif


/** DMX_FLAGS <BR> */
typedef struct
{
    // Recieve Flags
    /**  Indicates new data was recieved */
    unsigned int RxNew    :1;
    /**  Indicate valid break detected */
    unsigned int RxBreak  :1;
    /**  Indicates valid start is detected so store data */
    unsigned int RxStart  :1;
    /** Indicates 1 second timout occured */
    unsigned int RxTimeout :1;

    // Transmit Flags
    /** Indicated transmission is in progress */
    unsigned int TxRunning :1;
    /** Break active */
    unsigned int TxBREAK   :1;
    /**  Mark after Break Active */
    unsigned int TxMAB     :1; 

    /** Indicates a transmission is complete (Last Byte loaded into buffer) */
    unsigned int TxDone   :1;

    /** DMX_FLAGS Used to pass information from the ISR */
}DMX_FLAGS;

/** DMX_Flags <BR> Set in the ISR to indicate to helper functions etc */
volatile DMX_FLAGS DMX_Flags;


//=================== COMMON FUNCTIONS =====================


/*
* void dmx_init(void);
*
* Initialises the DMX512A Library
*
*/
void dmx_init(void)
{
    uint16_t load;

#ifndef BAUDPIN_OVERRIDE
    #ifdef DMX_USE_DIR_PIN
        #ifdef DMX_RECEIVER
        DMX_DIR_PIN=DMX_DIR_RX;      // Direction pin = RX = 0
        #else
        DMX_DIR_PIN=DMX_DIR_TX;      // Direction pin = TX = 1
        #endif
         DMX_DIR_TRIS=0;              // Set Direction pin as output
    #endif

    // Common Configuration the EUSART for 250KBPS
    DMX_SPBRGL = DMX_SPBRGL_LOAD;
    DMX_SPBRGH = DMX_SPBRGH_LOAD;

    DMX_BAUDCON = DMX_BAUDCON_LOAD;
    DMX_RCSTA = DMX_RCSTA_LOAD;
    DMX_TXSTA = DMX_STA_LOAD;
#endif

#ifdef DMX_RECEIVER
    // Clear the buffer
    for(load=0; load<DMX_RX_BUFFER_SIZE;load++)
    {
        DMX_RxData[load]=0;
    }
    // Set Default Values
    DMX_RxChannel=0;
    DMX_RxStart=0;
    DMX_RxAddrCount=0;
    DMX_RxState=0;
    DMX_RxDataPtr = &DMX_RxData[0];

    // Set up EUSART Options for RX
    DMX_RxChannel=0xFFFF;     // Set Initial channel number to neutral zone
    load=DMX_RCREG;           // Clear any data in the buffer
    load=DMX_RCREG;
    //DMX_RCIF=0;             // Clear interrupt flag done in HW
    DMX_RCIE=1;               // Enable the Receive interrupt


#endif

#ifdef DMX_CONTROLLER
 // Clear the buffer!
 for(load=0; load<DMX_TX_BUFFER_SIZE;load++)
 {
     DMX_TxData[load]=0;
 }

 // Set up the initial values
 DMX_TxCount=0;
 DMX_TxStart=0;
 DMX_TxState=TX_BREAK;

 DMX_Flags.TxBREAK=0;
 DMX_Flags.TxMAB=0;
 DMX_Flags.TxRunning=1;   // Enable DMX to start immediately
 DMX_Flags.TxDone=0;


 // Set up EUSART options for TX
 DMX_TX_PIN=1;       // Set pin High
 DMX_TX_TRIS=0;      // Set TX pin as output
 DMX_TXIE=1;         // Only Transmit when ready

#ifdef DMX_USE_TX_EN
 DMX_TX_EN=DMXTXDISABLE;
 DMX_TX_EN_TRIS=0;
#endif


#endif

#ifdef DMX_TIMER
 DMX_TimerState=0;
 
 DMX_Timer.HR_Count=0;
 DMX_Timer.MIN_Count=0;
 DMX_Timer.SEC_Count=0;
 DMX_Timer.flags=0;

 // Set up the Timer for 1ms Interrupts
 DMX_TMR=DMX_TMR_LOAD_1MS;
 DMX_TCON=DMX_TCON_LOAD;
 DMX_TMRIF=0;
 DMX_TMRIE=1;

 
 
#endif

 PEIE=1; // Turn the peripheral interrupt ON

}





/*
* void dmx_set_start(uint8_t cmd);
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
* cmd - uint8_t - Sets the start code to use.
*/
void dmx_set_start(uint8_t start)
{
#ifdef DMX_CONTROLLER
    DMX_TxStart = start;
#endif

#ifdef DMX_RECEIVER
    DMX_RxStart = start;
#endif
}


/*
* void dmx_interrupt(void)
*
* This processes all the DMX512 related Interrupts.
* This should be made inline or #defined as a Macro
*/
void dmx_interrupt(void)
{
#ifdef DMX_TIMER
    dmx_timer_interrupt();
#endif

#ifdef DMX_RECEIVER
    dmx_rx_interrupt();
#endif

#ifdef DMX_CONTROLLER
    dmx_tx_interrupt();
#endif


}


//=================== DMX TIMER INTERRUPT ====================
#ifdef DMX_TIMER
enum
{
    TIMER_1MS,
    TIMER_BREAK,
    TIMER_MAB,
    TIMER_FILL
};

inline void dmx_timer_interrupt(void)
{
    if(DMX_TMRIE && DMX_TMRIF)
    {
        DMX_TMRIF=0;

        switch(DMX_TimerState)
        {
            default:
                DMX_TimerState=TIMER_1MS;

            case TIMER_1MS:
#ifdef DMX_CONTROLLER
                if(DMX_Timer.BREAK==1 && DMX_Flags.TxRunning==1)  // Check if sending a new BREAK
                {
                    DMX_TMR = DMX_TMR_LOAD_BREAK;
                    DMX_TX_PIN=0;
                    DMX_TimerState=TIMER_BREAK;
                    DMX_Timer.BREAK=0;
                }
                else
                {
                    DMX_TMR = DMX_TMR_LOAD_1MS;
                }
#else
                DMX_TMR = DMX_TMR_LOAD_1MS;
#endif

#ifdef DMX_RECEIVER
                DMX_RxTimer++;          // Inc timeout counter for receiver
                if(DMX_RxTimer == DMX_RX_TIMEOUT_MS)
                {
                    DMX_RxTimer = DMX_RX_TIMEOUT_MS + 1;
                    DMX_Flags.RxTimeout=1;
                }
#endif

                DMX_Timer.MS_Count++;            // Inc the MS Counter
                DMX_Timer.MS=1;                  // Set the ms flag
                if(DMX_Timer.MS_Count==1000)     // Check for 1 second
                {
                    DMX_Timer.MS_Count=0;
                    DMX_Timer.SEC_Count++;
                    DMX_Timer.SEC=1;
                    if(DMX_Timer.SEC_Count==60)  // Check for Minute
                    {
                        DMX_Timer.SEC_Count=0;
                        DMX_Timer.MIN_Count++;
                        DMX_Timer.MIN=1;

                        if(DMX_Timer.MIN_Count==60) // Check for Hour
                        {
                            DMX_Timer.MIN_Count=0;
                            DMX_Timer.HR_Count++;
                            DMX_Timer.HR=1;
                        }
                    }
                }
                break;

#ifdef DMX_CONTROLLER
            case TIMER_BREAK:
                DMX_TX_PIN=1;               // Set pin high for MAB
                DMX_TMR = DMX_TMR_LOAD_MAB;   // Load the MAB time
                DMX_TimerState=TIMER_MAB;   // Next state is MAB end
                break;


            case TIMER_MAB:
                DMX_TXEN=1;    // Re-enable EUSART control of pin
                DMX_TXIE=1;    // Re-Enable EUSART Interrupt
                DMX_TMR = DMX_TMR_LOAD_FILL; // Load the Filler time
                DMX_TimerState=TIMER_1MS;  // Next int is the 1ms
                break;
#endif

        }
    }


}



uint8_t dmx_timer_ms(void)
{
    if(DMX_Timer.MS)
    {
        DMX_Timer.MS=0;
        return(1);
    }
    return(0);
}

uint16_t dmx_ms_count(void)
{
    return(DMX_Timer.MS_Count);
}

void dmx_ms_clear(void)
{
    DMX_Timer.MS_Count=0;
}


uint8_t dmx_timer_sec(void)
{
    if(DMX_Timer.SEC)
    {
        DMX_Timer.SEC=0;
        return(1);
    }
    return(0);
}


uint8_t dmx_sec_count(void)
{
    return(DMX_Timer.SEC_Count);
}

void dmx_sec_clear(void)
{
    DMX_Timer.SEC_Count=0;
}



#endif




//=============== DMX CONTROLLER SPECIFIC ======================
#ifdef DMX_CONTROLLER


//=================== DMX CONTROLLER INTERRUPT ===============



inline void dmx_tx_interrupt(void)
{
    if(DMX_TXIE && DMX_TXIF)
    {
        switch(DMX_TxState)
        {
            case TX_BREAK:
                DMX_TX_PIN=1;      // Ensure PIN will be high
                DMX_TXEN=0;        // Disable the EUSART's control of the TX pin
                DMX_Timer.BREAK=1; // Indticate a BREAK to the timer interrupt
                DMX_TXIE=0;        // Disable the EUSART Interrupt 
                DMX_TxState=TX_START;// Move to Start code sequence
                break;
                
            case TX_MAB:
                break;
                
            case TX_START:   // Send start and set up for data stream
                DMX_TXREG = DMX_TxStart;
                DMX_TxByte = &DMX_TxData[0]; // Point to First byte
                DMX_TxCount = 0;             // Clear the address counter
                DMX_TxState = TX_DATA;
                break;

            case TX_DATA:  // Send the data
                if(DMX_TxCount < DMX_TX_BUFFER_SIZE)
                {
                    DMX_TXREG = *DMX_TxByte;
                    DMX_TxByte++;   // Point to next byte
                    DMX_TxCount++;  // Count the addresses
                }
                else
                {

// If you observe a glitch on the first byte after the BREAK then you need
// to use this ERATTA fix. Uncomment DMX_USART_ERATTA in dmxconfig.h
#ifdef DMX_USART_ERATTA


                    if(DMX_TRMT==1) // Only move on when TSR is empty
                    {
                      DMX_TxState=TX_BREAK; // Next interrupt go to BREAK
                      DMX_Flags.TxDone=1;
                    }
#else
                   DMX_TXREG=0;          // Put one more byte in the buffer to force next interrupt
                   DMX_TxState=TX_BREAK; // Next interrupt go to BREAK
                   DMX_Flags.TxDone=1;
#endif

                }
                break;


                
        }
    }


}


/*
* void dmx_tx_set_start(uint8_t cmd);
*
* This selects the DMX512A start code for the CONTROLLER.
*
* The dmx_init() function defaults this value to '0' which is the most
* common start code that is used.
*
* cmd - uint8_t - Sets the start code to use.
*/
void dmx_tx_set_start(uint8_t start)
{
    DMX_TxStart = start;
}


/*
* uint8_t dmx_tx_get_start(void)
*
* Returns the DMX512A start code in use
*
* @return Returns current start code in use
*/
uint8_t dmx_tx_get_start(void)
{
  return(DMX_TxStart);
}


/*
* void dmx_write_byte(uint16_t addr, uint8_t data);
*
* This writes a single byte to the specific address in the TX buffer.
*
* addr - uint16_t - range 1 to DMX_TX_BUFFER_SIZE  (Range 1 to 512)
* data - uint8_t - data to insert in the buffer at location addr
*/
void dmx_write_byte(uint16_t addr, uint8_t data)
{
    if(addr <= DMX_TX_BUFFER_SIZE && addr !=0)
    {
        DMX_TxData[addr-1]=data;
    }
}


/*
* uint8_t dmx_read_byte(uint16_t addr);
*
* This reads a single byte to the specific address in the TX buffer.
*
* addr - uint16_t - range 1 to DMX_TX_BUFFER_SIZE  (Range 1 to 512)
* data - uint8_t - data to insert in the buffer at location addr
*/
uint8_t dmx_tx_read_byte(uint16_t addr)
{
    if(addr <= DMX_TX_BUFFER_SIZE && addr !=0)
    {
        return(DMX_TxData[addr-1]);
    }
    return(0);
}

/*
 * uint8_t dmx_tx_done(void)
 *
 * Indicates when a packet has been sent and starting a new one. Self clears
 *
 * Returns 1 if new data sent 0 if not
 *
 */
uint8_t dmx_tx_done(void)
{
    if(DMX_Flags.TxDone==1)
    {
        DMX_Flags.TxDone=0;
        return(1);
    }
    return(0);
}

/*
 * void dmx_tx_enable(uint8_t)
 *
 * Enables or disable the transmission sequence. i.e. Turns output on/off
 *
 * enable - uint8_t - 1 = turn TX on, 0 = turn TX Off
 *
 */
extern void dmx_tx_enable(uint8_t enable)
{
    if(enable==1)
    {
        DMX_Flags.TxRunning=1;
#ifdef DMX_USE_TX_EN
        DMX_TX_EN=DMXTXENABLE;
#endif
    }
    else
    {
        DMX_Flags.TxRunning=0;
#ifdef DMX_USE_TX_EN
        DMX_TX_EN=DMXTXDISABLE;
#endif

    }
}

/*
 * uint8_t dmx_tx_get_enable(void)
 *
 * @return Returns 1 if enabled, 0 if disabled
 *
 */
uint8_t dmx_tx_get_enable(void)
{
    return (DMX_Flags.TxRunning);
}


/*
* void dmx_write( uint16_t addr, uint8_t *data, uint8_t num)
*
* Copies an array of data to the output buffer.
*
*  This buffer write is performed autonomously - i.e. interrupts disabled
*
* addr - uint16_t - address in the buffer to start writing to 1 to DMX_TX_BUFFER_SIZE  (0 is invalid)
* *data - uint8_t - pointer to the data to copy to the buffer
* num - uint8_t - number of bytes to copy to the buffer. Limited range.
*/
void dmx_write( uint16_t addr, uint8_t *data, uint8_t num)
{
    if(addr <= DMX_TX_BUFFER_SIZE && addr !=0)
    {
       addr--;  // Adjust offset to match 0-511 range for buffer
       GIE=0;   // Go Atomic ##### SHOULD JUST DISABLE TXIE
       for(;num--;num>0)
       {
           DMX_TxData[addr]=*data;
           addr++;
           data++;
           if(addr >= DMX_TX_BUFFER_SIZE)num=0;
       }
       GIE=1; // End Atomic
    }

}

#endif //=============== END DMX_CONTROLLER SPECIFIC =============


//================ RECEIVER SPECIFIC ====================
#ifdef DMX_RECEIVER


//=================== DMX RECEIVER INTERRUPT ================
 enum
 {
    RX_WAIT_FOR_BREAK,
    RX_WAIT_FOR_START,
    RX_READ_DATA
 };
 
 void toggle()
 {
     if(RC3)
         RC3=0;
     else
         RC3=1;
 }
 
inline void dmx_rx_interrupt(void)
{
    uint8_t RxDat;

    if(DMX_RCIE & DMX_RCIF)
    {
        DMX_RCIF=0; //Clear interrupt flag
        
        // Check for Break - i.e. Framing Error
        if(DMX_FERR)
        {
           RxDat=DMX_RCREG;  // Clear the Framing Error - do not read before the bit test
           DMX_Flags.RxBreak=1;                 // Indicate a break
           if(DMX_RxState==RX_READ_DATA)
               TXREG1=0xAA;
           DMX_RxState=RX_WAIT_FOR_START;
           DMX_RxTimer=0;
           

#ifdef DMX_ACK_ALL_RX
           DMX_Flags.RxNew=1; // FOR USB ADAPTER WE WANT ANY DATA WE RECIEVE
#endif
        }

        switch(DMX_RxState)
        {
            case RX_WAIT_FOR_BREAK:
                RxDat=DMX_RCREG;   // Just keep clearing the buffer until overflow.
                TXREG1=0xaa;
                break;

            case RX_WAIT_FOR_START:
                //TXREG1=0xbb;
                if(DMX_RCIF)   // make sure there is data avaliable (ie not a break)
                {
                    RxDat=DMX_RCREG;
                    TXREG1=RxDat;
                    //Troubleshooting code
                    if(RxDat==DMX_RxStart)
                    {
                        //toggle();
                        // Valid Start Received
                        DMX_RxState = RX_READ_DATA;
                        DMX_RxDataPtr = &DMX_RxData[0]; // Point to Buffer
                        DMX_RxAddrCount = 1;            // Reset current addr - Start at address 1! (zero is OFF)
                        DMX_Flags.RxStart = 1;          // Indicate a Start
                    }
                    else
                    {
                        DMX_RxState=RX_WAIT_FOR_BREAK;
                    }
                }
                break;

            case RX_READ_DATA:
                TXREG1=0xcc;
                toggle();
                
                RxDat=DMX_RCREG;

                if(DMX_RxAddrCount >= DMX_RxChannel && DMX_RxChannel !=0 )  // A selection of channel zero is "OFF"
                {
                    *DMX_RxDataPtr=RxDat;
                    DMX_RxDataPtr++;
                }
                DMX_RxAddrCount++;

                // Check for end of data packet we can read
                if(DMX_RxAddrCount >= (DMX_RxChannel + DMX_RX_BUFFER_SIZE) && DMX_RxChannel !=0 )
                {
                    DMX_Flags.RxNew=1;
                    DMX_RxState=RX_WAIT_FOR_BREAK;
                    DMX_RxTimer = 0;
                    DMX_Flags.RxTimeout=0;
                }
                break;
        }

//        if(DMX_RxTimer > DMX_RX_TIMEOUT_MS)
//        {
//            DMX_Flags.RxTimeout=1;
//            DMX_RxTimer=0;
//        }
    }
}

/*
* void dmx_rx_set_start(uint8_t cmd);
*
* This selects the DMX512A start code for the RECEIVER.
*
* The dmx_init() function defaults this value to '0' which is the most
* common start code that is used.
*
* cmd - uint8_t - Sets the start code to use.
*/
void dmx_rx_set_start(uint8_t start)
{
    DMX_RxStart = start;
}

/**
* uint8_t dmx_rx_get_start(void)
*
* Returns the DMX512A start code in use
*
* @return Returns current start code in use
*/
uint8_t dmx_rx_get_start(void)
{
  return(DMX_RxStart);
}


/*
* void dmx_set_address(uint16_t base)
*
* Sets the base address to start reading data from.
*
* base - uint16_t - base address to use.
*/
void dmx_set_address(uint16_t base)
{
    if(base > 511) base=511;
    DMX_RxChannel=base;
}


/*
* uint16_t dmx_get_address(void)
*
* Gets the base address to start reading data from.
*
* Returns Base address in use.
*/
uint16_t dmx_get_address(void)
{
  return (DMX_RxChannel);
}




/*
* uint8_t dmx_new_data(void)
*
* Indicates if new data has been recieved
*
* Returns 0 for no new data, or the number of bytes of new data received
*/
uint8_t dmx_new_data(void)
{
    if(DMX_Flags.RxNew==1)
    {
        DMX_Flags.RxNew=0;
        return(1);
    }
    return(0);
}

/*
* uint8_t dmx_read_byte(uint8_t offset)
*
* Indicates if new data has been recieved
*
* offset - uint8_t - sets the offset into the RX buffer to read from
* Returns the byte read from the RX Buffer at the offset address
*/
uint8_t dmx_read_byte(uint8_t offset)
{
    if(offset >= DMX_RX_BUFFER_SIZE)offset = DMX_RX_BUFFER_SIZE-1;
    return (DMX_RxData[offset]);
}

/*
* void dmx_read( uint8_t offset, uint8_t *data, uint8_t num)
*
* Indicates if new data has been recieved
*
* offset - uint8_t - sets the offset into the RX buffer to read from
* *data - uint8_t - Pointer to where to copy the data to
*  num - uint8_t - Number of bytes to copy. Limited to buffer range.
*
*/
uint8_t dmx_read( uint8_t offset, uint8_t *data, uint8_t num)
{
    uint8_t count=0;

    for(;num > 0; num--)
    {
        if( offset < DMX_RX_BUFFER_SIZE )
        {
            *data=DMX_RxData[offset];
            *data++;
            offset++;
            count++;
        }
        else
        {
          break;
        }
    }
    return(count);
}


uint8_t dmx_rx_timeout(void)
{
    if(DMX_Flags.RxTimeout==1) return(1);
    return(0);
}


#endif //============ END DMX_RECEIVER SPECIFIC =================


// Back Channel Specific
#ifdef DMX_BACKCHANNEL
 #warning DMX512 Back Channel will be available in the future
#endif // END DMX_BACKCHANNEL SPECIFICS













