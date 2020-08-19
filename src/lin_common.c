/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */
/** Descriptive Data Item Name

  @Summary
    Brief one-line summary of the data item.
    
  @Description
    Full description, explaining the purpose and usage of data item.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
  @Remarks
    Any additional remarks
 */
int global_data;


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */

/** 
  @Function
    int ExampleLocalFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Description
    Full description, explaining the purpose and usage of the function.
    <p>
    Additional description in consecutive paragraphs separated by HTML 
    paragraph breaks, as necessary.
    <p>
    Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

  @Precondition
    List and describe any required preconditions. If there are no preconditions,
    enter "None."

  @Parameters
    @param param1 Describe the first parameter to the function.
    
    @param param2 Describe the second parameter to the function.

  @Returns
    List (if feasible) and describe the return values of the function.
    <ul>
      <li>1   Indicates an error occurred
      <li>0   Indicates an error did not occur
    </ul>

  @Remarks
    Describe any special behavior not described above.
    <p>
    Any additional remarks.

  @Example
    @code
    if(ExampleFunctionName(1, 2) == 0)
    {
        return 3;
    }
 */
static int ExampleLocalFunction(int param1, int param2) {
    return 0;
}


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

// *****************************************************************************

/** 
  @Function
    uint8_t LIN_getChecksum(uint8_t length, uint8_t* data)

  @Summary
    Get Checksum for LIN Package.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */
uint8_t LIN_getChecksum(uint8_t length, uint8_t* data){
    uint16_t checksum = 0;
    
    for (uint8_t i = 0; i < length; i++){
        checksum = checksum + *data++;
        if(checksum > 0xFF)
            checksum -= 0xFF;
    }
    checksum = ~checksum;
    
    return (uint8_t)checksum;
}

void LIN_slave_init(uint8_t tableLength, const lin_rx_cmd_t* const command, void (*processData)(void)){
    LIN_rxCommand = command;
    LIN_rxCommandLength = tableLength;
    LIN_processData = processData;
    LIN_stopTimer();
    LIN_enableRx();
    LIN_setTimerHandler();
}

lin_rx_state_t LIN_Slave_handler(void)  {
    static lin_rx_state_t LIN_rxState = LIN_RX_IDLE;
    static uint8_t rxDataIndex = 0;

    if(LIN_rxInProgress == true){
        if(LIN_timerRunning == false){
            //Timeout
            LIN_rxState = LIN_RX_ERROR;
        }
    }

    switch(LIN_rxState){
        case LIN_RX_IDLE:
            if(EUSART_is_rx_ready() > 0){
                //Start Timer
                LIN_startTimer(READ_TIMEOUT); 
                LIN_rxInProgress = true;
                LIN_rxState = LIN_RX_BREAK;
            }
            break;
        case LIN_RX_BREAK:
            if(EUSART_is_rx_ready() > 0){
                if(LIN_breakCheck() == true){  //Read Break
                    LIN_rxState = LIN_RX_SYNC;
                } else {
                    LIN_rxState = LIN_RX_ERROR;
                }
            }
            break;
        case LIN_RX_SYNC:
            if(EUSART_is_rx_ready() > 0){
                if(EUSART_Read() == 0x55){  //Read sync - discard
                    LIN_rxState = LIN_RX_PID;
                } else {
                    LIN_rxState = LIN_RX_ERROR;
                }
            }
            break;
        case LIN_RX_PID:
            if(EUSART_is_rx_ready() > 0){
                LIN_packet.PID = EUSART_Read();

                //check LIN Parity bits
                if(LIN_checkPID(LIN_packet.PID) == false){
                    LIN_rxState = LIN_RX_ERROR;
                    break;
                }
                LIN_packet.type = LIN_getFromTable(LIN_packet.PID, TYPE);
                if(LIN_packet.type == RECEIVE){
                    LIN_packet.length = LIN_getFromTable(LIN_packet.PID, LENGTH);
                    LIN_rxState = LIN_RX_DATA;
                }
                else{
                    LIN_disableRx();
                    LIN_rxState = LIN_RX_TX_DATA;
                }
            }
            break;
        case LIN_RX_DATA:
            if(EUSART_is_rx_ready() > 0){
                LIN_packet.data[rxDataIndex] = EUSART_Read();
                if(++rxDataIndex >= LIN_packet.length){
                    //received all data bytes
                    rxDataIndex = 0;
                    LIN_rxState = LIN_RX_CHECKSUM;
                }
            }
            break;
        case LIN_RX_CHECKSUM:
            if(EUSART_is_rx_ready() > 0){
                LIN_packet.checksum = EUSART_Read();
                if(LIN_packet.checksum != LIN_getChecksum(LIN_packet.length, LIN_packet.data)) {
                    LIN_rxState = LIN_RX_ERROR;
                }
                else {
                    LIN_rxState = LIN_RX_RDY;
                }
            }
            break;
        case LIN_RX_TX_DATA:
            LIN_queuePacket(LIN_packet.PID); //Send response automatically
            LIN_rxState = LIN_RX_RDY;
        case LIN_RX_RDY:
            LIN_processData();
        case LIN_RX_ERROR:
            LIN_stopTimer();
            rxDataIndex = 0;
            LIN_rxInProgress = false;
            memset(LIN_packet.rawPacket, 0, sizeof(LIN_packet.rawPacket));  //clear receive data
        case LIN_RX_WAIT:
            if(TX1STAbits.TRMT){
                LIN_enableRx();
                LIN_rxState = LIN_RX_IDLE;
            } else {
                LIN_rxState = LIN_RX_WAIT;
            }
            break;
    }
    return LIN_rxState;
}
uint8_t LIN_calcParity(uint8_t CMD){
    lin_pid_t PID;
    PID.rawPID = CMD;

    //Workaround for compiler bug - CAE_MCU8-200:
//    PID.P0 = PID.ID0 ^ PID.ID1 ^ PID.ID2 ^ PID.ID4;
//    PID.P1 = ~(PID.ID1 ^ PID.ID3 ^ PID.ID4 ^ PID.ID5);
    PID.P0 = PID.ID0 ^ PID.ID1;
    PID.P0 = PID.P0 ^ PID.ID2;
    PID.P0 = PID.P0 ^ PID.ID4;
    PID.P1 = PID.ID1 ^ PID.ID3;
    PID.P1 = PID.P1 ^ PID.ID4;
    PID.P1 = PID.P1 ^ PID.ID5;
    PID.P1 = ~PID.P1;
    
    return PID.rawPID;
}

uint8_t LIN_getChecksum(uint8_t length, uint8_t* data){
    uint16_t checksum = 0;
    
    for (uint8_t i = 0; i < length; i++){
        checksum = checksum + *data++;
        if(checksum > 0xFF)
            checksum -= 0xFF;
    }
    checksum = ~checksum;
    
    return (uint8_t)checksum;
}

void LIN_startTimer(uint8_t timeout){
    LIN_timeout = timeout;
    TMR0_WriteTimer(0);
    NOP();
    LIN_timerRunning = true;
}

void LIN_timerHandler(void){

    // callback function
    if (++CountCallBack >= LIN_timeout)
    {
        // ticker function call
        LIN_stopTimer();
    }
}

void LIN_setTimerHandler(void){
    TMR0_SetInterruptHandler(LIN_timerHandler);
}

void LIN_stopTimer(void){
    NOP();
    // reset ticker counter
    CountCallBack = 0;
    LIN_timerRunning = false;
}

void LIN_enableRx(void){
    RC1STAbits.CREN = 1;
    PIE1bits.RCIE = 1;
}

void LIN_disableRx(void){
    RC1STAbits.CREN = 0;
    PIE1bits.RCIE = 0;
}

bool LIN_breakCheck(void){
    
    if((EUSART_Read() == 0x00) && (EUSART_get_last_status().ferr == 1)){
        return true;
    }
    
    return false;
}
/* *****************************************************************************
 End of File
 */
