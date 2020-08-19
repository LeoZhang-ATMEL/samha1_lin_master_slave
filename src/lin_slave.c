/**
  LIN Slave Driver
	
  Company:
    Microchip Technology Inc.

  File Name:
    lin_slave.c

  Summary:
    LIN Slave Driver

  Description:
    This source file provides the driver for LIN slave nodes

 */

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "lin_slave.h"
#include "../eusart.h"
#include "../tmr0.h"

#define READ_TIMEOUT    15  //ms

static void (*LIN_processData)(void);

lin_packet_t LIN_packet;
bool LIN_rxInProgress = false;
const lin_rx_cmd_t* LIN_rxCommand;
uint8_t LIN_rxCommandLength;

static uint8_t LIN_timeout = 10; //TODO: Make dependent on Baudrate
static bool LIN_timerRunning = false;
static volatile uint8_t CountCallBack = 0;

/**
 * 
 * @param slave
 * @param tableLength
 * @param command
 * @param processData
 */
void LIN_init(lin_slave_node *slave){
    //slave->rxCommand = command;
    //slave->rxCommandLength = tableLength;
    slave->stopTimer();
    slave->enableRx()
    LIN_setTimerHandler();
    slave->rxInProgress = false;
    slave->timerRunning = false;
    //slave->processData = processData;
}

void LIN_queuePacket(uint8_t cmd){
    const lin_rx_cmd_t* tempSchedule = LIN_rxCommand;    //copy table pointer so we can modify it
    
    cmd &= 0x3F;    //clear possible parity bits
    for(uint8_t i = 0; i < LIN_rxCommandLength; i++){
        if(cmd == tempSchedule->cmd){
            break;
        }
        tempSchedule++;    //go to next entry
    }
    
    LIN_packet.type = tempSchedule->type;
    LIN_packet.length = tempSchedule->length;
    
    //Build Packet - User defined data
    //add data
    memcpy(LIN_packet.data, tempSchedule->data, LIN_packet.length);
    
    //Add Checksum
    LIN_packet.checksum = LIN_getChecksum(LIN_packet.length, LIN_packet.data);
    LIN_sendPacket(LIN_packet.length, LIN_packet.data);

    
}

lin_rx_state_t LIN_handler(lin_slave_node *slave){
    static lin_rx_state_t LIN_rxState = LIN_RX_IDLE;
    static uint8_t rxDataIndex = 0;

    if (slave->rxInProgress == true){
        if(slave->timerRunning == false){
            //Timeout
            slave->state = LIN_RX_ERROR;
        }
    }

    switch (slave->state) {
        case LIN_RX_IDLE:
            slave->enableRx();
            slave->state = LIN_RX_BREAK;
            break;
        case LIN_RX_BREAK:
            if (slave->breakReceived){
                //Start Timer
                LIN_startTimer(READ_TIMEOUT); 
                LIN_rxInProgress = true;
                slave->state = LIN_RX_SYNC;
            }
            break;
        case LIN_RX_SYNC:
            // No Sync Interrupt for SAM Device
            slave->state = LIN_RX_PID;
            break;
        case LIN_RX_PID:
            //check LIN Parity bits
            if(LIN_checkPID(slave) == false){
                LIN_rxState = LIN_RX_ERROR;
                break;
            }
            slave->pkg.type = LIN_getFromTable(slave);
            if(slave->pkg.type == RECEIVE){
                slave->pkg.length = LIN_getFromTable(LIN_packet.PID, LENGTH);
                slave->state = LIN_RX_DATA;
            }
            else{
                LIN_disableRx();
                slave->state = LIN_RX_TX_DATA;
            }
            break;
        case LIN_RX_DATA:
            if (slave->rxCount() >= slave->pkg.length){
                //received all data bytes
                slave->state = LIN_RX_CHECKSUM;
            }
            break;
        case LIN_RX_CHECKSUM:
            if (slave->rxCount() >= slave->pkg.length) {
                if(LIN_getChecksum(slave)) {
                    slave.state = LIN_RX_ERROR;
                }
                else {
                    slave.state = LIN_RX_RDY;
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
            slave->CountCallBack = 0;
            LIN_timerRunning = false
            rxDataIndex = 0;
            LIN_rxInProgress = false;
            memset(slave->pkg, 0, sizeof(slave->pkg));  //clear receive data
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

void LIN_sendPacket(uint8_t length, uint8_t* data){

    //Write data    
    for(uint8_t i = 0; i < length; i++){
        EUSART_Write(*(data + i));
    }
    //Add Checksum
    EUSART_Write(LIN_getChecksum(length, data));
}

uint8_t LIN_getPacket(uint8_t* data){
    uint8_t cmd = LIN_packet.PID & 0x3F;
    
    memcpy(data, LIN_packet.data, sizeof(LIN_packet.data));
    memset(LIN_packet.rawPacket, 0, sizeof(LIN_packet.rawPacket));
    
    return cmd;
}

uint8_t LIN_getFromTable(uint8_t cmd, lin_sch_param_t param){
    const lin_rx_cmd_t* rxCommand = LIN_rxCommand;    //copy table pointer so we can modify it
    
    cmd &= 0x3F;    //clear possible parity bits
    //check table
    for(uint8_t length = 0; length < LIN_rxCommandLength; length++){
        if(cmd == rxCommand->cmd){
            break;
        }
        rxCommand++;    //go to next entry

        if(length == (LIN_rxCommandLength-1)){
            return ERROR;   //command not in schedule table
        }
    }
    
    switch(param){
        case CMD:
            return rxCommand->cmd;
        case TYPE:
            return rxCommand->type;
        case LENGTH:
            return rxCommand->length;
        default:
            break;
    }
    
    return ERROR;
}

bool LIN_checkPID(uint8_t pid){
    if(LIN_getFromTable(pid, TYPE) == ERROR)
        return false;   //PID not in schedule table
    
    if(pid == LIN_calcParity(pid & 0x3F))
        return true;  
    
    return false; //Parity Error

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

