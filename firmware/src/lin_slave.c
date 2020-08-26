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

/**
 * 
 * @param slave
 * @param tableLength
 * @param command
 * @param processData
 */
void LIN_init(lin_slave_node *slave)
{
    slave->stopTimer();
    slave->enableRx();
    slave->timerRunning = false;
    //slave->processData = processData;
}

void LIN_queuePacket(lin_slave_node *slave)
{
    const lin_rx_cmd_t* tempSchedule = slave->rxCommand;    //copy table pointer so we can modify it
    uint8_t cmd  = slave->pkg.PID & 0x3F;;
    
    for (uint8_t i = 0; i < slave->rxCommandLength; i++){
        if (cmd == tempSchedule->cmd){
            break;
        }
        tempSchedule++;    //go to next entry
    }
    
    slave->pkg.type = tempSchedule->type;
    slave->pkg.length = tempSchedule->length;
    
    //Build Packet - User defined data
    //add data
    memcpy(slave->pkg.data, tempSchedule->data, slave->pkg.length);
    
    //Add Checksum
    slave->pkg.data[slave->pkg.length] = LIN_getChecksum(slave);
    slave->writeData(slave->pkg.data, slave->pkg.length);
}

lin_rx_state_t LIN_handler(lin_slave_node *slave)
{
    static lin_rx_state_t LIN_rxState = LIN_RX_IDLE;

    if (slave->rxInProgress == true){
        if(slave->timerIsRunning() == false){
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
                slave->startTimer();
                slave->state = LIN_RX_SYNC;
                slave->rxDataIndex = 0;
            }
            break;
        case LIN_RX_SYNC:
            // No Sync Interrupt for SAM Device
            slave->state = LIN_RX_PID;
            break;
        case LIN_RX_PID:
            if (slave->rxDataIndex >= 1) {
                //check LIN Parity bits
                if (LIN_checkPID(slave) == false){
                    LIN_rxState = LIN_RX_ERROR;
                    break;
                }
                slave->pkg.type = LIN_getFromTable(slave, TYPE);
                if (slave->pkg.type == RECEIVE) {
                    slave->pkg.length = LIN_getFromTable(slave, LENGTH);
                    slave->state = LIN_RX_DATA;
                }
                else{
                    slave->disableRx();
                    slave->state = LIN_RX_TX_DATA;
                }
            }
            break;
        case LIN_RX_DATA:
        case LIN_RX_CHECKSUM:
            if (slave->rxDataIndex >= slave->pkg.length) {
                if(LIN_getChecksum(slave)) {
                    slave->state = LIN_RX_ERROR;
                }
                else {
                    slave->state = LIN_RX_RDY;
                }
            }
            break;
        case LIN_RX_TX_DATA:
            LIN_queuePacket(slave); //Send response automatically
            LIN_rxState = LIN_RX_RDY;
        case LIN_RX_RDY:
            slave->processData();
        case LIN_RX_ERROR:
            slave->stopTimer();
            memset(&slave->pkg, 0, sizeof(lin_slave_node));  //clear receive data
        case LIN_RX_WAIT:
            if (1) {
                slave->enableRx();
                slave->state = LIN_RX_IDLE;
            } else {
                slave->state = LIN_RX_WAIT;
            }
            break;
    }
    return LIN_rxState;
}

uint8_t LIN_getPacket(lin_slave_node *slave, uint8_t* data)
{
    uint8_t cmd = slave->pkg.PID & 0x3F;
    
    memcpy(data, slave->pkg.data, sizeof(slave->pkg.data));
    memset(slave->pkg.rawPacket, 0, sizeof(slave->pkg.rawPacket));
    
    return cmd;
}

uint8_t LIN_getFromTable(lin_slave_node *slave, lin_sch_param_t param)
{
    const lin_rx_cmd_t* rxCommand = slave->rxCommand;    //copy table pointer so we can modify it

    uint8_t cmd = slave->pkg.PID;
    cmd &= 0x3F;    //clear possible parity bits
    //check table
    for (uint8_t length = 0; length < slave->rxCommandLength; length++){
        if (cmd == rxCommand->cmd){
            break;
        }
        rxCommand++;    //go to next entry

        if (length == (slave->rxCommandLength-1)) {
            return ERROR;   //command not in schedule table
        }
    }
    
    switch (param) {
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

bool LIN_checkPID(lin_slave_node *slave)
{
    if (LIN_getFromTable(slave, TYPE) == ERROR)
        return false;   //PID not in schedule table
    
    if(slave->pkg.PID == LIN_calcParity(slave->pkg.PID & 0x3F))
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

uint8_t LIN_getChecksum(lin_slave_node *slave)
{
    uint16_t checksum = 0;
    uint8_t *data = slave->pkg.data;
    
    for (uint8_t i = 0; i < slave->pkg.length; i++){
        checksum = checksum + *data++;
        if(checksum > 0xFF)
            checksum -= 0xFF;
    }
    checksum = ~checksum;
    
    return (uint8_t)checksum;
}
