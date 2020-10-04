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
    slave->enableRx();
    slave->timerRunning = false;
    //slave->processData = processData;
}

static void LIN_queuePacket(lin_slave_node *slave)
{
    const lin_rx_cmd_t* tempSchedule = slave->rxCommand;    //copy table pointer so we can modify it
    uint8_t cmd  = slave->pkg.PID & 0x3F;
    
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
    slave->pkg.data[slave->pkg.length] = LIN_getChecksum(slave->pkg.rawPacket, slave->pkg.length);
    slave->writeData(slave->pkg.data, slave->pkg.length + 1);
}

void LIN_handler(lin_slave_node *slave)
{
    if (slave->rxInProgress == true) {
        if(slave->timerIsRunning() == false){
            //Timeout
            slave->state = LIN_RX_ERROR;
        }
    }

    switch (slave->state) {
        case LIN_RX_IDLE:
            slave->readAbort();
            slave->readData(slave->pkg.rawPacket + 1, 10); /* 1 PID + 8 data + 1 CRC */
            slave->enableRx();
            slave->state = LIN_RX_PID;
            break;
        case LIN_RX_BREAK:
            slave->breakReceived = true; /* Ignore Break interrupt */
            if (slave->breakReceived) {
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
            if (slave->rxDataCount() >= 1) {
                //check LIN Parity bits
                if (LIN_checkPID(slave) == false){
                    slave->state = LIN_RX_ERROR;
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
            if (slave->rxDataCount() >= slave->pkg.length + 2) {
                if (LIN_getChecksum(slave->pkg.data, slave->pkg.length) != slave->pkg.rawPacket[slave->pkg.length + 2]) {
                    slave->state = LIN_RX_ERROR;
                }
                else {
                    slave->state = LIN_RX_RDY;
                }
            }
            break;
        case LIN_RX_TX_DATA:
            slave->writeFinished = false;
            LIN_queuePacket(slave); //Send response automatically
            slave->state = LIN_RX_TX_READY;
        case LIN_RX_TX_READY:
            if (slave->writeFinished) {
                slave->state = LIN_RX_RDY;
            }
            break;
        case LIN_RX_RDY:
            slave->processData();
        case LIN_RX_ERROR:
            memset(&slave->pkg, 0, sizeof(lin_packet_t));  //clear receive data
        case LIN_RX_WAIT:
            slave->state = LIN_RX_IDLE;
            break;
    }
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
