/**]}+
  LIN Master Driver
	
  Company:
    Microchip Technology Inc.

  File Name:
    lin_master.c

  Summary:
    LIN Master Driver

  Description:
    This source file provides the driver for LIN master nodes

 */

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
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
#include "lin_master.h"
#include "config/default/peripheral/sercom/usart/plib_sercom0_usart.h"

extern bool SERCOM0_USART_WriteIsBusy( void );
lin_m_state_t LIN_M_handler(lin_master_node *master)
{
    switch (master->state) {
        case LIN_M_IDLE:
            if (master->LIN_txReady == true) {
                master->LIN_txReady = false;
                master->abortRx();
                master->disableRx();
                master->txFinished = false;
                master->txBreakFinished = false;
                master->sendBreak(true);
                master->state = LIN_M_TX_BREAK;
            } else {
                //No Transmission to send
            }
            break;
        case LIN_M_TX_BREAK:
            //Transmission currently in progress.
            if (master->txBreakFinished == false || master->txBreakFinished == false) {
                break;
            }
            master->txFinished = false;
            master->sendBreak(false);
            if (master->LIN_packet.type == RECEIVE) {
                master->writeData(master->LIN_packet.rawPacket, 2); /* Plus SYNC, PID, 2 bytes */
            } else {
                master->writeData(master->LIN_packet.rawPacket, master->LIN_packet.length + 3); /* Plus SYNC, PID and CRC for 3 bytes */
            }
            master->state = LIN_M_TX_IP;
            break;
        case LIN_M_TX_IP:
            //Transmission currently in progress.
            if (master->txFinished == false) {
                break;
            }
            //Packet transmitted
            if (master->LIN_packet.type == RECEIVE) {
                //Need data returned?
                //LIN_startTimer(LIN_rxPacket.timeout);
                master->enableRx();
                master->readReady = false;
                // Start Receive, register Callback
                master->LIN_timerRunning = true;
                master->readData(master->LIN_rxPacket.data, master->LIN_rxPacket.rxLength);
                master->state = LIN_M_RX_IP;
            } else {
                master->state = LIN_M_IDLE;
            }

            break;
        case LIN_M_RX_IP:
            //Receiving Packet finished
            if (master->readReady == true) {
                master->disableRx();
                master->state = LIN_M_RX_RDY;
                break;
            }
            if (master->LIN_timerRunning == false) {
                // Need apply timeout
                master->disableRx();
                master->state = LIN_M_IDLE;
                memset(master->LIN_rxPacket.rawPacket, 0, sizeof(master->LIN_rxPacket.rawPacket));
                break;
            }
            break;
        case LIN_M_RX_RDY:
            //Received Transmission
            master->processData();  //LIN_processData();
            master->state = LIN_M_IDLE;
            break;
    }
    return master->state;
}

static void LIN_queuePacket(lin_master_node* master, uint8_t cmd, uint8_t* data)
{
    //copy table pointer so we can modify it
    const lin_cmd_packet_t* tempSchedule = master->schedule;

    for(uint8_t i = 0; i < master->scheduleLength; i++){
        if(cmd == tempSchedule->cmd){
            break;
        }
        tempSchedule++;    //go to next entry
    }
    
    //clear previous data
    memset(master->LIN_packet.rawPacket, 0, sizeof(master->LIN_packet.rawPacket));

    master->LIN_packet.type = tempSchedule->type;
    //Add SYNC and ID
    master->LIN_packet.SYNC = 0x55;
    master->LIN_packet.PID = LIN_calcParity(tempSchedule->cmd);

    if (tempSchedule->type == TRANSMIT) {
        //Build Packet - User defined data
        //add data
        if (tempSchedule->length > 0) {
            master->LIN_packet.length = tempSchedule->length;
            memcpy(master->LIN_packet.data, data, tempSchedule->length);
        } else {
            master->LIN_packet.length = 1; //send dummy byte for checksum
            master->LIN_packet.data[0] = 0xAA;
        }

        //Add Checksum
        master->LIN_packet.data[master->LIN_packet.length] = 
                LIN_getChecksum(master->LIN_packet.data, master->LIN_packet.length);

    } else { //Rx packet
        master->LIN_rxPacket.rxLength = tempSchedule->length; //data length for rx data processing
        master->LIN_rxPacket.cmd = tempSchedule->cmd; //command for rx data processing
        master->LIN_rxPacket.timeout = tempSchedule->timeout;
    }
    
    master->LIN_txReady = true;
}

uint8_t LIN_M_getPacket(lin_master_node* master, uint8_t* data)
{
    uint8_t cmd = master->LIN_rxPacket.cmd & 0x3F;
    
    memcpy(data, master->LIN_rxPacket.data, sizeof(master->LIN_rxPacket.data));
    memset(master->LIN_rxPacket.rawPacket, 0, sizeof(master->LIN_rxPacket.rawPacket));
    
    return cmd;
}

static void LIN_M_sendPeriodicTx(lin_master_node* master)
{
    const lin_cmd_packet_t* periodicTx;    //copy table pointer so we can modify it
    
    master->LIN_periodCallBack = 0;

    periodicTx = master->schedule + master->scheduleIndex;
    
    if (periodicTx->period > 0) {
        LIN_queuePacket(master, periodicTx->cmd, periodicTx->data);
    }
    
    do{ //Go to next valid periodic command
        if(++master->scheduleIndex >= master->scheduleLength){
            master->scheduleIndex = 0;
        }
        periodicTx = master->schedule + master->scheduleIndex;
    } while(periodicTx->period == 0);
    
    master->LIN_period = periodicTx->period;
}

/**
 * Callback handler for 1ms period
 * @param master
 */
void LIN_M_timerHandler(lin_master_node* master)
{
    if (master->LIN_timerRunning == true) {
        if ((++(master->rxTimeout)) >= master->LIN_rxPacket.timeout) {
            master->LIN_timerRunning = false;
            master->rxTimeout = 0;
        }
    }
    
    if (master->enablePeriodTx == true) {
        if (++master->LIN_periodCallBack >= master->LIN_period) {
            LIN_M_sendPeriodicTx(master);
        }
    }
}