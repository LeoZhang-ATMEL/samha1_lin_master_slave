/**
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

lin_m_state_t LIN_M_handler(lin_master_node *master)
{
    switch (master->state) {
        case LIN_M_IDLE:
            if (master->writeReady == true) {
                master->writeReady = false;
                master->abortRx();
                master->disableRx();
                master->writeReady = false;
                master->sendBreak(true);
                master->state = LIN_M_TX_BREAK;
            } else {
                //No Transmission to send
            }
            break;
        case LIN_M_TX_BREAK:
            //Transmission currently in progress.
            if (master->writeReady == false) {
                break;
            }
            master->sendBreak(false);
            master->writeData(master->pkg.rawPacket, master->pkg.length + 3); /* Plus SYNC, PID and CRC for 3 bytes */
            break;
        case LIN_M_TX_IP:
            //Transmission currently in progress.
            if (master->writeReady == false) {
                break;
            }
            //Packet transmitted
            if (master->pkg.type == RECEIVE)
            {
                //Need data returned?
                //LIN_startTimer(LIN_rxPacket.timeout);
                master->enableRx();
                master->readReady = false;
                master->readData(master->pkg.data, master->pkg.length);
                master->state = LIN_M_RX_IP;
                // Start Receive, register Callback
            } else {
                master->state = LIN_M_IDLE;
            }

            break;
        case LIN_M_RX_IP:
            //Receiving Packet within window
            if (master->readReady == false) {
                // Need apply timeout
                master->state = LIN_M_IDLE;
                memset(master->pkg.rawPacket, 0, sizeof(master->pkg.rawPacket));
                break;
            } else {
                //All data received and verified
                master->disableRx();
                master->state = LIN_M_RX_RDY;
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

uint8_t LIN_M_getPacket(lin_master_node* master, uint8_t* data)
{
    uint8_t cmd = master->pkg.PID & 0x3F;
    
    memcpy(data, master->pkg.data, sizeof(master->pkg.data));
    memset(master->pkg.rawPacket, 0, sizeof(master->pkg.rawPacket));
    
    return cmd;
}

