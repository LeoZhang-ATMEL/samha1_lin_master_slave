/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_lin_mc.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "definitions.h"                // SYS function prototypes
#include "app_lin_mc.h"
#include "lin_master.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_LIN_MC_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_LIN_MC_DATA app_lin_mcData;

typedef enum {
    UNLOCK = 0x00,
    RSSI = 0x01,
    LFRX = 0x02
} lin_m_cmd_t ;

uint8_t LM_UNLOCK_Data[1];
uint8_t LM_RSSI_Data[1];
uint8_t LM_LFRX_Data[8];

const lin_cmd_packet_t scheduleTable[] = {
    //Command, Type, TX/RX Length, Timeout, Period, Data Address
    {UNLOCK, TRANSMIT, 1, 0, 100, LM_UNLOCK_Data },
    {RSSI, TRANSMIT, 1, 0, 20, LM_RSSI_Data },
    {LFRX, RECEIVE, 8, 100, 100, LM_LFRX_Data }
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
void processLINMC(void);
void sendLINMCBreak(bool state);

/* TODO:  Add any necessary local functions.
*/

lin_master_node lin_mc = {
    .state = LIN_M_IDLE,
    .rxInProgress = false,
    .timerIsRunning = false,
    .enablePeriodTx = true,
    .LIN_txReady = false, /* must be false at init */
    .startTimer = TC3_TimerStart,
    .stopTimer = TC3_TimerStop,
    .sendBreak = sendLINMCBreak,
    .disableRx = SERCOM0_USART_ReceiverDisable,
    .enableRx = SERCOM0_USART_ReceiverEnable,
    .abortRx = SERCOM0_USART_ReadAbort,
    .processData = processLINMC,
    .rxDataCount = SERCOM0_USART_ReadCountGet,
    .writeData = SERCOM0_USART_Write,
    .readData = SERCOM0_USART_Read,
    .timerCallBack = 0,
    .schedule = scheduleTable,
    .scheduleLength = 3,
};
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
void processLINMC(void)
{
    uint8_t cmd;
    uint8_t data[8];

    cmd = LIN_M_getPacket(&lin_mc, data);

    switch (cmd) {
        case UNLOCK:
            if (data[0] == 0x01) {
            } else {
            }
            break;
        case RSSI:
            break;
        case LFRX:
            break;
        default:
            break;
    }
}

void sendLINMCBreak(bool state)
{
    /* Disable the USART before configurations */
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA &= ~SERCOM_USART_INT_CTRLA_ENABLE_Msk;
    while(SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY);
    SERCOM0_REGS->USART_INT.SERCOM_BAUD = (state ? BAUD_REG_BREAK : BAUD_REG_NORMAL);
    /* Enable the USART after the configurations */
    SERCOM0_REGS->USART_INT.SERCOM_CTRLA |= SERCOM_USART_INT_CTRLA_ENABLE_Msk;
    while(SERCOM0_REGS->USART_INT.SERCOM_SYNCBUSY);

    uint8_t breakdata = 0x00;
    if (state) {
        SERCOM0_USART_Write(&breakdata, 1);
        SYSTICK_TimerStart();
    } else {
        SYSTICK_TimerStop();
    }
}

void LIN_MC_SERCOM_USART_CALLBACK( uintptr_t context )
{
}

void LIN_MC_SERCOM_USART_TX_CALLBACK( uintptr_t context )
{
    lin_mc.txFinished = true;
}

void LIN_MC_TIMER_CALLBACK( uintptr_t context )
{
    lin_mc.txBreakFinished = true;
}

/*******************************************************************************
  Function:
    void APP_LIN_MC_Initialize ( void )

  Remarks:
    See prototype in app_lin_mc.h.
 */

void APP_LIN_MC_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_lin_mcData.state = APP_LIN_MC_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_LIN_MC_Tasks ( void )

  Remarks:
    See prototype in app_lin_mc.h.
 */

void APP_LIN_MC_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_lin_mcData.state )
    {
        /* Application's initial state. */
        case APP_LIN_MC_STATE_INIT:
        {
            bool appInitialized = true;
            SERCOM0_USART_WriteCallbackRegister(LIN_MC_SERCOM_USART_TX_CALLBACK, 0);
            SYSTICK_TimerPeriodSet(32544 + 1000); /* 48000 * 0.675 and some buffer */
            SYSTICK_TimerCallbackSet(LIN_MC_TIMER_CALLBACK, 0);

            if (appInitialized) {
                app_lin_mcData.state = APP_LIN_MC_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_LIN_MC_STATE_SERVICE_TASKS:
        {
            LIN_M_handler(&lin_mc);
            break;
        }

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


/*******************************************************************************
 End of File
 */
