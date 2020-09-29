/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_lin_vcu.c

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
#include "app_lin_sbc.h"
#include "definitions.h"                // SYS function prototypes
#include "lin_slave.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
typedef enum {
    UNLOCK = 0x00,
    RSSI = 0x01,
    LFRX = 0x02
} lin_v_cmd_t;
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_LIN_VCU_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_LIN_VCU_DATA app_lin_vcuData;

// *****************************************************************************
// *****************************************************************************
// Section: Local Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t V_UNLOCK_Data[1];
uint8_t V_RSSI_Data[1] = {0x10};
uint8_t V_LFRX_Data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

lin_rx_cmd_t linVCUCmdTable[] = {
    //Command, Type, TX/RX Length, Data Address
    {UNLOCK, RECEIVE, 1, V_UNLOCK_Data },
    {RSSI, RECEIVE, 1, V_RSSI_Data },
    {LFRX, TRANSMIT, 8, V_LFRX_Data }
};
#define TABLE_SIZE  (sizeof(linVCUCmdTable)/sizeof(lin_rx_cmd_t))
/* Timeout ms for LIN frame (dependence on BAUDRATE) */

static void processLINVcu(void);

lin_slave_node lin_vcu = {
    .state = LIN_RX_IDLE,
    .rxInProgress = false,
    .timerIsRunning = false,
    .startTimer = TC3_TimerStart,
    .stopTimer = TC3_TimerStop,
    .disableRx = SERCOM1_USART_ReceiverDisable,
    .enableRx = SERCOM1_USART_ReceiverEnable,
    .processData = processLINVcu,
    .rxDataCount = SERCOM1_USART_ReadCountGet,
    .writeData = SERCOM1_USART_Write,
    .readData = SERCOM1_USART_Read,
    .readAbort = SERCOM1_USART_ReadAbort,
    .rxCommand = linVCUCmdTable,
    .rxCommandLength = TABLE_SIZE,
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/void LIN_VCU_SERCOM_USART_CALLBACK( uintptr_t context )
{
    lin_vcu.rxDataIndex++;
    //SERCOM1_USART_Read(&testdata, 1);
}

void LIN_VCU_SERCOM_USART_TX_CALLBACK( uintptr_t context )
{
    lin_vcu.writeFinished = true;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
void processLINVcu(void)
{
    uint8_t cmd;
    uint8_t data[8];

    cmd = LIN_getPacket(&lin_vcu, data);

    switch (cmd) {
        case UNLOCK:
            LED0_Clear();
            break;
        case RSSI:
            LED0_Set();
            break;
        case LFRX:
            break;
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_LIN_VCU_Initialize ( void )

  Remarks:
    See prototype in app_lin_vcu.h.
 */

void APP_LIN_VCU_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_lin_vcuData.state = APP_LIN_VCU_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_LIN_VCU_Tasks ( void )

  Remarks:
    See prototype in app_lin_vcu.h.
 */

void APP_LIN_VCU_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_lin_vcuData.state )
    {
        /* Application's initial state. */
        case APP_LIN_VCU_STATE_INIT:
        {
            bool appInitialized = true;
            SERCOM1_USART_WriteCallbackRegister(LIN_VCU_SERCOM_USART_TX_CALLBACK, 0);

            if (appInitialized)
            {

                app_lin_vcuData.state = APP_LIN_VCU_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_LIN_VCU_STATE_SERVICE_TASKS:
        {
            LIN_handler(&lin_vcu);
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
