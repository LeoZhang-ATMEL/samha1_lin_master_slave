/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_lin_sbc.c

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
#include "app_lin_sbc.h"
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
}lin_cmd_t;

#define APP_LIN_SBC_TIMEOUT 15
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_LIN_SBC_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/
APP_LIN_SBC_DATA app_lin_sbcData;

// *****************************************************************************
// *****************************************************************************
// Section: Local Data Definitions
// *****************************************************************************
// *****************************************************************************

uint8_t UNLOCK_Data[1];
uint8_t RSSI_Data[1];
uint8_t LFRX_Data[8];

lin_rx_cmd_t linSBCCmdTable[] = {
    //Command, Type, TX/RX Length, Data Address
    {UNLOCK, RECEIVE, 1, UNLOCK_Data },
    {RSSI, RECEIVE, 1, RSSI_Data },
    {LFRX, TRANSMIT, 8, LFRX_Data }
};
#define TABLE_SIZE  (sizeof(linSBCCmdTable)/sizeof(lin_rx_cmd_t))
/* Timeout ms for LIN frame (dependence on BAUDRATE) */


static void APP_LIN_SBC_DisableRx(void);
static void APP_LIN_SBC_EnableRx(void);
static void processLINSbc(void);

lin_slave_node lin_sbc = {
    .state = LIN_RX_IDLE,
    .rxInProgress = false,
    .timerIsRunning = false,
    .startTimer = TC3_TimerStart,
    .stopTimer = TC3_TimerStop,
    .disableRx = APP_LIN_SBC_DisableRx,
    .enableRx = APP_LIN_SBC_EnableRx,
    .processData = processLINSbc,
    .rxDataCount = SERCOM5_USART_ReadCountGet,
    .rxCommand = linSBCCmdTable,
    .rxCommandLength = TABLE_SIZE,
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
static uint8_t testdata;
/* TODO:  Add any necessary callback functions.
*/
void LIN_SBC_SERCOM_USART_CALLBACK( uintptr_t context )
{
    lin_sbc.rxDataIndex++;
    SYS_CONSOLE_PRINT("%02X", testdata);
    SERCOM5_USART_Read(&testdata, 1);
}

void TC3_TIMER_CALLBACK(TC_TIMER_STATUS status, uintptr_t context)
{
    if (++app_lin_sbcData.countCallBack >= APP_LIN_SBC_TIMEOUT) {
        // reset ticker counter
        app_lin_sbcData.countCallBack = 0;
        lin_sbc.timerRunning = false;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
void processLINSbc(void)
{
    uint8_t cmd;
    uint8_t data[8];

    cmd = LIN_getPacket(&lin_sbc, data);

    switch (cmd) {
        case UNLOCK:
            if (data[0] == 0x01) {
                LED0_Clear();
            } else {
                LED0_Set();
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

static void APP_LIN_SBC_DisableRx(void)
{
    SERCOM5_REGS->USART_INT.SERCOM_CTRLB &= ~SERCOM_USART_INT_CTRLB_RXEN_Msk;
    
    /* Wait for sync */
    while(SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY);
}

static void APP_LIN_SBC_EnableRx(void)
{
    SERCOM5_USART_Abort_Read();
    SERCOM5_USART_Read(lin_sbc.pkg.rawPacket, 9);
    SERCOM5_REGS->USART_INT.SERCOM_CTRLB |= SERCOM_USART_INT_CTRLB_RXEN_Msk;
    
    /* Wait for sync */
    while(SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_LIN_SBC_Initialize ( void )

  Remarks:
    See prototype in app_lin_sbc.h.
 */

void APP_LIN_SBC_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_lin_sbcData.state = APP_LIN_SBC_STATE_INIT;

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    //SERCOM5_USART_Read(&testdata, 1);
    //TC3_TimerCallbackRegister(TC3_TIMER_CALLBACK, 0);
    //SERCOM5_USART_ReadCallbackRegister(LIN_SBC_SERCOM_USART_CALLBACK, 0);
    //APP_LIN_SBC_EnableRx();
}


/******************************************************************************
  Function:
    void APP_LIN_SBC_Tasks ( void )

  Remarks:
    See prototype in app_lin_sbc.h.
 */

void APP_LIN_SBC_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_lin_sbcData.state )
    {
        /* Application's initial state. */
        case APP_LIN_SBC_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                app_lin_sbcData.state = APP_LIN_SBC_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_LIN_SBC_STATE_SERVICE_TASKS:
        {
            LIN_handler(&lin_sbc);
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
