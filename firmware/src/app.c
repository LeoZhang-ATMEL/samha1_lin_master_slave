/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

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

#include "app.h"
#include "definitions.h"

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
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

volatile bool lin_sbc_recv = false;
uint8_t lin_sbc_data;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
void LINSBC_USART_CALLBACK( uintptr_t context )
{
    lin_sbc_recv = true;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;


    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    SERCOM5_USART_ReadCallbackRegister(LINSBC_USART_CALLBACK, 0);
    SYS_CMD_PRINT("Antolin Lotus Start\r\n");
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            appData.state = APP_STATE_LIN_SBC_WAIT_DATA;
            break;
        }

        /* TODO: implement your application state machine.*/
        case APP_STATE_LIN_SBC_WAIT_DATA:
        {
            lin_sbc_recv = false;
            SERCOM5_USART_Read(&lin_sbc_data, 1);
            appData.state = APP_STATE_LIN_SBC_RECV_DATA;
            break;
        }

        case APP_STATE_LIN_SBC_RECV_DATA:
        {
            if (lin_sbc_recv == true) {
                lin_sbc_recv = false;
                appData.state = APP_STATE_LIN_SBC_PRINT_DATA;
            }
            break;
        }

        case APP_STATE_LIN_SBC_PRINT_DATA:
        {
            SYS_CMD_PRINT("%x", lin_sbc_data);
            appData.state = APP_STATE_LIN_SBC_WAIT_DATA;
            break;
        }

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
