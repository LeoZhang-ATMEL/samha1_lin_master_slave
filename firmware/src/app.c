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
#include <string.h>
#include "definitions.h"                // SYS function prototypes
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
    This structure should be initialized by the APP_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
extern void LIN_M_timerHandler(lin_master_node* master);
extern lin_master_node lin_mc;

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


/* TODO:  Add any necessary local functions.
*/

void LIN_TimerCallBack(TC_TIMER_STATUS status, uintptr_t context)
{
    LIN_M_timerHandler(&lin_mc);
}
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
    TC3_TimerCallbackRegister(LIN_TimerCallBack, 0);
    TC3_TimerStart();
#if 0
    uint8_t swVer[NVMCTRL_RWWEEPROM_PAGESIZE] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11};
    while(NVMCTRL_IsBusy()); // workaround for revE
    NVMCTRL_RWWEEPROM_PageWrite((uint32_t *)swVer, NVMCTRL_RWWEEPROM_START_ADDRESS); // workaround for revE
    while(NVMCTRL_IsBusy());
    NVMCTRL_RWWEEPROM_RowErase(NVMCTRL_RWWEEPROM_START_ADDRESS);
    while (NVMCTRL_IsBusy()) {
    }

    NVMCTRL_RWWEEPROM_PageWrite((uint32_t *)swVer, NVMCTRL_RWWEEPROM_START_ADDRESS);
    while (NVMCTRL_IsBusy()) {
    }

    extern uint8_t LFRX_Data[8];
    NVMCTRL_RWWEEPROM_Read((uint32_t *)LFRX_Data, 8, NVMCTRL_RWWEEPROM_START_ADDRESS);
#endif
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
