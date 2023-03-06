//*****************************************************************************
// Debug utilities
// Apollo3
// Mariusz Lacina, Ambiq, 2023
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "uart_debug.h"


//----------------------------------------------------------------------------//
//  ITM_SWO pin: ITM Serial Wire Output.
//----------------------------------------------------------------------------//
const am_hal_gpio_pincfg_t g_GPIO_ITM_SWO =
{
    .uFuncSel            = AM_HAL_PIN_41_SWO,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};


//----------------------------------------------------------------------------//
//  Print interface tracking variable
//----------------------------------------------------------------------------//
static uint32_t g_ui32PrintInterface = AM_BSP_PRINT_INFC_UART0;


//----------------------------------------------------------------------------//
//  COM_UART pins.
//----------------------------------------------------------------------------//
#define GPIO_COM_UART_TX         22
#define GPIO_COM_UART_RX         23

//----------------------------------------------------------------------------//
//  COM_UART_TX pin: This pin is the COM_UART transmit pin.
//----------------------------------------------------------------------------//
const am_hal_gpio_pincfg_t g_GPIO_COM_UART_TX =
{
    .uFuncSel            = AM_HAL_PIN_22_UART0TX,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};


//----------------------------------------------------------------------------//
//  COM_UART_RX pin: This pin is the COM_UART receive pin.
//----------------------------------------------------------------------------//
const am_hal_gpio_pincfg_t g_GPIO_COM_UART_RX =
{
    .uFuncSel            = AM_HAL_PIN_23_UART0RX
};

//----------------------------------------------------------------------------//
//  GPIO disable configuration
//----------------------------------------------------------------------------//
const am_hal_gpio_pincfg_t g_GPIO_DISABLE =
{
    .uFuncSel       = 3,
    .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE
};


//----------------------------------------------------------------------------//
//  Default UART configuration settings
//----------------------------------------------------------------------------//
static void *g_sCOMUART;

static const am_hal_uart_config_t g_sBspUartConfig =
{
    //
    // Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate = 115200,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                       AM_HAL_UART_RX_FIFO_1_2),

    //
    // The default interface will just use polling instead of buffers.
    //
    .pui8TxBuffer = 0,
    .ui32TxBufferSize = 0,
    .pui8RxBuffer = 0,
    .ui32RxBufferSize = 0,
};

//----------------------------------------------------------------------------//
//  Enable printing over ITM.
//----------------------------------------------------------------------------//
void itm_printf_enable(void)
{
    am_hal_tpiu_config_t TPIUcfg;

    //
    // Set the global print interface.
    //
    g_ui32PrintInterface = AM_BSP_PRINT_INFC_SWO;

    //
    // Enable the ITM interface and the SWO pin.
    //
    am_hal_itm_enable();

    //
    // Enable the ITM and TPIU
    // Set the BAUD clock for 1M
    //
    TPIUcfg.ui32SetItmBaud = AM_HAL_TPIU_BAUD_1M;
    am_hal_tpiu_enable(&TPIUcfg);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_ITM_SWO, g_GPIO_ITM_SWO);

    //
    // Attach the ITM to the STDIO driver.
    //
    am_util_stdio_printf_init(am_hal_itm_print);
}


//----------------------------------------------------------------------------//
//  This function is used for printing a string via the UART, which for some
//! MCU devices may be multi-module.
//----------------------------------------------------------------------------//
void uart_string_print(char *pcString)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    //
    // Measure the length of the string.
    //
    while (pcString[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) pcString,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    am_hal_uart_transfer(g_sCOMUART, &sUartWrite);
   
} 


//----------------------------------------------------------------------------//
// Initialize and configure the UART
//----------------------------------------------------------------------------//
void uart_printf_enable(void)
{
    //
    // Save the information that we're using the UART for printing.
    //
    g_ui32PrintInterface = AM_BSP_PRINT_INFC_UART0;

    //
    // Initialize, power up, and configure the communication UART. Use the
    // custom configuration if it was provided. Otherwise, just use the default
    // configuration.
    //
    am_hal_uart_initialize(AM_BSP_UART_PRINT_INST, &g_sCOMUART);
    am_hal_uart_power_control(g_sCOMUART, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(g_sCOMUART, &g_sBspUartConfig);

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pinconfig(GPIO_COM_UART_TX, g_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(GPIO_COM_UART_RX, g_GPIO_COM_UART_RX);

    //
    // Register the BSP print function to the STDIO driver.
    //
    am_util_stdio_printf_init(uart_string_print);
} 


//----------------------------------------------------------------------------//
// Disable the UART
//----------------------------------------------------------------------------//
void uart_printf_disable(void)
{
    //
    // Make sure the UART has finished sending everything it's going to send.
    //
    am_hal_uart_tx_flush(g_sCOMUART);

    //
    // Detach the UART from the stdio driver.
    //
    am_util_stdio_printf_init(0);

    //
    // Power down the UART, and surrender the handle.
    //
    am_hal_uart_power_control(g_sCOMUART, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_uart_deinitialize(g_sCOMUART);

    //
    // Disable the UART pins.
    //
    am_hal_gpio_pinconfig(GPIO_COM_UART_TX, g_GPIO_DISABLE);
    am_hal_gpio_pinconfig(GPIO_COM_UART_RX, g_GPIO_DISABLE);

} 



//----------------------------------------------------------------------------//
// This function enables TPIU registers for debug printf messages and enables
// ITM GPIO pin to SWO mode. This function should be called after reset and
// after waking up from deep sleep.
//----------------------------------------------------------------------------//
void debug_printf_enable(void)
{
    if (g_ui32PrintInterface == AM_BSP_PRINT_INFC_SWO)
    {
        itm_printf_enable();
    }
    else if (g_ui32PrintInterface == AM_BSP_PRINT_INFC_UART0)
    {
        uart_printf_enable();
    }
}

//----------------------------------------------------------------------------//
//  This function disables TPIU registers for debug printf messages and
// enables ITM GPIO pin to GPIO mode and prepares the MCU to go to deep sleep.
//----------------------------------------------------------------------------//
void debug_printf_disable(void)
{
    if (g_ui32PrintInterface == AM_BSP_PRINT_INFC_SWO)
    {
        itm_printf_enable();
    }
    else if (g_ui32PrintInterface == AM_BSP_PRINT_INFC_UART0)
    {
        uart_printf_disable();
    }
}






