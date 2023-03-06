//*****************************************************************************
// Debug utilities
// Apollo3
// Mariusz Lacina, Ambiq, 2023
//*****************************************************************************

#ifndef UART_DEBUG_H
#define UART_DEBUG_H


//*****************************************************************************
//
// Print interface type
//
//*****************************************************************************
#define AM_BSP_PRINT_INFC_NONE              0
#define AM_BSP_PRINT_INFC_SWO               1
#define AM_BSP_PRINT_INFC_UART0             2

void debug_printf_enable(void);
void debug_printf_disable(void);
void uart_string_print(char *pcString);
#endif


