//*****************************************************************************
//
//! @file am1815_time.c
//!
//! @brief Example using the AM1815.
//!
//! Example that sets the initial time on the AM1815, configures the
//! countdown timer for 500ms, which is used to wake up Apollo MCU. 
//! Once Apollo is awake, the app reads and prints the time from the AM1815 
//! on the ITM port at 1MBaud.
//! CLKOUT is enabled in this example.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2019, Ambiq Micro
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices.h"
#include "am_util.h"











#define TIME_SET        0x0
#define TIME_READ       0x1
#define TIME_DISPLAY    0x2
#define APP_SLEEP       0x4

//*****************************************************************************
//
// Arrays for displaying time & date.
//
//*****************************************************************************
char *g_pcWeekday[] =
{
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday"
};
char *g_pcMonth[] =
{
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December"
};

//*****************************************************************************
//
// Structure in the AMx8x5 devices source for holding the time.
//
//*****************************************************************************
extern am_devices_amx8x5_time_t g_psTimeRegs;

//*****************************************************************************
//
// State variable.
//
//*****************************************************************************
volatile uint32_t g_ui32State = TIME_SET;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
am_hal_iom_config_t g_sIOMConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_2MHZ,
    .bSPHA = 1,
    .bSPOL = 1,
    .ui8WriteThreshold = 60,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// AMx8x5 structure.
//
//*****************************************************************************
am_devices_amx8x5_t g_sAMx8x5 =
{
    .bMode = AM_DEVICES_AMX8X5_MODE_SPI,
    .ui32IOMModule = AM_BSP_RTC_IOM,
    .ui32ChipSelect = 2,
    .ui32Address = 0xD2 >> 1,
};

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint64_t ui64IntStatus;

    //
    // Read and clear the interrupt status.
    //
    ui64IntStatus = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64IntStatus);

    //
    // Set the test state.
    //
    g_ui32State = TIME_READ;
}

//*****************************************************************************
//
// Configure GPIOs for communicating with the AMx8x5
//
//*****************************************************************************
void
configure_amx8x5_pins(void)
{

    //
    // Set up IOM1 I2C pins.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_RTC_IOM);
    am_hal_gpio_pin_config(AM_BSP_GPIO_IOM0_CS, AM_BSP_GPIO_CFG_IOM0_CS);

    //
    // Enable a GPIO interrupt for neg edges on the AMx8x5 FOUT pin.
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_RTC_FOUT, AM_BSP_GPIO_CFG_RTC_FOUT);
    am_hal_gpio_int_polarity_bit_set(AM_BSP_GPIO_RTC_FOUT, AM_HAL_GPIO_FALLING);
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_RTC_FOUT));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_RTC_FOUT));

    //
    // Enable functional GPIOs
    //
    am_bsp_pin_enable(RTC_NTIRQ);   // nTIRQ
    am_bsp_pin_enable(RTC_PSW);     // PSW
    am_bsp_pin_enable(RTC_NEXTR);   // nEXTR
    am_bsp_pin_enable(RTC_CLKOUT);  // RTC_CLKOUT
    am_bsp_pin_enable(RTC_NRST);    // RTC_NRST
}

//*****************************************************************************
//
// AM1815 read the time.
//
//*****************************************************************************
void
time_read(void)
{
    //
    // Enable the IOM and I2C mux.
    //
    am_hal_iom_enable(AM_BSP_RTC_IOM);

    //
    // Read the time.
    //
    am_devices_amx8x5_time_get(&g_sAMx8x5);

    //
    // Disable the IOM and I2C mux.
    //
    am_hal_iom_disable(AM_BSP_RTC_IOM);

    //
    // Print the time.
    //
    g_ui32State = TIME_DISPLAY;
}

//*****************************************************************************
//
// AM1815 set OUTB bit.
//
//*****************************************************************************
void
set_outb(void)
{
    uint8_t temp = 0;
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    temp  = am_devices_amx8x5_reg_read(&g_sAMx8x5, AM_DEVICES_AMX8X5_CONTROL_1);
    temp |= 0x20;   // set outb bit
    am_devices_amx8x5_reg_write(&g_sAMx8x5, AM_DEVICES_AMX8X5_CONTROL_1, temp);
    am_hal_iom_disable(AM_BSP_RTC_IOM);
}

//*****************************************************************************
//
// AM1815 clear OUTB bit.
//
//*****************************************************************************
void
clear_outb(void)
{
    uint8_t temp = 0;
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    temp  = am_devices_amx8x5_reg_read(&g_sAMx8x5, AM_DEVICES_AMX8X5_CONTROL_1);
    temp &= 0xDF;   // clear outb bit
    am_devices_amx8x5_reg_write(&g_sAMx8x5, AM_DEVICES_AMX8X5_CONTROL_1, temp);
    am_hal_iom_disable(AM_BSP_RTC_IOM);
}

//*****************************************************************************
//
// AM1815 toggle OUTB bit.
//
//*****************************************************************************
void
toggle_outb(void)
{
    uint8_t temp = 0;
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    temp  = am_devices_amx8x5_reg_read(&g_sAMx8x5, AM_DEVICES_AMX8X5_CONTROL_1);
    if(temp & 0x20)
    {
        temp &= 0xDF;   // clear outb bit
    }
    else
    {
        temp |= 0x20;   // set outb bit
    }
    am_devices_amx8x5_reg_write(&g_sAMx8x5, AM_DEVICES_AMX8X5_CONTROL_1, temp);
    am_hal_iom_disable(AM_BSP_RTC_IOM);
}

//*****************************************************************************
//
// AM1815 clear LKO2 bit.
//
//*****************************************************************************
void
clear_lko2(void)
{
    uint8_t temp = 0;
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    temp  = am_devices_amx8x5_reg_read(&g_sAMx8x5, AM_DEVICES_AMX8X5_OSC_STATUS);
    temp &= 0xDF;   // clear lko2 bit
    am_devices_amx8x5_reg_write(&g_sAMx8x5, AM_DEVICES_AMX8X5_OSC_STATUS, temp);
    am_hal_iom_disable(AM_BSP_RTC_IOM);
}


//*****************************************************************************
//
// AM1815 set register bit.
//
//*****************************************************************************
void
set_reg_bit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t temp = 0;
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    temp  = am_devices_amx8x5_reg_read(&g_sAMx8x5, reg);
    temp |= bit_mask;   // set reg bit
    am_devices_amx8x5_reg_write(&g_sAMx8x5, reg, temp);
    am_hal_iom_disable(AM_BSP_RTC_IOM);
}

//*****************************************************************************
//
// AM1815 toggle register bit.
//
//*****************************************************************************
void
toggle_reg_bit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t temp = 0;
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    temp  = am_devices_amx8x5_reg_read(&g_sAMx8x5, reg);
    if(temp & bit_mask)
    {
        temp &= ~bit_mask;   // clear reg bit
    }
    else
    {
        temp |= bit_mask;   // set reg bit
    }
    am_devices_amx8x5_reg_write(&g_sAMx8x5, reg, temp);
    am_hal_iom_disable(AM_BSP_RTC_IOM);
}

//*****************************************************************************
//
// AM1815 clear register bit.
//
//*****************************************************************************
void
clear_reg_bit(uint8_t reg, uint8_t bit_mask)
{
    uint8_t temp = 0;
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    temp  = am_devices_amx8x5_reg_read(&g_sAMx8x5, reg);
    temp &= ~bit_mask;   // clear reg bit
    am_devices_amx8x5_reg_write(&g_sAMx8x5, reg, temp);
    am_hal_iom_disable(AM_BSP_RTC_IOM);
}

//*****************************************************************************
//
// Support function:
// toVal() converts a string to an ASCII value.
//
//*****************************************************************************
int
toVal(char *pcAsciiStr)
{
    int iRetVal = 0;
    iRetVal += pcAsciiStr[1] - '0';
    iRetVal += pcAsciiStr[0] == ' ' ? 0 : (pcAsciiStr[0] - '0') * 10;
    return iRetVal;
}

//*****************************************************************************
//
// Support function:
// mthToIndex() converts a string indicating a month to an index value.
// The return value is a value 0-12, with 0-11 indicating the month given
// by the string, and 12 indicating that the string is not a month.
//
//*****************************************************************************
int
mthToIndex(char *pcMon)
{
    int idx;
    for (idx = 0; idx < 12; idx++)
    {
        if ( am_util_string_strnicmp(g_pcMonth[idx], pcMon, 3) == 0 )
        {
            return idx;
        }
    }
    return 12;
}

//*****************************************************************************
//
// AM1815 write the time.
//
//*****************************************************************************
void
time_set(void)
{
    //
    // Enable the IOM and I2C mux.
    //
    am_hal_iom_enable(AM_BSP_RTC_IOM);

    //
    // Set the time and protect the time from being written again.
    //
    am_devices_amx8x5_time_set(&g_sAMx8x5, true);

    //
    // Disable the IOM and I2C mux.
    //
    am_hal_iom_disable(AM_BSP_RTC_IOM);

    //
    // Now sleep.
    //
    g_ui32State = TIME_READ;
}

void 
amx8x5_init(void)
{
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    //
    // Write the Key register.
    //
    am_devices_amx8x5_reg_write(&g_sAMx8x5, AM_DEVICES_AMX8X5_CONFIG_KEY,
                                    AM_DEVICES_AMX8X5_CONFIG_KEY_9D);
    am_devices_amx8x5_reg_write(&g_sAMx8x5, AM_DEVICES_AMX8X5_OUTPUT_CTRL, 0x0F);   // enable output1 on FOUT, CLKOUT, nRST, nTIRQ
    am_devices_amx8x5_reg_write(&g_sAMx8x5, AM_DEVICES_AMX8X5_SQW, 0x81);           // enable CLKOUT output

    am_hal_iom_disable(AM_BSP_RTC_IOM);
}



//*****************************************************************************
//
// AM1815 countdown start.
//
//*****************************************************************************
void
countdown_start(void)
{
    //
    // Enable the IOM and I2C mux.
    //
    am_hal_iom_enable(AM_BSP_RTC_IOM);

    //
    // RTC Timer init.
    // 500ms period, level, FOUT interrupt.
    //
    am_devices_amx8x5_countdown_set(&g_sAMx8x5, 0, 500000, 0, 2);

    //
    // Disable the IOM and I2C mux.
    //
    am_hal_iom_disable(AM_BSP_RTC_IOM);

}

//*****************************************************************************
//
// Main.
//
//*****************************************************************************
int
main(void) 
{
    uint8_t ui8Temp, ui8Temp1;
    
    //
    // Set the system clock to maximum frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);
    
    //
    // Configure for low power.
    //
    am_bsp_low_power_init();

    //
    // Initialize the ITM print
    // 
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_itm_string_print);
    am_bsp_pin_enable(ITM_SWO);
    am_hal_itm_enable();
    am_bsp_debug_printf_enable();
    
    //
    // Clear the terminal and print banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("AM1815 RTC Time\r\n");

    //
    // Configure IOM1 for talking with the AM1815.
    //
    am_hal_iom_pwrctrl_enable(AM_BSP_RTC_IOM);
    am_hal_iom_config(AM_BSP_RTC_IOM, &g_sIOMConfig);

    //
    // Configure the GPIOs to work with the AM1815.
    //
    configure_amx8x5_pins();

    //
    // Release nEXTR signal
    //
    am_hal_gpio_out_bit_set(AM_BSP_GPIO_RTC_NEXTR);
 
    //
    // Enable the IOM interrupt in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
    am_hal_interrupt_master_enable();

    amx8x5_init();

    //
    // Read back OUTPUT_CTRL and EXTENDED_ADDR registers to confirm
    //
    am_hal_iom_enable(AM_BSP_RTC_IOM);
    ui8Temp  = am_devices_amx8x5_reg_read(&g_sAMx8x5, AM_DEVICES_AMX8X5_OUTPUT_CTRL);
    ui8Temp1 = am_devices_amx8x5_reg_read(&g_sAMx8x5, AM_DEVICES_AMX8X5_EXTENDED_ADDR);
    am_hal_iom_disable(AM_BSP_RTC_IOM);
   
    clear_lko2();   // to enable OUTB output on PSW

    am_util_stdio_printf("AM_DEVICES_AMX8X5_OUTPUT_CTRL Reg = 0x%02x. \n", ui8Temp);
    am_util_stdio_printf("AM_DEVICES_AMX8X5_EXTENDED_ADDR Reg = 0x%02x. \n", ui8Temp1);
    
    am_util_delay_ms(10);   // give itm some time to print.

    //
    // We are done printing. Disable debug printf messages on ITM.
    //
    am_bsp_debug_printf_disable();
    
    //
    // Loop forever, waiting for interrupts.
    //
    while (1)
    {
        if (g_ui32State == TIME_SET)
        {
            //
            // Set the RTC time for this example.
            // WARNING this will destroy any time epoch currently in the RTC.
            //
#if defined(gcc)  ||  defined(__ARMCC_VERSION)  ||  defined(__IAR_SYSTEMS_ICC__)
            //
            // The RTC is initialized from the date and time strings that are
            // obtained from the compiler at compile time.
            //
            g_psTimeRegs.ui8Hour = toVal(&__TIME__[0]);
            g_psTimeRegs.ui8Minute = toVal(&__TIME__[3]);
            g_psTimeRegs.ui8Second = toVal(&__TIME__[6]);
            g_psTimeRegs.ui8Hundredth = 00;
            g_psTimeRegs.ui8Weekday = am_util_time_computeDayofWeek(2000 + toVal(&__DATE__[9]), mthToIndex(&__DATE__[0]) + 1, toVal(&__DATE__[4]) );
            g_psTimeRegs.ui8Date = toVal(&__DATE__[4]);
            g_psTimeRegs.ui8Month = mthToIndex(&__DATE__[0]) + 1;
            g_psTimeRegs.ui8Year = toVal(&__DATE__[9]);
            g_psTimeRegs.ui8Century = 0;
#else
            //
            // Set the time to Monday at 5:17 UTC on 8/6/2012.
            //
            g_psTimeRegs.ui8Hundredth = 0;
            g_psTimeRegs.ui8Second = 0;
            g_psTimeRegs.ui8Minute = 17;
            g_psTimeRegs.ui8Hour = 5;
            g_psTimeRegs.ui8Date = 6;
            g_psTimeRegs.ui8Weekday = 1;
            g_psTimeRegs.ui8Month = 8;
            g_psTimeRegs.ui8Year = 12;
#endif
            //
            // Set am1815 to 24 hour mode.
            //
            g_psTimeRegs.ui8Mode = AM_DEVICES_AMX8X5_24HR_MODE;

            //
            // Write the time to the AM1815.
            //
            time_set();
        }
        else if (g_ui32State == TIME_READ)
        {
            //
            // Read the time from the AM1815.
            //
             time_read();
        }
        else if (g_ui32State == TIME_DISPLAY)
        {
            //
            // Enable the ITM to print the time.
            //
            am_bsp_debug_printf_enable();

            //
            // Print the time.
            //
            am_util_stdio_printf("AM1815 RTC says: It is now ");
            am_util_stdio_printf("%d : ", g_psTimeRegs.ui8Hour);
            am_util_stdio_printf("%02d : ", g_psTimeRegs.ui8Minute);
            am_util_stdio_printf("%02d.", g_psTimeRegs.ui8Second);
            am_util_stdio_printf("%02d ", g_psTimeRegs.ui8Hundredth);
            am_util_stdio_printf(g_pcWeekday[g_psTimeRegs.ui8Weekday]);
            am_util_stdio_printf(" ");
            am_util_stdio_printf(g_pcMonth[g_psTimeRegs.ui8Month - 1]);
            am_util_stdio_printf(" ");
            am_util_stdio_printf("%d, ", g_psTimeRegs.ui8Date);
            am_util_stdio_printf("20%02d", g_psTimeRegs.ui8Year);
            am_util_stdio_printf("\n");
            
            //
            // We are done printing.
            // Disable the ITM.
            //
            am_bsp_debug_printf_disable();
            toggle_outb();
            toggle_reg_bit(AM_DEVICES_AMX8X5_CONTROL_1, 0x08);  // RSP bit in CONTROL 1 reg
            
            //
            // Sleep until the next interrupt from the AM1815.
            //
            g_ui32State = APP_SLEEP;
        }
        else
        {
            //
            // Start the AM1815 countdown timer and then sleep.
            //
            countdown_start();
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        }
    }
}
