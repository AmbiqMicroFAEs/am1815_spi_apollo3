//*****************************************************************************
// AM1815 SPI BASED EXAMPLE, 
// Apollo3 Blue
// Mariusz Lacina, Ambiq 2023 
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "am_mcu_apollo.h"
#include "am_hal_gpio.h"
#include "am_bsp.h"
#include "am_util.h"
#include "iom_dma.h"
#include "uart_debug.h"
#include "iom_spi.h"
#include "amx8x5_device.h"

//*****************************************************************************
//
// Structure in the AMx8x5 devices source for holding the time.
//
//*****************************************************************************
amx8x5_time_t g_psTimeRegs;


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
//  This function enables several power-saving features of the MCU, and
// disables some of the less-frequently used peripherals
//*****************************************************************************
void low_power_init(void)
{
    //
    // Initialize for low power in the power control block
    //
    am_hal_pwrctrl_low_power_init();

    //
    // Disable the RTC.
    //
    am_hal_rtc_osc_disable();
} 


//*****************************************************************************
//
// Support function:
// toVal() converts a string to an ASCII value.
//
//*****************************************************************************
int toVal(char *pcAsciiStr)
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
int mthToIndex(char *pcMon)
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
// uint32_t amx8x5_ext1_gen(void)
//
// It generates EXT1 interrupt using GPIO
//
//*****************************************************************************
void amx8x5_ext1_gen(void)
{
	am_hal_gpio_state_write(GPIO_RTC_EXTI,AM_HAL_GPIO_OUTPUT_SET);
		
	am_util_delay_us(10);
		
	am_hal_gpio_state_write(GPIO_RTC_EXTI,AM_HAL_GPIO_OUTPUT_CLEAR);
	
	am_util_delay_us(10);
	
}

//*****************************************************************************
//
// uint32_t amx8x5_init(void)
//
//*****************************************************************************
uint32_t amx8x5_init(void)
{
uint32_t status=AM_HAL_STATUS_SUCCESS;
	
		//
    // Write the Key register.
    //
    status = amx8x5_reg_write(AM_DEVICES_AMX8X5_CONFIG_KEY,AM_DEVICES_AMX8X5_CONFIG_KEY_9D);
		if(status != AM_HAL_STATUS_SUCCESS)return status;
	
		am_util_delay_us(3);
	
    status = amx8x5_reg_write(AM_DEVICES_AMX8X5_OUTPUT_CTRL, 0x00);   	// disable outputs
		if(status != AM_HAL_STATUS_SUCCESS)return status;
	
		am_util_delay_us(3);
	
		status = amx8x5_reg_write(AM_DEVICES_AMX8X5_SQW, 0b00100000);       // disable CLKOUT output
		if(status != AM_HAL_STATUS_SUCCESS)return status;
		
		am_util_delay_us(3);

		status = amx8x5_reg_write(AM_DEVICES_AMX8X5_INT_MASK, 0b11100000); 	//default POR state
		if(status != AM_HAL_STATUS_SUCCESS)return status;
		
		am_util_delay_us(3);
		
		status = amx8x5_reg_write(AM_DEVICES_AMX8X5_STATUS,0x00);  					//Clear INT flags
			
		am_util_delay_us(3);
	
return status;
}

//*****************************************************************************
//
// uint32_t amx8x5_reset(void)
//
//*****************************************************************************
uint32_t amx8x5_ext_reset(void)
{
uint32_t status;	
	
	status = am_hal_gpio_state_write(GPIO_RTC_EXTR,AM_HAL_GPIO_OUTPUT_SET);
	if(status != AM_HAL_STATUS_SUCCESS)return status;
	
	am_util_delay_ms(10);
	
	status = am_hal_gpio_state_write(GPIO_RTC_EXTR,AM_HAL_GPIO_OUTPUT_CLEAR);
	if(status != AM_HAL_STATUS_SUCCESS)return status;
	
	am_util_delay_ms(10);
	
	status = am_hal_gpio_state_write(GPIO_RTC_EXTR,AM_HAL_GPIO_OUTPUT_SET);

	am_util_delay_ms(500);
	
return status;	
}


//*****************************************************************************
//
// uint32_t amx8x5_int_clear(void)
//
// It clears interrupts
//
//*****************************************************************************
uint32_t amx8x5_int_clear(void)
{
uint32_t status;
uint8_t ui8Temp1;
	
	status = amx8x5_reg_read(AM_DEVICES_AMX8X5_STATUS,&ui8Temp1);		
	if(status != AM_HAL_STATUS_SUCCESS)return status;
	
	ui8Temp1 &= (~ui8Temp1);
	
	status = amx8x5_reg_write(AM_DEVICES_AMX8X5_STATUS,ui8Temp1);

return status;
}



//*****************************************************************************
//
// uint32_t amx8x5_wait_por_ready(void)
//
// Wait until the RTC is ready after POR
//
//*****************************************************************************
uint32_t amx8x5_wait_por_ready(void)
{
uint32_t status, fout;
uint8_t delay=10;	
	
		//
		// Time limit (delay) is added for RESET sources other than POR
		//
		// Before use, please study: Power On AC Electrical Parameters (datasheet)
		//
		// heavily influenced by ambient temp
		//
		
	
		do{
		status = am_hal_gpio_state_read(GPIO_RTC_FOUT,AM_HAL_GPIO_INPUT_READ,&fout);
		am_util_delay_ms(100);
		if(--delay == 0)return status;
		if(status != AM_HAL_STATUS_SUCCESS)return status;		
			
		}while(fout == 0);
		
return status;		
}



//*****************************************************************************
//
//  Sleep Test Loop
//
//*****************************************************************************
void sleep_loop(void)
{
uint32_t status;
uint8_t ui8Temp,ui8Temp1;
	
	//
	// Time set
	//		
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
	g_psTimeRegs.ui8Mode = AM_DEVICES_AMX8X5_24HR_MODE;

	status = amx8x5_time_set(&g_psTimeRegs, true);
	if(status != AM_HAL_STATUS_SUCCESS)
		{
		while(1){}; //errors occured !	
		}

	am_util_delay_us(3);
		
	//
	//	Enable EXTI interrupt for wake-up
	//
	status = amx8x5_reg_set(AM_DEVICES_AMX8X5_INT_MASK, 0x01); 				//enable EX1E interrupt, default POR edge: falling
	if(status != AM_HAL_STATUS_SUCCESS)
		{
		while(1){}; //errors occured !	
		}	
		
	am_util_delay_us(3);	
		
	//
	// Sleep loop
	//		
	while(1)
	{
	//
	// Clear interrupts
	//	
	status = amx8x5_int_clear();
	if(status != 0)
		{
		while(1){}; //errors occured or sleep request not accepted!	
		}			
		
	am_util_delay_us(3);
		
	//
	// enter sleep mode
	//
	status = amx8x5_sleep_set(0, 1); //0 -> nRST is pulled low in sleep mode
	if(status != 0)
		{
		while(1){}; //errors occured or sleep request not accepted!	
		}		
		
	//Delay	
	am_util_delay_ms(3000);	
			
	//
	// EXT1 interrupt generation -> wake-up
	//
	amx8x5_ext1_gen();
		
	//
	// Read and print the time
	//
	status = amx8x5_time_get(&g_psTimeRegs);
	if(status != AM_HAL_STATUS_SUCCESS)
		{
		while(1){}; //errors occured !	
		}	
		
	if(ui8Temp != g_psTimeRegs.ui8Second)
		{
		ui8Temp = g_psTimeRegs.ui8Second;		
			
		//
		// Print the time.
		//
		am_util_stdio_printf("AM1815 RTC time update: ");
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
		am_util_stdio_printf("\r");			
		}		
	}		
}



//*****************************************************************************
//
//  MAIN loop
//
//*****************************************************************************
int main(void)
{
uint32_t status;
uint8_t ui8Temp,ui8Temp1;
	
	
  am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

  //
  // Set the default cache configuration
  //
  am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
  am_hal_cachectrl_enable();
  //am_hal_cachectrl_disable();
  
  //
  // Configure the board for low power operation.
  //
  low_power_init();
  
  //
  // IOM Init
  //
  status = iom_spi_init();
	if(status != AM_HAL_STATUS_SUCCESS)
		{
			while(1){}; //errors occured, debug it!
		}

  //
  // Enable the print interface.
  //
  debug_printf_enable();
  
  //
  // Print banner.
  //
  am_util_stdio_printf("IOM DMA SPI <-> AM1815 example \n\n\r");
  
  //
  // Global interrupts
  //
  am_hal_interrupt_master_enable();

	//
	// Reset RTC
	//
	status = amx8x5_ext_reset();
	if(status != AM_HAL_STATUS_SUCCESS)
		{
		while(1){}; //errors occured !	
		}	
		
	//
	// Software Reset RTC
	//
	status = amx8x5_reset();
	if(status != AM_HAL_STATUS_SUCCESS)
		{
		while(1){}; //errors occured !	
		}			
	am_util_delay_us(3);
			
	//
	// Wait RTC power-up
	//
	status = amx8x5_wait_por_ready();
	if(status != AM_HAL_STATUS_SUCCESS)
		{
		while(1){}; //errors occured !	
		}	
		
	am_util_delay_ms(4000); //Wait for XTAL start-up - please refer "Artasie AMX8X5 Startup Timing
	
	//
  // Init RTC
  //
	status = amx8x5_init();
	if(status != AM_HAL_STATUS_SUCCESS)
		{
		while(1){}; //errors occured !	
		}
	

//*****************************************************************************
//
//  APP tests loop
//
//*****************************************************************************

		sleep_loop();
		
		
		
	while(1){};	
		
	
}

