//*****************************************************************************
// AMx8x5 RTC Driver
// Mariusz Lacina, Ambiq, 2023
//*****************************************************************************
#ifndef AMX8X5_DEVICE_H
#define AMX8X5_DEVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_hal_gpio.h"


#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
// GPIOs for communicating with the AMx8x5, 
// Ambiq Microbus shield, socket 1
//*****************************************************************************
#define GPIO_RTC_CS					44
#define GPIO_RTC_SCK				5
#define GPIO_RTC_MISO				6
#define	GPIO_RTC_MOSI				7
#define GPIO_RTC_FOUT				31
#define GPIO_RTC_EXTR				39
#define GPIO_RTC_CLKOUT			48
#define GPIO_RTC_EXTI				40
#define GPIO_RTC_PSW				13
#define GPIO_RTC_WDI				35
#define GPIO_RTC_RST				8


//*****************************************************************************
// Register Defines.
//*****************************************************************************
#define AM_DEVICES_AMX8X5_HUNDREDTHS                0x00
#define AM_DEVICES_AMX8X5_YEARS                     0x06
#define AM_DEVICES_AMX8X5_ALARM_HUNDRS              0x08
#define AM_DEVICES_AMX8X5_STATUS                    0x0F
#define AM_DEVICES_AMX8X5_CONTROL_1                 0x10
#define AM_DEVICES_AMX8X5_CONTROL_2                 0x11
#define AM_DEVICES_AMX8X5_INT_MASK                  0x12
#define AM_DEVICES_AMX8X5_SQW                       0x13
#define AM_DEVICES_AMX8X5_CAL_XT                    0x14
#define AM_DEVICES_AMX8X5_CAL_RC_HI                 0x15
#define AM_DEVICES_AMX8X5_CAL_RC_LOW                0x16
#define AM_DEVICES_AMX8X5_SLEEP_CTRL                0x17
#define AM_DEVICES_AMX8X5_TIMER_CTRL                0x18
#define AM_DEVICES_AMX8X5_TIMER                     0x19
#define AM_DEVICES_AMX8X5_TIMER_INITIAL             0x1A
#define AM_DEVICES_AMX8X5_WDT                       0x1B
#define AM_DEVICES_AMX8X5_OSC_CONTROL               0x1C
#define AM_DEVICES_AMX8X5_OSC_STATUS                0x1D
#define AM_DEVICES_AMX8X5_CONFIG_KEY                0x1F
#define AM_DEVICES_AMX8X5_ACAL_FLT                  0x26
  
#define AM_DEVICES_AMX8X5_OUTPUT_CTRL               0x30
  
#define AM_DEVICES_AMX8X5_EXTENDED_ADDR             0x3F
#define AM_DEVICES_AMX8X5_RAM_START                 0x40

// Keys.
#define AM_DEVICES_AMX8X5_CONFIG_KEY_VAL            0xA1
#define AM_DEVICES_AMX8X5_CONFIG_KEY_3C             0x3C
#define AM_DEVICES_AMX8X5_CONFIG_KEY_9D             0x9D

// Modes
#define AM_DEVICES_AMX8X5_12HR_MODE                 0x01
#define AM_DEVICES_AMX8X5_24HR_MODE                 0x02


//*****************************************************************************
// Structure used for time keeping.
//*****************************************************************************
typedef struct
{
    uint8_t ui8Hundredth;
    uint8_t ui8Second;
    uint8_t ui8Minute;
    uint8_t ui8Hour;
    uint8_t ui8Date;
		uint8_t ui8Month;
		uint8_t ui8Year;
    uint8_t ui8Weekday;
    uint8_t ui8Century;
		uint8_t ui8Mode;
} amx8x5_time_t;

//extern amx8x5_time_t Time;

//*****************************************************************************
// External function definitions
//*****************************************************************************

extern uint32_t amx8x5_reg_clear(uint8_t ui8Address, uint8_t ui8Mask);

extern uint32_t amx8x5_reg_read(uint8_t ui8Register, uint8_t *pbuf);

extern uint32_t amx8x5_reg_set(uint8_t ui8Address, uint8_t ui8Mask);

extern uint32_t amx8x5_reg_block_read(uint8_t ui8StartRegister,
																			uint8_t *pui32Values,
																			uint32_t ui32NumBytes);

extern uint32_t amx8x5_reg_write(uint8_t ui8Register, uint8_t ui8Value);


extern uint32_t amx8x5_reg_block_write(uint8_t ui8StartRegister,
                                  uint8_t *pui32Values,
                                  uint32_t ui32NumBytes);

extern uint32_t amx8x5_reset(void);

extern uint32_t amx8x5_time_get(amx8x5_time_t *time);
extern uint32_t amx8x5_time_set(amx8x5_time_t *time, uint8_t ui8Protect);

extern uint32_t am_devices_amx8x5_cal_set(uint8_t ui8Mode,int32_t iAdjust);

extern uint32_t amx8x5_alarm_set(amx8x5_time_t *alarm_time, uint8_t ui8Repeat, 
												            uint8_t ui8IntMode, uint8_t ui8Pin);

extern uint32_t amx8x5_countdown_set(uint8_t ui8Range, int32_t iPeriod, uint8_t ui8Repeat,
                                uint8_t ui8Pin);

extern uint32_t amx8x5_osc_sel(uint8_t ui8OSC);

extern uint32_t amx8x5_sqw_set(uint8_t ui8SQFS, uint8_t ui8Pin);

extern uint32_t amx8x5_sleep_set(uint8_t ui8Timeout, uint8_t ui8Mode);

extern uint32_t amx8x5_watchdog_set(uint32_t ui8Period, uint8_t ui8Pin);

extern uint32_t amx8x5_autocal_set(uint8_t ui8Period);

extern uint32_t amx8x5_ext_address_get(uint8_t ui8Address, uint8_t* ext_address);

extern uint32_t amx8x5_ram_read(uint8_t ui8Address, uint8_t* ram_data);

extern uint32_t amx8x5_ram_write(uint8_t ui8Address, uint8_t ui8Data);

#ifdef __cplusplus
}
#endif

#endif 
