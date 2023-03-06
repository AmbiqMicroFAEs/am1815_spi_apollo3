//*****************************************************************************
// Simple DMA Driver
// Apollo3
// Mariusz Lacina, Ambiq, 2023
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "iom_dma.h"
#include "am_util_delay.h"

#ifdef __IAR_SYSTEMS_ICC__
#define AM_INSTR_CLZ(n)                     __CLZ(n)
#else
#define AM_INSTR_CLZ(n)                     __builtin_clz(n)
#endif

volatile bool pop_tail;   //flag used to pop last incomplete word from fifo
uint32_t  *transaction_rx_buff;
uint32_t   transaction_rx_size;




//
// Only keep IOM interrupts we're interested in
//
// Necessary interrupts for respective modes
// Need both CMDCMP & DCMP, as for Read we need to wait for DCMP after CMDCMP
#define IOM_DMA_INT_DMAMODE  (AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_ERR | AM_HAL_IOM_INT_DCMP)
//#define IOM_DMA_INT_DMAMODE  (AM_HAL_IOM_INT_ERR)

// Configures the interrupts to provided coniguration - clearing all pending interrupts
#define IOM_DMA_SET_INTEN(ui32Module, intCfg)                               \
    do                                                                  \
    {                                                                   \
        IOMn(ui32Module)->INTEN = 0;                                    \
        IOMn(ui32Module)->INTCLR = AM_HAL_IOM_INT_ALL;                  \
        IOMn(ui32Module)->INTEN = (intCfg);                             \
    } while (0);

//----------------------------------------------------------------------------//
//  IOMx Callbacks
//----------------------------------------------------------------------------//
am_dma_iom_callback_t user_callbacks[6] = {NULL};
void* callback_cntx[6] = {NULL};


//*****************************************************************************
// compute_freq()
//*****************************************************************************
//
// Compute the interface frequency based on the given parameters
//
static uint32_t compute_freq(uint32_t ui32HFRCfreqHz,
             uint32_t ui32Fsel, uint32_t ui32Div3,
             uint32_t ui32DivEn, uint32_t ui32TotPer)
{
    uint32_t ui32Denomfinal, ui32ClkFreq;

    ui32Denomfinal = ((1 << (ui32Fsel - 1)) * (1 + ui32Div3 * 2) * (1 + ui32DivEn * (ui32TotPer)));
    ui32ClkFreq = (ui32HFRCfreqHz) / ui32Denomfinal;                           // Compute the set frequency value
    ui32ClkFreq +=  (((ui32HFRCfreqHz) % ui32Denomfinal) > (ui32Denomfinal / 2)) ? 1 : 0;

    return ui32ClkFreq;
} // compute_freq()


//*****************************************************************************
// onebit()
//*****************************************************************************
//
// A power of 2?
// Return true if ui32Value has exactly 1 bit set, otherwise false.
//
static bool onebit(uint32_t ui32Value)
{
    return ui32Value  &&  !(ui32Value & (ui32Value - 1));
} // onebit()

//*****************************************************************************
//
// iom_get_interface_clock_cfg()
//
// Returns the proper settings for the CLKCFG register.
//
// ui32FreqHz - The desired interface frequency in Hz.
//
// Given a desired serial interface clock frequency, this function computes
// the appropriate settings for the various fields in the CLKCFG register
// and returns the 32-bit value that should be written to that register.
// The actual interface frequency may be slightly lower than the specified
// frequency, but the actual frequency is also returned.
//
// Note A couple of criteria that this algorithm follow are:
//  1. For power savings, choose the highest FSEL possible.
//  2. Use DIV3 when possible rather than DIVEN.
//
// Return An unsigned 64-bit value.
// The lower 32-bits represent the value to use to set CLKCFG.
// The upper 32-bits represent the actual frequency (in Hz) that will result
// from setting CLKCFG with the lower 32-bits.
//
// 0 (64 bits) = error. Note that the caller must check the entire 64 bits.
// It is not an error if only the low 32-bits are 0 (this is a valid value).
// But the entire 64 bits returning 0 is an error.
//!
//*****************************************************************************
static uint64_t iom_get_interface_clock_cfg(uint32_t ui32FreqHz, uint32_t ui32Phase )
{
    uint32_t ui32Fsel, ui32Div3, ui32DivEn, ui32TotPer, ui32LowPer;
    uint32_t ui32Denom, ui32v1, ui32Denomfinal, ui32ClkFreq, ui32ClkCfg;
    uint32_t ui32HFRCfreqHz;
    int32_t i32Div, i32N;

    if ( ui32FreqHz == 0 )
    {
        return 0;
    }

    //
    // Set the HFRC clock frequency.
    //
    ui32HFRCfreqHz = AM_HAL_CLKGEN_FREQ_MAX_HZ;

    //
    // Compute various parameters used for computing the optimal CLKCFG setting.
    //
    i32Div = (ui32HFRCfreqHz / ui32FreqHz) + ((ui32HFRCfreqHz % ui32FreqHz) ? 1 : 0);    // Round up (ceiling)

    //
    // Compute N (count the number of LS zeros of Div) = ctz(Div) = log2(Div & (-Div))
    //
    i32N = 31 - AM_INSTR_CLZ((i32Div & (-i32Div)));

    if ( i32N > 6 )
    {
        i32N = 6;
    }

    ui32Div3 = ( (ui32FreqHz < (ui32HFRCfreqHz / 16384))            ||
                 ( ((ui32FreqHz >= (ui32HFRCfreqHz / 3))    &&
                    (ui32FreqHz <= ((ui32HFRCfreqHz / 2) - 1)) ) ) ) ? 1 : 0;
    ui32Denom = ( 1 << i32N ) * ( 1 + (ui32Div3 * 2) );
    ui32TotPer = i32Div / ui32Denom;
    ui32TotPer += (i32Div % ui32Denom) ? 1 : 0;
    ui32v1 = 31 - AM_INSTR_CLZ(ui32TotPer);     // v1 = log2(TotPer)
    ui32Fsel = (ui32v1 > 7) ? ui32v1 + i32N - 7 : i32N;
    ui32Fsel++;

    if ( ui32Fsel > 7 )
    {
        //
        // This is an error, can't go that low.
        //
        return 0;
    }

    if ( ui32v1 > 7 )
    {
        ui32DivEn = ui32TotPer;     // Save TotPer for the round up calculation
        ui32TotPer = ui32TotPer>>(ui32v1-7);
        ui32TotPer += ((ui32DivEn) % (1 << (ui32v1 - 7))) ? 1 : 0;
    }

    ui32DivEn = ( (ui32FreqHz >= (ui32HFRCfreqHz / 4)) ||
                  ((1 << (ui32Fsel - 1)) == i32Div) ) ? 0 : 1;

    if (ui32Phase == 1)
    {
        ui32LowPer = (ui32TotPer - 2) / 2;          // Longer high phase
    }
    else
    {
        ui32LowPer = (ui32TotPer - 1) / 2;          // Longer low phase
    }

    ui32ClkCfg = _VAL2FLD(IOM0_CLKCFG_FSEL,   ui32Fsel)     |
                 _VAL2FLD(IOM0_CLKCFG_DIV3,   ui32Div3)     |
                 _VAL2FLD(IOM0_CLKCFG_DIVEN,  ui32DivEn)    |
                 _VAL2FLD(IOM0_CLKCFG_LOWPER, ui32LowPer)   |
                 _VAL2FLD(IOM0_CLKCFG_TOTPER, ui32TotPer - 1);

    //
    // Now, compute the actual frequency, which will be returned.
    //
    ui32ClkFreq = compute_freq(ui32HFRCfreqHz, ui32Fsel, ui32Div3, ui32DivEn, ui32TotPer - 1);

    //
    // Determine if the actual frequency is a power of 2 (MHz).
    //
    if ( (ui32ClkFreq % 250000) == 0 )
    {
        //
        // If the actual clock frequency is a power of 2 ranging from 250KHz up,
        // we can simplify the CLKCFG value using DIV3 (which also results in a
        // better duty cycle).
        //
        ui32Denomfinal = ui32ClkFreq / (uint32_t)250000;

        if ( onebit(ui32Denomfinal) )
        {
            //
            // These configurations can be simplified by using DIV3.  Configs
            // using DIV3 have a 50% duty cycle, while those from DIVEN will
            // have a 66/33 duty cycle.
            //
            ui32TotPer = ui32LowPer = ui32DivEn = 0;
            ui32Div3 = 1;

            //
            // Now, compute the return values.
            //
            ui32ClkFreq = compute_freq(ui32HFRCfreqHz, ui32Fsel, ui32Div3, ui32DivEn, ui32TotPer);

    ui32ClkCfg = _VAL2FLD(IOM0_CLKCFG_FSEL,   ui32Fsel)     |
                 _VAL2FLD(IOM0_CLKCFG_DIV3,   1)            |
                 _VAL2FLD(IOM0_CLKCFG_DIVEN,  0)            |
                 _VAL2FLD(IOM0_CLKCFG_LOWPER, 0)            |
                 _VAL2FLD(IOM0_CLKCFG_TOTPER, 0);
        }
    }

    return ( ((uint64_t)ui32ClkFreq) << 32) | (uint64_t)ui32ClkCfg;

} //iom_get_interface_clock_cfg()


//----------------------------------------------------------------------------//
//  IOM I2C configuration function.
//----------------------------------------------------------------------------//
uint32_t iom_dma_spi_configure(void *pHandle, iom_dma_config_t *psConfig)
{
uint32_t ui32ClkCfg;
iom_dma_state_t *pIOMState = (iom_dma_state_t*)pHandle;
uint32_t status = AM_HAL_STATUS_SUCCESS;
uint32_t ui32Module;


    ui32Module = pIOMState->ui32Module;
    //
    // Save the interface mode and chip select in the global handle.
    //
    pIOMState->eInterfaceMode = psConfig->eInterfaceMode;

    //
    // Set the IOM read/write FIFO thresholds to default values.
    //
    IOMn(ui32Module)->FIFOTHR =
        _VAL2FLD(IOM0_FIFOTHR_FIFORTHR, 16) |
        _VAL2FLD(IOM0_FIFOTHR_FIFOWTHR, 16);


   if ( psConfig->eInterfaceMode == AM_HAL_IOM_SPI_MODE )
    {
#ifndef AM_HAL_DISABLE_API_VALIDATION
        //
        // Validate the SPI mode
        //
        if ( psConfig->eSpiMode > AM_HAL_IOM_SPI_MODE_3 )
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }
        if (psConfig->ui32ClockFreq > AM_HAL_IOM_MAX_FREQ)
        {
            return AM_HAL_STATUS_INVALID_ARG;
        }
#endif // AM_HAL_DISABLE_API_VALIDATION

        //
        // Determine the CLKCFG value for SPI.
        //
        ui32ClkCfg = iom_get_interface_clock_cfg(psConfig->ui32ClockFreq, (psConfig->eSpiMode & 2) >> 1);

        //
        // Set the SPI configuration.
        //
        IOMn(ui32Module)->MSPICFG =
            ( ((psConfig->eSpiMode << IOM0_MSPICFG_SPOL_Pos) & (IOM0_MSPICFG_SPHA_Msk | IOM0_MSPICFG_SPOL_Msk))  |
             _VAL2FLD(IOM0_MSPICFG_FULLDUP, 0)                              |
             _VAL2FLD(IOM0_MSPICFG_WTFC,    IOM0_MSPICFG_WTFC_DIS)          |
             _VAL2FLD(IOM0_MSPICFG_RDFC,    IOM0_MSPICFG_RDFC_DIS)          |
             _VAL2FLD(IOM0_MSPICFG_MOSIINV, IOM0_MSPICFG_MOSIINV_NORMAL)    |
             _VAL2FLD(IOM0_MSPICFG_WTFCIRQ, IOM0_MSPICFG_WTFCIRQ_MISO)      |
             _VAL2FLD(IOM0_MSPICFG_WTFCPOL, IOM0_MSPICFG_WTFCPOL_HIGH)      |
             _VAL2FLD(IOM0_MSPICFG_RDFCPOL, IOM0_MSPICFG_RDFCPOL_HIGH)      |
             _VAL2FLD(IOM0_MSPICFG_SPILSB,  IOM0_MSPICFG_SPILSB_MSB)        |
             _VAL2FLD(IOM0_MSPICFG_DINDLY,  0)                              |
             _VAL2FLD(IOM0_MSPICFG_DOUTDLY, 0)                              |
             _VAL2FLD(IOM0_MSPICFG_MSPIRST, 0) );
    }else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Enable and set the clock configuration.
    //
    ui32ClkCfg |= _VAL2FLD(IOM0_CLKCFG_IOCLKEN, 1);
    IOMn(ui32Module)->CLKCFG = ui32ClkCfg;

    //
    // Return the status.
    //
    return status;
}

//----------------------------------------------------------------------------//
//  IOM I2C configuration function.
//----------------------------------------------------------------------------//
uint32_t iom_dma_i2c_configure(void *pHandle, iom_dma_config_t *psConfig)
{
uint32_t ui32ClkCfg;
iom_dma_state_t *pIOMState = (iom_dma_state_t*)pHandle;
uint32_t status = AM_HAL_STATUS_SUCCESS;
uint32_t ui32Module;


    ui32Module = pIOMState->ui32Module;
    //
    // Save the interface mode and chip select in the global handle.
    //
    pIOMState->eInterfaceMode = psConfig->eInterfaceMode;

    //
    // Set the IOM read/write FIFO thresholds to default values.
    //
    IOMn(ui32Module)->FIFOTHR =
        _VAL2FLD(IOM0_FIFOTHR_FIFORTHR, 16) |
        _VAL2FLD(IOM0_FIFOTHR_FIFOWTHR, 16);

    if ( psConfig->eInterfaceMode == AM_HAL_IOM_I2C_MODE )
    {

    switch (psConfig->ui32ClockFreq)
    {
    case AM_HAL_IOM_100KHZ:
      //
      // settings below should give ~100 kHz
      //
      ui32ClkCfg =
                  _VAL2FLD(IOM0_CLKCFG_TOTPER, 0x77)                    |
                  _VAL2FLD(IOM0_CLKCFG_LOWPER, 0x3B)                    |
                  _VAL2FLD(IOM0_CLKCFG_DIVEN, IOM0_CLKCFG_DIVEN_EN)     |
                  _VAL2FLD(IOM0_CLKCFG_DIV3, IOM0_CLKCFG_DIV3_DIS)      |
                  _VAL2FLD(IOM0_CLKCFG_FSEL, IOM0_CLKCFG_FSEL_HFRC_DIV2)|
                  _VAL2FLD(IOM0_CLKCFG_IOCLKEN, 1);
      IOMn(ui32Module)->MI2CCFG = 
                  _VAL2FLD(IOM0_MI2CCFG_STRDIS, 0)                      |
                  _VAL2FLD(IOM0_MI2CCFG_SMPCNT, 3)                      |
                  _VAL2FLD(IOM0_MI2CCFG_SDAENDLY, 15)                   |
                  _VAL2FLD(IOM0_MI2CCFG_SCLENDLY, 0)                    |
                  _VAL2FLD(IOM0_MI2CCFG_MI2CRST, 1)                     |
                  _VAL2FLD(IOM0_MI2CCFG_SDADLY, 3)                      |
                  _VAL2FLD(IOM0_MI2CCFG_ARBEN, IOM0_MI2CCFG_ARBEN_ARBDIS)|
                  _VAL2FLD(IOM0_MI2CCFG_I2CLSB, IOM0_MI2CCFG_I2CLSB_MSBFIRST)|
                  _VAL2FLD(IOM0_MI2CCFG_ADDRSZ, IOM0_MI2CCFG_ADDRSZ_ADDRSZ7);
    break;

    case AM_HAL_IOM_400KHZ:
      //
      // settings below should give ~400 kHz
      //
      ui32ClkCfg = 
                  _VAL2FLD(IOM0_CLKCFG_TOTPER, 0x1D)                    |
                  _VAL2FLD(IOM0_CLKCFG_LOWPER, 0x0E)                    |
                  _VAL2FLD(IOM0_CLKCFG_DIVEN, IOM0_CLKCFG_DIVEN_EN)     |
                  _VAL2FLD(IOM0_CLKCFG_DIV3, IOM0_CLKCFG_DIV3_DIS)      |
                  _VAL2FLD(IOM0_CLKCFG_FSEL, IOM0_CLKCFG_FSEL_HFRC_DIV2)|
                  _VAL2FLD(IOM0_CLKCFG_IOCLKEN, 1);
      IOMn(ui32Module)->MI2CCFG = 
                  _VAL2FLD(IOM0_MI2CCFG_STRDIS, 0)                      |
                  _VAL2FLD(IOM0_MI2CCFG_SMPCNT, 3)                      |
                  _VAL2FLD(IOM0_MI2CCFG_SDAENDLY, 15)                   |
                  _VAL2FLD(IOM0_MI2CCFG_SCLENDLY, 2)                    |
                  _VAL2FLD(IOM0_MI2CCFG_MI2CRST, 1)                     |
                  _VAL2FLD(IOM0_MI2CCFG_SDADLY, 3)                      |
                  _VAL2FLD(IOM0_MI2CCFG_ARBEN, IOM0_MI2CCFG_ARBEN_ARBDIS)|
                  _VAL2FLD(IOM0_MI2CCFG_I2CLSB, IOM0_MI2CCFG_I2CLSB_MSBFIRST)|
                  _VAL2FLD(IOM0_MI2CCFG_ADDRSZ, IOM0_MI2CCFG_ADDRSZ_ADDRSZ7);
    break;

    case AM_HAL_IOM_1MHZ:
      //
      // settings below should give ~860 kHz
      //
      ui32ClkCfg = 
                  _VAL2FLD(IOM0_CLKCFG_TOTPER, 0x06)                    |
                  _VAL2FLD(IOM0_CLKCFG_LOWPER, 0x03)                    |
                  _VAL2FLD(IOM0_CLKCFG_DIVEN, IOM0_CLKCFG_DIVEN_EN)     |
                  _VAL2FLD(IOM0_CLKCFG_DIV3, IOM0_CLKCFG_DIV3_DIS)      |
                  _VAL2FLD(IOM0_CLKCFG_FSEL, IOM0_CLKCFG_FSEL_HFRC_DIV4)|
                  _VAL2FLD(IOM0_CLKCFG_IOCLKEN, 1);
      IOMn(ui32Module)->MI2CCFG = 
                  _VAL2FLD(IOM0_MI2CCFG_STRDIS, 0)                      |
                  _VAL2FLD(IOM0_MI2CCFG_SMPCNT, 0x21)                   |
                  _VAL2FLD(IOM0_MI2CCFG_SDAENDLY, 3)                    |
                  _VAL2FLD(IOM0_MI2CCFG_SCLENDLY, 0)                    |
                  _VAL2FLD(IOM0_MI2CCFG_MI2CRST, 1)                     |
                  _VAL2FLD(IOM0_MI2CCFG_SDADLY, 0)                      |
                  _VAL2FLD(IOM0_MI2CCFG_ARBEN, IOM0_MI2CCFG_ARBEN_ARBDIS)|
                  _VAL2FLD(IOM0_MI2CCFG_I2CLSB, IOM0_MI2CCFG_I2CLSB_MSBFIRST)|
                  _VAL2FLD(IOM0_MI2CCFG_ADDRSZ, IOM0_MI2CCFG_ADDRSZ_ADDRSZ7);
    break;

    default:
        return AM_HAL_STATUS_INVALID_ARG;
    }

    }
    else
    {
        return AM_HAL_STATUS_OUT_OF_RANGE;
    }

    //
    // Enable and set the clock configuration.
    //
    ui32ClkCfg |= _VAL2FLD(IOM0_CLKCFG_IOCLKEN, 1);
    IOMn(ui32Module)->CLKCFG = ui32ClkCfg;

    //
    // Return the status.
    //
    return status;
}


//----------------------------------------------------------------------------//
//  IOM DMA mode configuration
//----------------------------------------------------------------------------//
uint32_t iom_dma_configure(void *pHandle, iom_dma_config_t *psConfig)
{
  if ( psConfig->eInterfaceMode == AM_HAL_IOM_I2C_MODE )
  {
  return iom_dma_i2c_configure(pHandle,psConfig);
  }
  else
  if ( psConfig->eInterfaceMode == AM_HAL_IOM_SPI_MODE )
  {
  return iom_dma_spi_configure(pHandle,psConfig);
  }

return AM_HAL_STATUS_INVALID_ARG;
}


//----------------------------------------------------------------------------//
//  IOM Enable
//----------------------------------------------------------------------------//
uint32_t iom_dma_enable(void *pHandle)
{
iom_dma_state_t *pIOMState = (iom_dma_state_t*)pHandle;
uint32_t status = AM_HAL_STATUS_SUCCESS;

    // Enable submodule
    if(pIOMState->eInterfaceMode == AM_HAL_IOM_I2C_MODE)
    {
    IOMn(pIOMState->ui32Module)->SUBMODCTRL =
       _VAL2FLD(IOM0_SUBMODCTRL_SMOD1EN, 1) |
       _VAL2FLD(IOM0_SUBMODCTRL_SMOD0EN, 0) |
       _VAL2FLD(IOM0_SUBMODCTRL_SMOD1TYPE, 0);
    }else
    {
    IOMn(pIOMState->ui32Module)->SUBMODCTRL =
       _VAL2FLD(IOM0_SUBMODCTRL_SMOD1EN, 0) |
       _VAL2FLD(IOM0_SUBMODCTRL_SMOD0EN, 1) |
       _VAL2FLD(IOM0_SUBMODCTRL_SMOD1TYPE, 0);
    }

 
    // Initialize the DMA Trigger Setting
    //
    // DMATRIG, set DTHREN and/or DCMDCMPEN.
    // Note - it is recommended that DTHREN always be set.
    //
    IOMn(pIOMState->ui32Module)->DMATRIGEN = _VAL2FLD(IOM0_DMATRIGEN_DTHREN, 1);
 
    if (status == AM_HAL_STATUS_SUCCESS)
    {
        pIOMState->prefix.s.bEnable = true;
    }

    //
    // We're done, return the status.
    //
    return status;
}


//----------------------------------------------------------------------------//
//  IOM Disable
//----------------------------------------------------------------------------//
uint32_t iom_dma_disable(void *pHandle)
{
iom_dma_state_t *pIOMState = (iom_dma_state_t*)pHandle;


  if (!pIOMState->prefix.s.bEnable)
  {
    return AM_HAL_STATUS_SUCCESS;
  }

  //
  // Disable the submodules
  //
  IOMn(pIOMState->ui32Module)->SUBMODCTRL_b.SMOD0EN = 0;
  IOMn(pIOMState->ui32Module)->SUBMODCTRL_b.SMOD1EN = 0;

  pIOMState->prefix.s.bEnable = false;

  //
  // Return the status.
  //
  return AM_HAL_STATUS_SUCCESS;
}


//----------------------------------------------------------------------------//
//  Start DMA transfer
//----------------------------------------------------------------------------//
uint32_t iom_dma_start(void *pHandle, iom_dma_transfer_t *psTransaction)
{
iom_dma_state_t *pIOMState = (iom_dma_state_t *)pHandle;
uint32_t ui32Module = pIOMState->ui32Module;
TDMACommand dma_cmd;  

    if ( (psTransaction->ui32NumBytes == 0) && 
         (psTransaction->eDirection != AM_HAL_IOM_TX))
    {
        //
        // Only TX is supported for 0-length transactions. A 0-length
        // transfer presumes that only an offset value is being written.
        return AM_HAL_STATUS_INVALID_ARG; 
    }

    //
    //! user callback
    //
    user_callbacks[ui32Module] = psTransaction->pCallback;
    callback_cntx[ui32Module] = psTransaction->pCallbackCtxt;

    //
    //Check number of bytes and set pop_last_word if necessarly
    //
    pop_tail = false;
    if(pIOMState->eInterfaceMode == AM_HAL_IOM_I2C_MODE)
    {
    if(psTransaction->ui32NumBytes%4 > 0)pop_tail = true;
    }

    //
    // Initialize the DMA state machine (clear the DMACPL flag).
    //
    IOMn(ui32Module)->DMASTAT = 0;
    IOMn(ui32Module)->DMACFG = 0;
 
    //
    // Set interrupts
    //
    IOM_DMA_SET_INTEN(ui32Module, IOM_DMA_INT_DMAMODE);
  
    //AM_REGn(IOM, ui32Module, INTCLR) = AM_HAL_IOM_INT_ALL;
     
    //Reset FIFO errors
    if(IOMn(ui32Module)->FIFOPTR_b.FIFO1SIZ > 0)
    {
    IOMn(ui32Module)->FIFOCTRL_b.FIFORSTN = 0;
    IOMn(ui32Module)->FIFOCTRL_b.FIFORSTN = 1;

    while(IOMn(ui32Module)->FIFOPTR_b.FIFO1SIZ > 0)
      {
      IOMn(ui32Module)->FIFOPOP = 0;
      }
    }

    IOMn(ui32Module)->FIFOTHR_b.FIFORTHR = 16;
    IOMn(ui32Module)->FIFOTHR_b.FIFOWTHR = 16;
    IOMn(ui32Module)->FIFOCTRL_b.POPWR = 1;
    IOMn(ui32Module)->DMATRIGEN_b.DTHREN = 1;
    IOMn(ui32Module)->DMATRIGEN_b.DCMDCMPEN = 1;
     
    //Total count
    IOMn(ui32Module)->DMATOTCOUNT_b.TOTCOUNT = psTransaction->ui32NumBytes;
   
    //Target address
    if(psTransaction->eDirection == AM_HAL_IOM_TX)
      {
      IOMn(ui32Module)->DMACFG_b.DMADIR = 1;  
      IOMn(ui32Module)->DMATARGADDR = (uint32_t)psTransaction->pui32Buffer;  
      transaction_rx_buff = NULL;
      transaction_rx_size = 0;
      }else
      {
      IOMn(ui32Module)->DMATARGADDR = (uint32_t)psTransaction->pui32Buffer;
      transaction_rx_buff = psTransaction->pui32Buffer;
      transaction_rx_size = psTransaction->ui32NumBytes;
      }

    //Offset (&command & address)
    IOMn(ui32Module)->OFFSETHI = 0;

    //Device address
    IOMn(ui32Module)->DEVCFG_b.DEVADDR = psTransaction->uPeerInfo.ui32I2CDevAddr;
   
    //DCX Line disabled
    IOMn(ui32Module)->DCX_b.DCXEN = 0;

    //Clock strech detection enabled
    IOMn(ui32Module)->DMACFG_b.DMAEN = 1;

    IOMn(ui32Module)->MI2CCFG_b.STRDIS = false;

    //Command word
    if(psTransaction->eDirection == AM_HAL_IOM_TX)
    {
    dma_cmd.CMD = 1; //WRITE
    }else
    {
    dma_cmd.CMD = 2; //READ
    }
    dma_cmd.OFFSETLO = 0;
    dma_cmd.OFFSETCNT = 0;
    dma_cmd.CMDSEL = psTransaction->uPeerInfo.ui32I2CDevAddr;
    dma_cmd.TSIZE = psTransaction->ui32NumBytes;
    dma_cmd.CONT = psTransaction->bContinue;
      
    //Transaction start
    IOMn(ui32Module)->CMD =  *(uint32_t*)&dma_cmd;  
   
      
    return AM_HAL_STATUS_SUCCESS;
}


//----------------------------------------------------------------------------//
//  POP last uncomplete word from FIFO
//  Read the incomplete tail word from receve fifo
//  The procedure fixes issues with clock streching slave devices
//----------------------------------------------------------------------------//
void iom_i2c_dma_pop_tail(uint32_t ui32Module)
{
volatile uint8_t *ptr_in, *ptr_out, tail_size;
uint32_t tail;

  while((IOMn(ui32Module)->DMASTAT_b.DMACPL == 0)
        &&(IOMn(ui32Module)->DMASTAT_b.DMATIP == 0))
    {

    };
  
  if(pop_tail)
  {
  pop_tail = false;
  if(transaction_rx_size == 0)return;
  
  if(IOMn(ui32Module)->DMATOTCOUNT_b.TOTCOUNT == 0)return;

  tail_size = transaction_rx_size%4;
  tail = IOMn(ui32Module)->FIFOPOP;
  IOMn(ui32Module)->FIFOPOP = 0;
  ptr_in = (uint8_t*)&tail;
  ptr_out = (uint8_t*)transaction_rx_buff;
  
  ptr_out += transaction_rx_size-transaction_rx_size%4;

  
  while(tail_size>0)
    {
    tail_size--;
    *(ptr_out++) = *(ptr_in++);
    }

  while(IOMn(ui32Module)->FIFOPTR_b.FIFO1SIZ > 0)
    {
    IOMn(ui32Module)->FIFOPOP = 0;
    }
  }
}


//----------------------------------------------------------------------------//
//  IOM interrupt service
//----------------------------------------------------------------------------//
void am_iomaster0_isr(void)
{
uint32_t ui32Status;

  ui32Status = IOMn(0)->INTSTAT;
  ui32Status &= IOMn(0)->INTEN;  //Enabled only
   
  IOMn(0)->INTCLR = ui32Status;  //Clear interrupts

  if(user_callbacks[0] == NULL) return; 

  (user_callbacks[0])(0,callback_cntx[0],ui32Status);

}

void am_iomaster1_isr(void)
{
uint32_t ui32Status;

   if(user_callbacks[1] == NULL) return;

  ui32Status = IOMn(1)->INTSTAT;
  ui32Status &= IOMn(1)->INTEN;  //Enabled only

  //Clear interupts
  IOMn(1)->INTCLR = AM_HAL_IOM_INT_ALL;  

  if(user_callbacks[1] == NULL) return;

  (user_callbacks[1])(1,callback_cntx[1],ui32Status);
}

void am_iomaster2_isr(void)
{
uint32_t ui32Status;

  ui32Status = IOMn(2)->INTSTAT;
  ui32Status &= IOMn(2)->INTEN;  //Enabled only

  //Clear interupts
  IOMn(2)->INTCLR = AM_HAL_IOM_INT_ALL;    

  if(user_callbacks[2] == NULL) return;

  (user_callbacks[2])(2,callback_cntx[2],ui32Status);
}

void am_iomaster3_isr(void)
{
uint32_t ui32Status;

  ui32Status = IOMn(3)->INTSTAT;
  ui32Status &= IOMn(3)->INTEN;  //Enabled only

  //Clear interupts
  IOMn(3)->INTCLR = AM_HAL_IOM_INT_ALL;    

  if(user_callbacks[3] == NULL) return;

  (user_callbacks[3])(3,callback_cntx[3],ui32Status);
}

void am_iomaster4_isr(void)
{
uint32_t ui32Status;

  ui32Status = IOMn(4)->INTSTAT;
  ui32Status &= IOMn(4)->INTEN;  //Enabled only

  //Clear interupts
  IOMn(4)->INTCLR = AM_HAL_IOM_INT_ALL;  

  if(user_callbacks[4] == NULL) return;

  (user_callbacks[4])(4,callback_cntx[4],ui32Status);
}

void am_iomaster5_isr(void)
{
uint32_t ui32Status;

  ui32Status = IOMn(5)->INTSTAT;
  ui32Status &= IOMn(5)->INTEN;  //Enabled only

  //Clear interupts
  IOMn(5)->INTCLR = AM_HAL_IOM_INT_ALL;

  if(user_callbacks[5] == NULL) return;  

  (user_callbacks[5])(5,callback_cntx[5],ui32Status);  
}

//----------------------------------------------------------------------------//
//  Enable interrupt
//----------------------------------------------------------------------------//
void iom_dma_int_enable(uint32_t iom_module)
{
  switch(iom_module)
  {
  case 0: NVIC_EnableIRQ(IOMSTR0_IRQn);
  break;
  case 1: NVIC_EnableIRQ(IOMSTR1_IRQn);
  break;
  case 2: NVIC_EnableIRQ(IOMSTR2_IRQn);
  break;
  case 3: NVIC_EnableIRQ(IOMSTR3_IRQn);
  break;
  case 4: NVIC_EnableIRQ(IOMSTR4_IRQn);
  break;
  case 5: NVIC_EnableIRQ(IOMSTR5_IRQn);
  }
}

//----------------------------------------------------------------------------//
//  Disable interrupt
//----------------------------------------------------------------------------//
void iom_dma_int_disable(uint32_t iom_module)
{
  switch(iom_module)
  {
  case 0: NVIC_DisableIRQ(IOMSTR0_IRQn);
  break;
  case 1: NVIC_DisableIRQ(IOMSTR1_IRQn);
  break;
  case 2: NVIC_DisableIRQ(IOMSTR2_IRQn);
  break;
  case 3: NVIC_DisableIRQ(IOMSTR3_IRQn);
  break;
  case 4: NVIC_DisableIRQ(IOMSTR4_IRQn);
  break;
  case 5: NVIC_DisableIRQ(IOMSTR5_IRQn);
  }
}




