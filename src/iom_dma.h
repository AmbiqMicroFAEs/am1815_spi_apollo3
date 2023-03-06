//*****************************************************************************
// Simple DMA Driver
// Apollo3
// Mariusz Lacina, Ambiq, 2023
//*****************************************************************************

#ifndef IOM_DMA_H
#define IOM_DMA_H

#include <stdint.h>
#include <stdbool.h>
#include <am_hal_global.h>
#include <am_hal_iom.h>

typedef void (*am_dma_iom_callback_t)(uint8_t IomModule, void *pCallbackCtxt, uint32_t transactionStatus);




//----------------------------------------------------------------------------//
//  IOM DMA state structure
//----------------------------------------------------------------------------//
typedef struct{
    am_hal_handle_prefix_t  prefix;
    //
    // Physical module number.
    //
    uint32_t                ui32Module;

    //
    // Interface mode.
    //
    am_hal_iom_mode_e       eInterfaceMode;

}iom_dma_state_t;


//----------------------------------------------------------------------------//
//  Configuration structure for the IOM DMA.
//----------------------------------------------------------------------------//
typedef struct
{
    //
    //! Select the interface mode, SPI or I2C
    //
    am_hal_iom_mode_e eInterfaceMode;

    //! SPI Mode.
    am_hal_iom_spi_mode_e eSpiMode;

    //
    //! Select the interface clock frequency
    //
    uint32_t ui32ClockFreq;

}iom_dma_config_t;




//----------------------------------------------------------------------------//
//  Transfer configuration structure for the IOM DMA.
//----------------------------------------------------------------------------//
typedef struct{

  union
  {
  //
  //! Chip enable (chip select) for this transaction on this device.
  //
      uint32_t ui32SpiChipSelect;
      uint32_t ui32I2CDevAddr;
  } uPeerInfo;

  //
  //! Number of bytes to transfer
  //
  uint32_t ui32NumBytes;

  //
  //! Transfer Direction (Transmit/Receive)
  //
  am_hal_iom_dir_e eDirection;  

  //
  //! Buffer
  //
  uint32_t *pui32Buffer;   

  //
  //! User callback
  //
  am_dma_iom_callback_t pCallback;

  //
  //! Callback context
  //
  void  *pCallbackCtxt;

  //
  // Continue - holds the SPI or I2C bus for multiple transactions.
  //
  bool     bContinue; 

}iom_dma_transfer_t;


//----------------------------------------------------------------------------//
//  Prototypes
//----------------------------------------------------------------------------//
uint32_t iom_dma_start(void *pHandle, iom_dma_transfer_t *psTransaction);
uint32_t iom_dma_disable(void *pHandle);
uint32_t iom_dma_enable(void *pHandle);
uint32_t iom_dma_configure(void *pHandle, iom_dma_config_t *psConfig);

void iom_dma_int_enable(uint32_t iom_module);
void iom_dma_int_disable(uint32_t iom_module);
void iom0_int_service(void);
uint32_t iom_dma_interrupt_service(void *pHandle, uint32_t ui32IntMask);
void iom_dma_reset(void *pHandle);

void iom_i2c_dma_pop_tail(uint32_t ui32Module);


//----------------------------------------------------------------------------//
//  DMA command
//----------------------------------------------------------------------------//
typedef struct{
  uint32_t  CMD       :5;
  uint32_t  OFFSETCNT :2;
  uint32_t  CONT      :1;
  uint32_t  TSIZE     :12;
  uint32_t  CMDSEL    :2;
  uint32_t  RSRVD22   :2;
  uint32_t  OFFSETLO  :8;
}TDMACommand;

//----------------------------------------------------------------------------//
//  Chip enable channels
//----------------------------------------------------------------------------//
typedef enum{
  IOMSPI_CS0 = 0,
  IOMSPI_CS1,
  IOMSPI_CS2,
  IOMSPI_CS3
}TCEChannels;


//----------------------------------------------------------------------------//
//  Interface limitations
//----------------------------------------------------------------------------//
#define AM_HAL_IOM_FIFO_SIZE_MAX        32
#define AM_HAL_IOM_MAX_OFFSETSIZE       3
#define AM_HAL_IOM_MAX_TXNSIZE_I2C      255


//----------------------------------------------------------------------------//
//  IOM Interrupts
//----------------------------------------------------------------------------//
#define AM_HAL_IOM_INT_CQERR            IOM0_INTEN_CQERR_Msk        // Error during command queue operations
#define AM_HAL_IOM_INT_CQUPD            IOM0_INTEN_CQUPD_Msk        // Command queue operation  performed a register write with the register address bit 0 set to 1.
#define AM_HAL_IOM_INT_CQPAUSED         IOM0_INTEN_CQPAUSED_Msk     // Command queue operation paused
#define AM_HAL_IOM_INT_DERR             IOM0_INTEN_DERR_Msk         // DMA error received
#define AM_HAL_IOM_INT_DCMP             IOM0_INTEN_DCMP_Msk         // DMA transfer complete
#define AM_HAL_IOM_INT_ARB              IOM0_INTEN_ARB_Msk          // Arbitration loss
#define AM_HAL_IOM_INT_STOP             IOM0_INTEN_STOP_Msk         // STOP command
#define AM_HAL_IOM_INT_START            IOM0_INTEN_START_Msk        // START command
#define AM_HAL_IOM_INT_ICMD             IOM0_INTEN_ICMD_Msk         // ILLEGAL command
#define AM_HAL_IOM_INT_IACC             IOM0_INTEN_IACC_Msk         // Illegal FIFO access
#define AM_HAL_IOM_INT_NAK              IOM0_INTEN_NAK_Msk          // I2C NAK
#define AM_HAL_IOM_INT_FOVFL            IOM0_INTEN_FOVFL_Msk        // Write FIFO overflow
#define AM_HAL_IOM_INT_FUNDFL           IOM0_INTEN_FUNDFL_Msk       // Read FIFO underflow
#define AM_HAL_IOM_INT_THR              IOM0_INTEN_THR_Msk          // FIFO threshold interrupt
#define AM_HAL_IOM_INT_CMDCMP           IOM0_INTEN_CMDCMP_Msk       // Command complete





#endif

