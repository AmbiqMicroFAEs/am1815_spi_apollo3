//*****************************************************************************
// SIMPLIFIED IOM SPI DRIVER
// Apollo3 Blue
// Mariusz Lacina, Ambiq 2023 
//*****************************************************************************


#ifndef IOM_SPI_H
#define IOM_SPI_H

#include "iom_dma.h"

//*****************************************************************************
// IOM CS channel
//*****************************************************************************
#define     CS_CHANNEL  IOMSPI_CS2


uint32_t iom_spi_init(void);
uint32_t iom_spi_write(uint32_t chip_select, uint32_t *pbuf, uint32_t size, bool cont, bool block);
uint32_t iom_spi_read(uint32_t chip_select, uint32_t *pbuf, uint32_t size, bool cont, bool block);

#endif


