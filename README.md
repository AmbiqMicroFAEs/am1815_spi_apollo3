"am1815_spi_apollo3" 

Project description:
The basic functionality of the AM1815 is presented in this example. The RTC enters sleep mode and leaves it periodically every 3 seconds using the external interrupt EXTI input. The extern XTAL is used as the main oscillator.
The current time is printed over the UART. The IOM/DMA is used to communicate with the RTC.

Board configuration:


![am1815](https://user-images.githubusercontent.com/69169627/223063958-3fce7a23-26b5-4097-aca0-34b8d9f07a96.png)

Project folders structure:

![obraz](https://user-images.githubusercontent.com/69169627/223064288-2902054d-6ad8-431c-b1f8-288cc982788a.png)

Measured average current consumption of the AM1815 (67 nA/3.3V): 

![am1815_current](https://user-images.githubusercontent.com/69169627/223068565-69881698-59c2-4508-bc54-81cbb8bb5b45.png)

UART output:


![AM1815_uart](https://user-images.githubusercontent.com/69169627/223069148-75757b4b-b305-454b-82bd-cadfb6cf4d33.png)
