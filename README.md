# Drivers-STM32F4
Library containing drivers for handling peripherals on the STM32F407G-DISC1 board (stm32f407vg processor). The library was built on the basis of the Datasheet and Reference manual and the online course.

## Drivers include:
- GPIO drivers
- SPI drivers
- I2C drivers
- UART drivers

### GPIO Drivers APIs requirements:
- ***GPIO initialization***
- ***Enable/Disable GPIO port clock***
- ***Read From a GPIO Pin***
- ***Write to GPIO Pin***
- ***Configure alternate functionality***
- ***Interrupt Handling***

### SPI Drivers APIs requirements:
- ***SPI Initialization / peripheral clock control***
- ***SPI TX***
- ***SPI RX***
- ***SPI Interrupt configure & handling***
- ***Other SPI management APIs***

### I2C Drivers APIs requirements:
- ***I2C Initialization***
- ***I2C Master TX***
- ***I2C Master RX***
- ***I2C Slave Tx***
- ***I2C Slave Rx***
- ***I2C Error Interrupt handling***
- ***I2C Event Interrupt handling***