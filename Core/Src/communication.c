#include "communication.h"
#include "spi.h"
#include "usart.h"
#include <memory.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define WRTIE_REGISTER_MASK 0x80
#define READ_REGISTER_MASK 0x00

static inline void hal_spi_startTransmissionSignal(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

static inline void hal_spi_endTransmissionSignal(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

static inline bool hal_gpio_isIRQ(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
}

static inline void hal_gpio_shutdown(bool enable)
{
    GPIO_PinState value = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, value);
}

static inline void hal_gpio_setLed(bool enable)
{
    GPIO_PinState value = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, value);
}

static inline bool hal_gpio_isButtonPressed(void)
{
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET);
}

void hal_spi_writeRegister(uint8_t reg, uint8_t value)
{
    typedef union {
        uint8_t u8[2];
    } SpiData_t;

    reg |= WRTIE_REGISTER_MASK;
    SpiData_t data = {.u8 = {reg, value}};
    hal_spi_startTransmissionSignal();
    HAL_SPI_Transmit(&hspi1, data.u8, sizeof(data), 5);
    hal_spi_endTransmissionSignal();
}

uint8_t hal_spi_readRegister(uint8_t reg)
{
    typedef union {
        uint16_t u16;
        uint8_t u8[2];
    } SpiData_t;
    reg |= READ_REGISTER_MASK;
    SpiData_t data = {.u8 = {reg, 0}};
    SpiData_t result = {.u16 = 0};
    hal_spi_startTransmissionSignal();
    HAL_SPI_TransmitReceive(&hspi1, data.u8, result.u8, sizeof(data), 5);
    hal_spi_endTransmissionSignal();

    return result.u8[1];
}

static void si4432_initialize(void)
{
    // AN415 - Page 8
    // Turn on the radio by pulling down the PWRDN pin
    hal_gpio_shutdown(true);
    hal_gpio_shutdown(false);
    // Wait at least 15ms before any initialization SPI commands are sent to the
    // radio
    HAL_Delay(15);
    // Read interrupt status registers to clear the interrupt flags and release
    // NIRQ pin
    hal_spi_readRegister(0x03);
    hal_spi_readRegister(0x04);
}

static void si4432_softwareReset(void)
{
    // AN415 - Page 10
    // Write 0x80 to the Operating & Function Control1 register for SW reset
    hal_spi_writeRegister(0x07, 0x80);
    while (hal_gpio_isIRQ())
        ;
    // Read interrupt status registers to clear the interrupt flags and release
    // NIRQ pin
    hal_spi_readRegister(0x03);
    hal_spi_readRegister(0x04);
}

static void si4432_enableReceiver(void)
{
    /*enable receiver chain*/
    hal_spi_writeRegister(0x07, 0x05); // write 0x05 to the Operating Function Control 1 register
    // Enable two interrupts:
    //  a) one which shows that a valid packet received: 'ipkval'
    //  b) second shows if the packet received with incorrect CRC: 'icrcerror'
    hal_spi_writeRegister(0x05, 0x03); // write 0x03 to the Interrupt Enable 1 register
    hal_spi_writeRegister(0x06, 0x00); // write 0x00 to the Interrupt Enable 2 register
    // read interrupt status registers to release all pending interrupts
    hal_spi_readRegister(0x03); // read the Interrupt Status1 register
    hal_spi_readRegister(0x04); // read the Interrupt Status2 register
}

uint8_t comm_init(void)
{
    hal_gpio_setLed(true);

    si4432_initialize();
    si4432_softwareReset();

    /*set the physical parameters*/
    // set the center frequency to 915 MHz
    hal_spi_writeRegister(0x75, 0x75); // write 0x75 to the Frequency Band Select register
    hal_spi_writeRegister(0x76, 0xBB); // write 0xBB to the Nominal Carrier Frequency1 register
    hal_spi_writeRegister(0x77, 0x80); // write 0x80 to the Nominal Carrier Frequency0 register
    // set the desired TX data rate (9.6kbps)
    hal_spi_writeRegister(0x6E, 0x4E); // write 0x4E to the TXDataRate 1 register
    hal_spi_writeRegister(0x6F, 0xA5); // write 0xA5 to the TXDataRate 0 register
    hal_spi_writeRegister(0x70, 0x2C); // write 0x2C to the Modulation Mode Control 1 register

    // set the Tx deviation register (+-45kHz)
    hal_spi_writeRegister(0x72, 0x48);

    /*set the modem parameters according to the exel calculator(parameters: 9.6 kbps, deviation: 45 kHz, channel filter
     * BW: 102.2 kHz*/
    hal_spi_writeRegister(0x1C, 0x1E); // write 0x1E to the IF Filter Bandwidth register
    hal_spi_writeRegister(0x20, 0xD0); // write 0xD0 to the Clock Recovery Oversampling Ratio register
    hal_spi_writeRegister(0x21, 0x00); // write 0x00 to the Clock Recovery Offset 2 register
    hal_spi_writeRegister(0x22, 0x9D); // write 0x9D to the Clock Recovery Offset 1 register
    hal_spi_writeRegister(0x23, 0x49); // write 0x49 to the Clock Recovery Offset 0 register
    hal_spi_writeRegister(0x24, 0x00); // write 0x00 to the Clock Recovery Timing Loop Gain 1 register
    hal_spi_writeRegister(0x25, 0x24); // write 0x24 to the Clock Recovery Timing Loop Gain 0 register
    hal_spi_writeRegister(0x1D, 0x40); // write 0x40 to the AFC Loop Gearshift Override register
    hal_spi_writeRegister(0x1E, 0x0A); // write 0x0A to the AFC Timing Control register
    hal_spi_writeRegister(0x2A, 0x20); // write 0x20 to the AFC Limiter register

    /*set the packet structure and the modulation type*/
    // set the preamble length to 5bytes
    hal_spi_writeRegister(0x34, 0x0A); // write 0x0A to the Preamble Length register
    // set preamble detection threshold to 20bits
    hal_spi_writeRegister(0x35, 0x2A); // write 0x2A to the Preamble Detection Control  register

    // Disable header bytes; set variable packet length (the length of the payload is defined by the
    // received packet length field of the packet); set the synch word to two bytes long
    hal_spi_writeRegister(0x33, 0x02); // write 0x02 to the Header Control2 register

    // Set the sync word pattern to 0x2DD4
    hal_spi_writeRegister(0x36, 0x2D); // write 0x2D to the Sync Word 3 register
    hal_spi_writeRegister(0x37, 0xD4); // write 0xD4 to the Sync Word 2 register

    // enable the TX & RX packet handler and CRC-16 (IBM) check
    hal_spi_writeRegister(0x30, 0x8D); // write 0x8D to the Data Access Control register
    // Disable the receive header filters
    hal_spi_writeRegister(0x32, 0x00); // write 0x00 to the Header Control1 register
    // enable FIFO mode and GFSK modulation
    hal_spi_writeRegister(0x71, 0x63); // write 0x63 to the Modulation Mode Control 2 register

    /*set the GPIO's according to the RF switch */
    hal_spi_writeRegister(0x0C, 0x12); // write 0x12 to the GPIO1 Configuration(set the TX state)
    hal_spi_writeRegister(0x0D, 0x15); // write 0x15 to the GPIO2 Configuration(set the RX state)

    /*set the non-default Si443x registers*/
    // set AGC Override1 Register
    hal_spi_writeRegister(0x69, 0x60); // write 0x60 to the AGC Override1 register
    // set  cap. bank
    hal_spi_writeRegister(0x09, 0xD7); // write 0xD7 to the Crystal Oscillator Load Capacitance register

    hal_gpio_setLed(false);

    si4432_enableReceiver();

    return 0;
}

void comm_run(void)
{
    volatile uint8_t itStatus1;
    volatile uint8_t itStatus2;
    uint8_t length;
    uint8_t temp8;
    uint8_t payload[10];
    uint8_t string[32];
    uint8_t rssi = 0;

    if (hal_gpio_isButtonPressed())
    {
        HAL_Delay(1);
        while (hal_gpio_isButtonPressed())
            ;
        sprintf((char *)string, "Button pressed \r\n");
        HAL_UART_Transmit(&huart2, string, strlen((const char *)string), 5);
        // disable the receiver chain (but keep the XTAL running to have shorter TX on time!)
        hal_spi_writeRegister(0x07, 0x01); // write 0x01 to the Operating Function Control 1 register
        // turn on the LED to show the packet transmission
        hal_gpio_setLed(true);
        /*SET THE CONTENT OF THE PACKET*/
        // set the length of the payload to 8bytes
        hal_spi_writeRegister(0x3E, 8); // write 8 to the Transmit Packet Length register
        // fill the payload into the transmit FIFO
        hal_spi_writeRegister(0x7F, 0x42); // write 0x42 ('B') to the FIFO Access register
        hal_spi_writeRegister(0x7F, 0x55); // write 0x55 ('U') to the FIFO Access register
        hal_spi_writeRegister(0x7F, 0x54); // write 0x54 ('T') to the FIFO Access register
        hal_spi_writeRegister(0x7F, 0x54); // write 0x54 ('T') to the FIFO Access register
        hal_spi_writeRegister(0x7F, 0x4F); // write 0x4F ('O') to the FIFO Access register
        hal_spi_writeRegister(0x7F, 0x4E); // write 0x4E ('N') to the FIFO Access register
        hal_spi_writeRegister(0x7F, 0x31); // write 0x31 ('1') to the FIFO Access register
        hal_spi_writeRegister(0x7F, 0x0D); // write 0x0D (CR) to the FIFO Access register

        // Disable all other interrupts and enable the packet sent interrupt only.
        // This will be used for indicating the successfull packet transmission for the MCU
        hal_spi_writeRegister(0x05, 0x04); // write 0x04 to the Interrupt Enable 1 register
        hal_spi_writeRegister(0x06, 0x00); // write 0x03 to the Interrupt Enable 2 register
        // Read interrupt status regsiters. It clear all pending interrupts and the nIRQ pin goes back to high.
        itStatus1 = hal_spi_readRegister(0x03); // read the Interrupt Status1 register
        itStatus2 = hal_spi_readRegister(0x04); // read the Interrupt Status2 register

        /*enable transmitter*/
        // The radio forms the packet and send it automatically.
        hal_spi_writeRegister(0x07, 0x09); // write 0x09 to the Operating Function Control 1 register

        /*wait for the packet sent interrupt*/
        // The MCU just needs to wait for the 'ipksent' interrupt.
        while (hal_gpio_isIRQ())
            ;
        // read interrupt status registers to release the interrupt flags
        itStatus1 = hal_spi_readRegister(0x03); // read the Interrupt Status1 register
        itStatus2 = hal_spi_readRegister(0x04); // read the Interrupt Status2 register

        // wait a bit for showing the LED a bit longer
        HAL_Delay(2);
        // turn off the LED
        hal_gpio_setLed(false);

        // after packet transmission set the interrupt enable bits according receiving mode
        // Enable two interrupts:
        //  a) one which shows that a valid packet received: 'ipkval'
        //  b) second shows if the packet received with incorrect CRC: 'icrcerror'
        hal_spi_writeRegister(0x05, 0x03); // write 0x03 to the Interrupt Enable 1 register
        hal_spi_writeRegister(0x06, 0x00); // write 0x00 to the Interrupt Enable 2 register
        // read interrupt status registers to release all pending interrupts
        itStatus1 = hal_spi_readRegister(0x03); // read the Interrupt Status1 register
        itStatus2 = hal_spi_readRegister(0x04); // read the Interrupt Status2 register

        /*enable receiver chain again*/
        hal_spi_writeRegister(0x07, 0x05);
        sprintf((char *)string, "Data Sent... \r\n");
        HAL_UART_Transmit(&huart2, string, strlen((const char *)string), 5);
    }

    // wait for the interrupt event
    // If it occurs, then it means a packet received or CRC error happened
    if (hal_gpio_isIRQ() == false)
    {
        rssi = hal_spi_readRegister(0x26);
        sprintf((char *)string, "Receiver: %d \r\n", rssi);
        HAL_UART_Transmit(&huart2, string, strlen((const char *)string), 5);
        // disable the receiver chain
        hal_spi_writeRegister(0x07, 0x01); // write 0x01 to the Operating Function Control 1 register
        // read interrupt status registers
        itStatus1 = hal_spi_readRegister(0x03); // read the Interrupt Status1 register
        itStatus2 = hal_spi_readRegister(0x04); // read the Interrupt Status2 register

        /*CRC Error interrupt occured*/
        if ((itStatus1 & 0x01) == 0x01)
        {
            // reset the RX FIFO
            hal_spi_writeRegister(0x08, 0x02); // write 0x02 to the Operating Function Control 2 register
            hal_spi_writeRegister(0x08, 0x00); // write 0x00 to the Operating Function Control 2 register
            // blink LED to show the error
            for (uint32_t i = 0; i < 20; ++i)
            {
                hal_gpio_setLed(true);
                HAL_Delay(50);
                hal_gpio_setLed(false);
            }
            sprintf((char *)string, "CRC Error \r\n");
            HAL_UART_Transmit(&huart2, string, strlen((const char *)string), 5);
        }

        if (itStatus2)
        {
        }

        /*packet received interrupt occured*/
        if ((itStatus1 & 0x02) == 0x02)
        {
            // Read the length of the received payload
            length = hal_spi_readRegister(0x4B); // read the Received Packet Length register
            // check whether the received payload is not longer than the allocated buffer in the MCU
            if (length < 11)
            {
                // Get the reeived payload from the RX FIFO
                for (temp8 = 0; temp8 < length; temp8++)
                {
                    payload[temp8] = hal_spi_readRegister(0x7F); // read the FIFO Access register
                }

                // check whether the acknowledgement packet received
                if (length == 4)
                {
                    if (memcmp(&payload[0], "ACK", 3) == 0)
                    {
                        sprintf((char *)string, "Acknowledgment successfull \r\n");
                        HAL_UART_Transmit(&huart2, string, strlen((const char *)string), 5);
                    }
                }

                // check whether an expected packet received, this should be acknowledged
                if (length == 8)
                {
                    if (memcmp(&payload[0], "BUTTON1", 7) == 0)
                    {
                        sprintf((char *)string, "Packet received....Ack... \r\n");
                        HAL_UART_Transmit(&huart2, string, strlen((const char *)string), 5);

                        /*send back an acknowledgement*/
                        // turn on LED1 to show packet transmission
                        hal_gpio_setLed(true);
                        // The Tx deviation register has to set according to the deviation before every transmission
                        // (+-45kHz)
                        hal_spi_writeRegister(0x72, 0x48); // write 0x48 to the Frequency Deviation register
                        /*set packet content*/
                        // set the length of the payload to 4bytes
                        hal_spi_writeRegister(0x3E, 4); // write 4 to the Transmit Packet Length register
                        // fill the payload into the transmit FIFO
                        hal_spi_writeRegister(0x7F, 0x41); // write 0x41 ('A') to the FIFO Access register
                        hal_spi_writeRegister(0x7F, 0x43); // write 0x43 ('C') to the FIFO Access register
                        hal_spi_writeRegister(0x7F, 0x4B); // write 0x4B ('K') to the FIFO Access register
                        hal_spi_writeRegister(0x7F, 0x0D); // write 0x0D (CR) to the FIFO Access register

                        // Disable all other interrupts and enable the packet sent interrupt only.
                        // This will be used for indicating the successfull packet transmission for the MCU
                        hal_spi_writeRegister(0x05, 0x04); // write 0x04 to the Interrupt Enable 1 register
                        hal_spi_writeRegister(0x06, 0x00); // write 0x00 to the Interrupt Enable 2 register
                        // Read interrupt status regsiters. It clear all pending interrupts and the nIRQ pin goes back
                        // to high.
                        itStatus1 = hal_spi_readRegister(0x03); // read the Interrupt Status1 register
                        itStatus2 = hal_spi_readRegister(0x04); // read the Interrupt Status2 register

                        /*enable transmitter*/
                        // The radio forms the packet and send it automatically.
                        hal_spi_writeRegister(0x07, 0x09); // write 0x09 to the Operating Function Control 1 register

                        /*wait for the packet sent interrupt*/
                        // The MCU just needs to wait for the 'ipksent' interrupt.
                        while (hal_gpio_isIRQ() == true)
                            ;
                        // read interrupt status registers to release the interrupt flags
                        itStatus1 = hal_spi_readRegister(0x03); // read the Interrupt Status1 register
                        itStatus2 = hal_spi_readRegister(0x04); // read the Interrupt Status2 register

                        // wait a bit for showing the LED a bit longer
                        HAL_Delay(250);
                        // turn off the LED
                        hal_gpio_setLed(false);

                        // after packet transmission set the interrupt enable bits according receiving mode
                        // Enable two interrupts:
                        //  a) one which shows that a valid packet received: 'ipkval'
                        //  b) second shows if the packet received with incorrect CRC: 'icrcerror'
                        hal_spi_writeRegister(0x05, 0x03); // write 0x03 to the Interrupt Enable 1 register
                        hal_spi_writeRegister(0x06, 0x00); // write 0x00 to the Interrupt Enable 2 register
                        // read interrupt status registers to release all pending interrupts
                        itStatus1 = hal_spi_readRegister(0x03); // read the Interrupt Status1 register
                        itStatus2 = hal_spi_readRegister(0x04); // read the Interrupt Status2 register
                        // set the Frequency Deviation register according to the AFC limiter
                        hal_spi_writeRegister(0x72, 0x1F); // write 0x1F to the Frequency Deviation register
                    }
                }
            }
        }
        // reset the RX FIFO
        hal_spi_writeRegister(0x08, 0x02); // write 0x02 to the Operating Function Control 2 register
        hal_spi_writeRegister(0x08, 0x00); // write 0x00 to the Operating Function Control 2 register
        // enable the receiver chain again
        hal_spi_writeRegister(0x07, 0x05); // write 0x05 to the Operating Function Control 1 register
    }
}