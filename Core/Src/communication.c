#include "communication.h"
#include "SI4432.h"
#include "spi.h"


uint8_t comm_init(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(15);
    SI44_Init(&hspi1, GPIOA, GPIO_PIN_4);
    HAL_Delay(15);
    uint8_t version;
    SI44_Read(0x00, &version, 1);

    /* General config */
    si44_config conf;
    conf.encoding_options = 0;
    conf.modulation_source = SI44_MODULATION_SOURCE_FIFO;    //Use FIFO mode
    conf.modulation_type = SI44_MODULATION_TYPE_GFSK;        //Modulation GFSK
    SI44_SetConfig(&conf);
    /* Packet handler config */
    si44_ph_config ph_conf;
    ph_conf.path = SI44_PH_PATH_TX_ONLY;
    ph_conf.preambule_length = 8;                //Preambule length in nibbles (8 * 4 -> 32 bits)
    ph_conf.sync = SI44_PH_SYNC_2;               //Use 2 bytes (3 and 4) length sync-word
    ph_conf.header = SI44_PH_HEADER_OFF;         //Don`t use header
    ph_conf.crc = SI44_PH_CRC_DATA_ONLY;         //Use CRC, computed over data only
    ph_conf.crc_type = SI44_PH_CRC_TYPE_IBM16;   //Use IBM16 as a CRC type
    SI44_SetPHConfig(&ph_conf);

    SI44_SetFrequency(433.95);               //Set up 433.95 MHz
    SI44_SetDataRate(2400);                  //Set up data rate as 2400 bit/sec
    SI44_SetFrequencyDeviation(2400);        //Set deviation to 2400 Hz (BH=1 for baud=2400
    SI44_SetTXPower(SI44_TX_POWER_11dBm);    //Set TX power to 11dBm (12.5 mW)

    SI44_Read(0x00, &version, 1);
    return version;
}