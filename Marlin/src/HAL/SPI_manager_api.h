#ifndef _SPI_API_H_
#define _SPI_API_H_

#include <stdint.h>
#include "HAL_spi_pins.h"

namespace HAL {
namespace SPI {

/**
 * create_logical_spi_channel 
 *
 *  returns logical channel number, -1 if out of channels
 *
 *  spi_channel
 *   1 - hardware controller used for LCD's SD card
 *   2+ - hardware controller, system dependent
 *
 *  cs_pin -  chip select pin, -1 if app will control chip select
 *  cs_polarity - 1 if active high, 0 if active lower_bound
 *  frequency in Hz for the SPI clock
 *  spi_mode - 0-3
 *  SCK (software SPI only)
 *  MOSI (software SPI only)
 *  MISO (software SPI only)
 *
 */


// hardware SPI
int8_t create_logical_spi_channel(int8_t spi_channel, pin_t cs_pin, bool cs_polarity, uint32_t frequency, uint8_t spi_mode);

// software SPI
int8_t create_logical_spi_channel(pin_t SCK, pin_t MOSI, pin_t MISO, pin_t cs, bool cs_polarity, uint32_t frequency, uint8_t spi_mode);

void read(uint8_t channel, uint8_t *buffer, uint32_t length);
uint8_t read(uint8_t channel);

void write(uint8_t channel, const uint8_t *buffer, uint32_t length);
void write(uint8_t channel, uint8_t value);

void transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length);
uint8_t transfer(uint8_t channel, uint8_t value);

// change SPI frequency for specified channel
void set_frequency(int8_t channel, uint32_t frequency);


}
}


#endif /* _SPI_API_H_ */