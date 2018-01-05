#ifndef _SPI_API_H_
#define _SPI_API_H_

#include <stdint.h>
#include "HAL_spi_pins.h"

namespace HAL {
namespace SPI {

/**
 * spi_channel
 *  0 - software SPI
 *  1 - hardware controller used for LCD's SD card
 *  2+ - system dependent
 */

int8_t create_logical_spi_channel(int8_t spi_channel, pin_t cs_pin, bool cs_polarity, uint32_t rate, uint8_t spi_mode, bool bit_order);
int8_t create_logical_spi_channel(pin_t SCK, pin_t MOSI, pin_t MISO, pin_t cs, bool cs_polarity, uint32_t rate, uint8_t spi_mode, bool bit_order);

void read(uint8_t channel, uint8_t *buffer, uint32_t length);
uint8_t read(uint8_t channel);

void write(uint8_t channel, const uint8_t *buffer, uint32_t length);
void write(uint8_t channel, uint8_t value);

void transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length);
uint8_t transfer(uint8_t channel, uint8_t value);


typedef struct {
  int8_t spi_channel;    // 0: SW spi, 1: LCD SD card , 2+ system dependent
  pin_t SCK;
  pin_t MOSI;
  pin_t MISO;
  pin_t chip_select;          // -1 - not used
  bool chip_select_polarity;  // 0 - active low
  uint32_t rate;
  uint8_t spi_mode;
  bool bit_order;             // 1 - MSB first
  bool busy;
} LogicalChannel;


}
}


#endif /* _SPI_API_H_ */