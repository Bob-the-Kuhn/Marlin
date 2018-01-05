#ifdef TARGET_LPC1768

#include "spi_api.h"
#include "spi_hardware.h"

void HAL_SPI_transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length);


namespace HAL {
namespace SPI {



int8_t max_logical_channel = -1;

LogicalChannel logical_channels[];

int8_t create_logical_spi_channel(int8_t spi_channel, pin_t cs_pin, bool cs_polarity, uint32_t rate, uint8_t spi_mode, bool bit_order) {
  max_logical_channel ++;
  logical_channels[max_logical_channel].spi_channel = spi_channel;    // 0: SW spi, 1: LCD SD card , 2+ system dependent
  logical_channels[max_logical_channel].SCK = -1;
  logical_channels[max_logical_channel].MOSI = -1;
  logical_channels[max_logical_channel].MISO = -1;
  logical_channels[max_logical_channel].chip_select = cs_pin;          // -1 - not used
  logical_channels[max_logical_channel].chip_select_polarity = cs_polarity;  // 0 - active low
  logical_channels[max_logical_channel].rate = rate;
  logical_channels[max_logical_channel].spi_mode = spi_mode;
  logical_channels[max_logical_channel].bit_order = bit_order;
  logical_channels[max_logical_channel].busy = false;
  return max_logical_channel;
} 

int8_t create_logical_spi_channel(pin_t SCK, pin_t MOSI, pin_t MISO, pin_t cs, bool cs_polarity, uint32_t rate, uint8_t spi_mode, bool bit_order);
  max_logical_channel ++;
  logical_channels[max_logical_channel].spi_channel = 0;    // 0: SW spi, 1: LCD SD card , 2+ system dependent
  logical_channels[max_logical_channel].SCK = SCK;
  logical_channels[max_logical_channel].MOSI = MOSI;
  logical_channels[max_logical_channel].MISO = MISO;
  logical_channels[max_logical_channel].chip_select = cs_pin;          // -1 - not used
  logical_channels[max_logical_channel].chip_select_polarity = cs_polarity;  // 0 - active low
  logical_channels[max_logical_channel].rate = rate;
  logical_channels[max_logical_channel].spi_mode = spi_mode;
  logical_channels[max_logical_channel].bit_order = bit_order;
  logical_channels[max_logical_channel].busy = false;
  return max_logical_channel;
} 


void read(uint8_t channel, uint8_t *buffer, uint32_t length) {
  HAL_SPI_transfer(channel, nullptr, buffer, length);
}

uint8_t read(uint8_t channel) {
  uint8_t buffer;
  HAL_SPI_transfer(channel, nullptr, &buffer, 1);
  return buffer;
}

void write(uint8_t channel, const uint8_t *buffer, uint32_t length) {
  HAL_SPI_transfer(channel, buffer, nullptr, length);
}

void write(uint8_t channel, uint8_t value) {
  HAL_SPI_transfer(channel, &value, nullptr, 1);
}

void transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length) {
  HAL_SPI_transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length);
}

uint8_t transfer(uint8_t channel, uint8_t value) {
  uint8_t buffer;
  HAL_SPI_transfer(channel, &value, &buffer, 1);
  return buffer;
}


#endif