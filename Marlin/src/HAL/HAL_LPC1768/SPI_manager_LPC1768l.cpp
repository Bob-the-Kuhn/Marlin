#ifdef TARGET_LPC1768

#include "../spi_api.h"

#include <lpc17xx_ssp.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_gpio.h>
#include "lpc17xx_clkpwr.h"

#include "../spi_api.h



/** @brief SSP configuration structure */
/**
typedef struct {
	uint32_t Databit; 		//  Databit number, should be SSP_DATABIT_x,
							where x is in range from 4 - 16 
	uint32_t CPHA;			//  Clock phase, should be:
    - SSP_CPHA_FIRST: first clock edge
							- SSP_CPHA_SECOND: second clock edge 
	uint32_t CPOL;			//  Clock polarity, should be:
							- SSP_CPOL_HI: high level
							- SSP_CPOL_LO: low level 
	uint32_t Mode;			//  SSP mode, should be:
							- SSP_MASTER_MODE: Master mode
							- SSP_SLAVE_MODE: Slave mode 
	uint32_t FrameFormat;	//  Frame Format:
							- SSP_FRAME_SPI: Motorola SPI frame format
							- SSP_FRAME_TI: TI frame format
							- SSP_FRAME_MICROWIRE: National Microwire frame format 
	uint32_t ClockRate;		//  Clock rate,in Hz 
} SSP_CFG_Type;
*/

/**
 * @brief SSP Transfer Type definitions
 */
/**
typedef enum {
	SSP_TRANSFER_POLLING = 0,	// < Polling transfer 
	SSP_TRANSFER_INTERRUPT		// < Interrupt transfer 
} SSP_TRANSFER_Type;
*/

/**
 * @brief SPI Data configuration structure definitions
 */
/**
typedef struct {
	void *tx_data;				// < Pointer to transmit data 
	uint32_t tx_cnt;			// < Transmit counter 
	void *rx_data;				// < Pointer to transmit data 
	uint32_t rx_cnt;			// < Receive counter 
	uint32_t length;			// < Length of transfer data 
	uint32_t status;			// < Current status of SSP activity 
} SSP_DATA_SETUP_Type;
*/







extern "C" void SSP0_IRQHandler(void);
extern "C" void SSP1_IRQHandler(void);

namespace HAL {
namespace SPI {
  
/* spi_channel vs.  hardware_channels :
 *   0: SW SPI      0: SW SPI
 *   1:             1: clk(0_15), mosi(0_28), miso(0_17), SSP0
 *   2:             2: clk(0_7),  mosi(0_9),  miso(0_8),  SSP1
 */

/*
 * Defines the Hardware setup for an SPI channel
 * The pins and (if applicable) the Hardware Peripheral
 *
 */

struct HardwareChannel {
  LPC_SSP_TypeDef *peripheral;
  IRQn_Type IRQn;
  uint8_t clk_port;
  uint8_t clk_pin;
  uint8_t mosi_port;
  uint8_t mosi_pin;
  uint8_t miso_port;
  uint8_t miso_pin;
  SSP_DATA_SETUP_Type xfer_config;
  volatile FlagStatus xfer_complete;
  bool initialised;
  volatile bool in_use;
} hardware_channel[3] = {
    {0, 0, 0, 0, 0, 0, 0, 0, { nullptr, 0, nullptr, 0, 0, SSP_STAT_DONE }, RESET, false, false},
    {LPC_SSP0, SSP0_IRQn, 0, 15, 0, 18, 0, 17, { nullptr, 0, nullptr, 0, 0, SSP_STAT_DONE }, RESET, false, false},
    {LPC_SSP1, SSP1_IRQn, 0, 7,  0, 9,  0, 8 , { nullptr, 0, nullptr, 0, 0, SSP_STAT_DONE }, RESET, false, false}
};


struct LogicalChannel {
  int8_t spi_channel;    // 0: SW spi, 1: LCD SD card , 2+ system dependent
  pin_t SCK;
  pin_t MOSI;
  pin_t MISO;
  pin_t chip_select;          // -1 - not used
  bool chip_select_polarity;  // 0 - active low
  uint32_t frequency;
  uint32_t CR0;
  uint32_t CPSR;
  uint8_t spi_mode;
  bool bit_order;             // 1 - MSB first
  bool busy;
} logical_channel[];



//Internal functions

extern "C" void ssp_irq_handler(uint8_t HW_CHANNEL);


void chip_select_active(int8_t channel) {
  if(logical_channel[channel].chip_select >= 0) { 
    if(logical_channel[channel].chip_select_polarity == SignalPolarity::ACTIVE_HIGH) 
      GPIO_SetValue(LPC1768_pin_port(logical_channel[channel].chip_select), (1 << LPC1768_pin_pin(logical_channel[channel].chip_select)));
    else 
      GPIO_ClearValue(LPC1768_pin_port(logical_channel[channel].chip_select), (1 << LPC1768_pin_pin(logical_channel[channel].chip_select)));
  }  
}

void chip_select_inactive(int8_t channel) {
  if(logical_channel[channel].chip_select >= 0) { 
    if(logical_channel[channel].chip_select_polarity == SignalPolarity::ACTIVE_HIGH) 
      GPIO_ClearValue(LPC1768_pin_port(logical_channel[channel].chip_select), (1 << LPC1768_pin_pin(logical_channel[channel].chip_select)));
    else 
      GPIO_SetValue(LPC1768_pin_port(logical_channel[channel].chip_select), (1 << LPC1768_pin_pin(logical_channel[channel].chip_select)));
  }    
}


/*
 * SPI API Implementation
 */
bool initialise_pins(uint8_t channel) {



  PINSEL_CFG_Type pin_cfg;
  pin_cfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  pin_cfg.Pinmode = PINSEL_PINMODE_PULLUP;

  if(HW_CHANNEL.initialised == false && logical_channel[chan].spi_channel) {  // only do this for the hardware SPIs
    pin_cfg.Funcnum = 2; //ssp (spi) function
    pin_cfg.Portnum = HW_CHANNEL(channel).clk_port;
    pin_cfg.Pinnum = HW_CHANNEL.clk_pin;
    PINSEL_ConfigPin(&pin_cfg); //clk

    pin_cfg.Portnum = HW_CHANNEL.miso_port;
    pin_cfg.Pinnum = HW_CHANNEL.miso_pin;
    PINSEL_ConfigPin(&pin_cfg); //miso

    pin_cfg.Portnum = HW_CHANNEL.mosi_port;
    pin_cfg.Pinnum = HW_CHANNEL.mosi_pin;
    PINSEL_ConfigPin(&pin_cfg); //mosi

    SSP_Init(HW_CHANNEL.peripheral, &logical_channel->config);
    SSP_Cmd(HW_CHANNEL.peripheral, ENABLE);

    HW_CHANNEL.initialised = true;

    //NVIC_SetPriority(HW_CHANNEL.IRQn, NVIC_EncodePriority(0, 3, 0)); //Very Low priority
    //NVIC_EnableIRQ(HW_CHANNEL.IRQn);
  }

  if (logical_channel[channel].chip_select >= 0) {
    pin_cfg.Portnum = LPC1768_pin_port(logical_channel[channel].chip_select);
    pin_cfg.Pinnum = LPC1768_pin_pin(logical_channel[channel].chip_select);
    pin_cfg.Pinmode = logical_channel[channel].chip_select_polarity ==  SignalPolarity::ACTIVE_LOW ? PINSEL_PINMODE_PULLUP : PINSEL_PINMODE_PULLDOWN;
    pin_cfg.Funcnum = 0; //gpio function
    PINSEL_ConfigPin(&pin_cfg); // chip select

    GPIO_SetDir(LPC1768_pin_port(logical_channel[channel].chip_select), (1 << LPC1768_pin_pin(logical_channel[channel].chip_select), 1);
    GPIO_SetValue(LPC1768_pin_port(logical_channel[channel].chip_select), (1 << LPC1768_pin_pin(logical_channel[channel].chip_select)));
  }
  
  if (logical_channel[channel].spi_channel ==0) {  // setup software spi pins
    pin_cfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    pin_cfg.Pinmode = PINSEL_PINMODE_PULLUP;
    pin_cfg.Funcnum = 0; //gpio function
    
    pin_cfg.Portnum = LPC1768_pin_port(logical_channel[channel].SCK);
    pin_cfg.Pinnum = LPC1768_pin_pin(logical_channel[channel].SCK);
    PINSEL_ConfigPin(&pin_cfg);
    if (???) 
      GPIO_SetValue(LPC1768_pin_port(logical_channel[channel].SCK), (1 << LPC1768_pin_pin(logical_channel[channel].SCK)));
    else  
      GPIO_ClearValue(LPC1768_pin_port(logical_channel[channel].SCK), (1 << LPC1768_pin_pin(logical_channel[channel].SCK)));
    GPIO_SetDir(LPC1768_pin_port(logical_channel[channel].SCK), (1 << LPC1768_pin_pin(logical_channel[channel].SCK), 1));
    

    pin_cfg.Portnum = LPC1768_pin_port(logical_channel[channel].MOSI);
    pin_cfg.Pinnum = LPC1768_pin_pin(logical_channel[channel].MOSI);
    PINSEL_ConfigPin(&pin_cfg);
    GPIO_ClearValue(LPC1768_pin_port(logical_channel[channel].MOSI), (1 << LPC1768_pin_pin(logical_channel[channel].MOSI)));
    GPIO_SetDir(LPC1768_pin_port(logical_channel[channel].MOSI), (1 << LPC1768_pin_pin(logical_channel[channel].MOSI), 1));


    pin_cfg.Portnum = LPC1768_pin_port(logical_channel[channel].MISO);
    pin_cfg.Pinnum = LPC1768_pin_pin(logical_channel[channel].MISO);
    PINSEL_ConfigPin(&pin_cfg);
    GPIO_SetDir(LPC1768_pin_port(logical_channel[channel].MISO), (1 << LPC1768_pin_pin(logical_channel[channel].MISO), 0));
  }  
  return true;
}


void set_frequency(uint8_t channel, uint32_t frequency) {
  LogicalChannel* logical_channel = get_logical_channel(channel);
  if(logical_channel == nullptr) return;

  SSP_Cmd(logical_channel->HW_CHANNEL.peripheral, DISABLE);
  uint32_t prescale, cr0_div, cmp_clk, ssp_clk;

  if (logical_channel->HW_CHANNEL.peripheral == LPC_SSP0){
    ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP0);
  } else if (logical_channel->HW_CHANNEL.peripheral == LPC_SSP1) {
    ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP1);
  } else {
    return;
  }
  //find the closest clock divider / prescaler
  cr0_div = 0;
  cmp_clk = 0xFFFFFFFF;
  prescale = 2;
  while (cmp_clk > frequency) {
    cmp_clk = ssp_clk / ((cr0_div + 1) * prescale);
    if (cmp_clk > frequency) {
      cr0_div++;
      if (cr0_div > 0xFF) {
        cr0_div = 0;
        prescale += 2;
      }
    }
  }

  logical_channel->HW_CHANNEL.peripheral->CR0 &= (~SSP_CR0_SCR(0xFF)) & SSP_CR0_BITMASK;
  logical_channel->HW_CHANNEL.peripheral->CR0 |= (SSP_CR0_SCR(cr0_div)) & SSP_CR0_BITMASK;
  logical_channel->CR0 = logical_channel->HW_CHANNEL.peripheral->CR0; // preserve for restore

  logical_channel->HW_CHANNEL.peripheral->CPSR = prescale & SSP_CPSR_BITMASK;
  logical_channel->CPSR = logical_channel->HW_CHANNEL.peripheral->CPSR; // preserve for restore

  logical_channel->config.Clockfrequency = ssp_clk / ((cr0_div + 1) * prescale);

  SSP_Cmd(logical_channel->HW_CHANNEL.peripheral, ENABLE);
}

//////////////////////////////////////////////////////


void get_frequency_settings(uint8_t channel) { 
  if (logical_channel[channel].spi_channel) {   // hardware SPI
    if (hardware_channel[logical_channel[channel].spi_channel -1].peripheral == LPC_SSP0){
      ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP0);
    } else if (logical_channel->HW_CHANNEL.peripheral == LPC_SSP1) {
      ssp_clk = CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP1);
    } else {
      return;
    }
    //find the closest clock divider / prescaler
    cr0_div = 0;
    cmp_clk = 0xFFFFFFFF;
    prescale = 2;
    while (cmp_clk > frequency) {
      cmp_clk = ssp_clk / ((cr0_div + 1) * prescale);
      if (cmp_clk > frequency) {
        cr0_div++;
        if (cr0_div > 0xFF) {
          cr0_div = 0;
          prescale += 2;
        }
      }
    }
  }
  else {  // software SPI - CR0 & CPSR not used
    cr0_div = 0;
    prescale = 0;
  } 
  
  logical_channel[channel].CR0 = cr0_div;
  logical_channel[channel].CPSR = prescale;
}

int8_t num_logical_channels = -1;

LogicalChannel logical_channel[];

int8_t create_logical_spi_channel(int8_t spi_channel, pin_t cs_pin, bool cs_polarity, uint32_t frequency, uint8_t spi_mode, bool bit_order) {
  num_logical_channels ++;
  logical_channel[num_logical_channels].spi_channel = spi_channel;    // 0: SW spi, 1: LCD SD card , 2+ system dependent
  logical_channel[num_logical_channels].SCK = -1;
  logical_channel[num_logical_channels].MOSI = -1;
  logical_channel[num_logical_channels].MISO = -1;
  logical_channel[num_logical_channels].chip_select = cs_pin;          // -1 - not used
  logical_channel[num_logical_channels].chip_select_polarity = cs_polarity;  // 0 - active low
  logical_channel[num_logical_channels].frequency = frequency;
  logical_channel[num_logical_channels].spi_mode = spi_mode;
  logical_channel[num_logical_channels].bit_order = bit_order;
  logical_channel[num_logical_channels].busy = false;
  get_frequency_settings(num_logical_channels);  // get settings for CR0 & CPSR
  return num_logical_channels;
} 

int8_t create_logical_spi_channel(pin_t SCK, pin_t MOSI, pin_t MISO, pin_t cs, bool cs_polarity, uint32_t frequency, uint8_t spi_mode, bool bit_order);
  num_logical_channels ++;
  logical_channel[num_logical_channels].spi_channel = 0;    // 0: SW spi, 1: LCD SD card , 2+ system dependent
  logical_channel[num_logical_channels].SCK = SCK;
  logical_channel[num_logical_channels].MOSI = MOSI;
  logical_channel[num_logical_channels].MISO = MISO;
  logical_channel[num_logical_channels].chip_select = cs_pin;          // -1 - not used
  logical_channel[num_logical_channels].chip_select_polarity = cs_polarity;  // 0 - active low
  logical_channel[num_logical_channels].frequency = frequency;
  logical_channel[num_logical_channels].spi_mode = spi_mode;
  logical_channel[num_logical_channels].bit_order = bit_order;
  logical_channel[num_logical_channels].CR0 = 0;  // not used on software SPI
  logical_channel[num_logical_channels].CPSR = 0; // not used on software SPI
  logical_channel[num_logical_channels].busy = false;
  return num_logical_channels;
} 


void HAL_SPI_transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length) {
  
  if (logical_channel[channel].spi_channel) {  // hardware spi
  
    LogicalChannel* logical_channel = get_logical_channel(channel);


    logical_channel->HW_CHANNEL.xfer_config.tx_data = (void *)buffer_write;
    logical_channel->HW_CHANNEL.xfer_config.rx_data = (void *)buffer_read;
    logical_channel->HW_CHANNEL.xfer_config.length = length;

    (void)SSP_ReadWrite(logical_channel->HW_CHANNEL.peripheral, &logical_channel->HW_CHANNEL.xfer_config, SSP_TRANSFER_POLLING); //SSP_TRANSFER_INTERRUPT
  }
  else 
    HAL_LPC1768_SW_SPI_transfer(????);
}


int8_t active_logical_channel = -1;
int8_t active_hardware_channel = -1;

bool begin_transmision(int8_t channel) {
if ((active_logical_channel == channel) && logical_channel[channel].spi_channel == active_hardware_channel))
    return true;   // already setup & running
  if (active_logical_channel >= 0) 
    return false;  // another channel is active
  initialise_pins(channel);   
  setup_hardware_channel(logical_channel[channel].spi_channel); 
  chip_select_active(channel);
}

bool end_transmision(int8_t channel) {
  if ((active_logical_channel != channel)
    return false; // trying to shut down the wrong channel
   active_logical_channel = -1;
   active_hardware_channel = -1;
   logical_channel[channel].busy = false;
   hardware_channel[logical_channel[channel].spi_channel].busy = false;
   chip_select_inactive(channel);
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

/*
 *  Interrupt Handlers
 */
extern "C" void ssp_irq_handler(uint8_t HW_CHANNEL) {

  SSP_DATA_SETUP_Type *xf_setup;
  uint32_t tmp;
  uint8_t dataword;

  // Disable all SSP interrupts
  SSP_IntConfig(hardware_channel[HW_CHANNEL].peripheral, SSP_INTCFG_ROR | SSP_INTCFG_RT | SSP_INTCFG_RX | SSP_INTCFG_TX, DISABLE);

  dataword = (SSP_GetDataSize(hardware_channel[HW_CHANNEL].peripheral) > 8) ? 1 : 0;

  xf_setup = &hardware_channel[HW_CHANNEL].xfer_config;
  // save status
  tmp = SSP_GetRawIntStatusReg(hardware_channel[HW_CHANNEL].peripheral);
  xf_setup->status = tmp;

  // Check overrun error
  if (tmp & SSP_RIS_ROR) {
    // Clear interrupt
    SSP_ClearIntPending(hardware_channel[HW_CHANNEL].peripheral, SSP_INTCLR_ROR);
    // update status
    xf_setup->status |= SSP_STAT_ERROR;
    // Set Complete Flag
    hardware_channel[HW_CHANNEL].xfer_complete = SET;
    if(!hardware_channel[HW_CHANNEL].active_channel->ssel_override) clear_ssel(hardware_channel[HW_CHANNEL].active_channel);
    return;
  }

  if ((xf_setup->tx_cnt != xf_setup->length) || (xf_setup->rx_cnt != xf_setup->length)) {
    /* check if RX FIFO contains data */
    while ((SSP_GetStatus(hardware_channel[HW_CHANNEL].peripheral, SSP_STAT_RXFIFO_NOTEMPTY)) && (xf_setup->rx_cnt != xf_setup->length)) {
      // Read data from SSP data
      tmp = SSP_ReceiveData(hardware_channel[HW_CHANNEL].peripheral);

      // Store data to destination
      if (xf_setup->rx_data != nullptr) {
        if (dataword == 0) {
          *(uint8_t *) ((uint32_t) xf_setup->rx_data + xf_setup->rx_cnt) = (uint8_t) tmp;
        } else {
          *(uint16_t *) ((uint32_t) xf_setup->rx_data + xf_setup->rx_cnt) = (uint16_t) tmp;
        }
      }
      // Increase counter
      if (dataword == 0) {
        xf_setup->rx_cnt++;
      } else {
        xf_setup->rx_cnt += 2;
      }
    }

    while ((SSP_GetStatus(hardware_channel[HW_CHANNEL].peripheral, SSP_STAT_TXFIFO_NOTFULL)) && (xf_setup->tx_cnt != xf_setup->length)) {
      // Write data to buffer
      if (xf_setup->tx_data == nullptr) {
        if (dataword == 0) {
          SSP_SendData(hardware_channel[HW_CHANNEL].peripheral, 0xFF);
          xf_setup->tx_cnt++;
        } else {
          SSP_SendData(hardware_channel[HW_CHANNEL].peripheral, 0xFFFF);
          xf_setup->tx_cnt += 2;
        }
      } else {
        if (dataword == 0) {
          SSP_SendData(hardware_channel[HW_CHANNEL].peripheral, (*(uint8_t *) ((uint32_t) xf_setup->tx_data + xf_setup->tx_cnt)));
          xf_setup->tx_cnt++;
        } else {
          SSP_SendData(hardware_channel[HW_CHANNEL].peripheral, (*(uint16_t *) ((uint32_t) xf_setup->tx_data + xf_setup->tx_cnt)));
          xf_setup->tx_cnt += 2;
        }
      }

      // Check overrun error
      if (SSP_GetRawIntStatus(hardware_channel[HW_CHANNEL].peripheral, SSP_INTSTAT_RAW_ROR)) {
        // update status
        xf_setup->status |= SSP_STAT_ERROR;
        // Set Complete Flag
        hardware_channel[HW_CHANNEL].xfer_complete = SET;
        if(!hardware_channel[HW_CHANNEL].active_channel->ssel_override) clear_ssel(hardware_channel[HW_CHANNEL].active_channel);
        return;
      }

      // Check for any data available in RX FIFO
      while ((SSP_GetStatus(hardware_channel[HW_CHANNEL].peripheral, SSP_STAT_RXFIFO_NOTEMPTY)) && (xf_setup->rx_cnt != xf_setup->length)) {
        // Read data from SSP data
        tmp = SSP_ReceiveData(hardware_channel[HW_CHANNEL].peripheral);

        // Store data to destination
        if (xf_setup->rx_data != nullptr) {
          if (dataword == 0) {
            *(uint8_t *) ((uint32_t) xf_setup->rx_data + xf_setup->rx_cnt) = (uint8_t) tmp;
          } else {
            *(uint16_t *) ((uint32_t) xf_setup->rx_data + xf_setup->rx_cnt) = (uint16_t) tmp;
          }
        }
        // Increase counter
        if (dataword == 0) {
          xf_setup->rx_cnt++;
        } else {
          xf_setup->rx_cnt += 2;
        }
      }
    }
  }

  // If there more data to sent or receive
  if ((xf_setup->rx_cnt != xf_setup->length) || (xf_setup->tx_cnt != xf_setup->length)) {
    // Enable all interrupt
    SSP_IntConfig(hardware_channel[HW_CHANNEL].peripheral, SSP_INTCFG_ROR | SSP_INTCFG_RT | SSP_INTCFG_RX | SSP_INTCFG_TX, ENABLE);
  } else {
    // Save status
    xf_setup->status = SSP_STAT_DONE;
    // Set Complete Flag
    hardware_channel[HW_CHANNEL].xfer_complete = SET;
    if(!hardware_channel[HW_CHANNEL].active_channel->ssel_override) clear_ssel(hardware_channel[HW_CHANNEL].active_channel);
  }
}

}
}

extern "C" void SSP0_IRQHandler(void) {
  HAL::SPI::ssp_irq_handler(0);
}

extern "C" void SSP1_IRQHandler(void) {
  HAL::SPI::ssp_irq_handler(1);
}


#endif