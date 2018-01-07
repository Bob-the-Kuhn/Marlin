#ifdef TARGET_LPC1768

#include "../spi_api.h"

#include <lpc17xx_ssp.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_gpio.h>
#include "lpc17xx_clkpwr.h"

#include "../spi_api.h

/**
 *   SPI MODE  CPOL   CPHA
 *      0        0     0     CPOL 0 - clock idles at low
 *      1        0     1     CPOL 1 - clock idles at high
 *      2        1     0     CPHA 0 - data changes on trailing edge (slave clocks in data on leading edge)
 *      3        1     1     CPHA 1 - data changes on leading edge (slave clocks in data on trailing edge)
 */

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
  pin_t sck;
  pin_t mosi;
  pin_t miso;
  pin_t chip_select;          // -1 - not used
  bool chip_select_polarity;  // 0 - active low
  uint32_t frequency;
  uint32_t frequency_used;    // may need to convert SD card speeds to a real frequency
//  uint32_t CR0;
//  uint32_t CPSR;
  uint8_t spi_mode;
  bool bit_order;             // 1 - MSB first
  uint8_t num_bits;
  bool busy;
  uint32_t* mosi_set_reg;
  uint32_t* mosi_clr_reg;
  uint32_t mosi_bit_mask;
  uint32_t* clock_active_register;
  uint32_t* clock_inactive_register;
  uint32_t sck_bit_mask;
  uint32_t miso_mask;
  uint32_t* miso_read_reg;
  uint32_t miso_mask;
} logical_channel[];



//Internal functions

extern "C" void ssp_irq_handler(uint8_t hw_channel);
#define HW_CHANNEL(chan) logical_channel[chan].spi_channel
#define SHIFT_LEFT <<
#define SHIFT_RIGHT >>

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

  if(HW_CHANNEL(channel).initialised == false && logical_channel[chan].spi_channel) {  // only do this for the hardware SPIs
    pin_cfg.Funcnum = 2; //ssp (spi) function
    pin_cfg.Portnum = HW_CHANNEL(channel).clk_port;
    pin_cfg.Pinnum = HW_CHANNEL(channel).clk_pin;
    PINSEL_ConfigPin(&pin_cfg); //clk

    pin_cfg.Portnum = HW_CHANNEL(channel).miso_port;
    pin_cfg.Pinnum = HW_CHANNEL(channel).miso_pin;
    PINSEL_ConfigPin(&pin_cfg); //miso

    pin_cfg.Portnum = HW_CHANNEL(channel).mosi_port;
    pin_cfg.Pinnum = HW_CHANNEL(channel).mosi_pin;
    PINSEL_ConfigPin(&pin_cfg); //mosi

    HW_CHANNEL(channel).initialised = true;

    //NVIC_SetPriority(HW_CHANNEL(channel).IRQn, NVIC_EncodePriority(0, 3, 0)); //Very Low priority
    //NVIC_EnableIRQ(HW_CHANNEL(channel).IRQn);
  }

  if (logical_channel[channel].chip_select >= 0) { // only do this for the hardware SPIs
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
    
    pin_cfg.Portnum = LPC1768_pin_port(logical_channel[channel].sck);
    pin_cfg.Pinnum = LPC1768_pin_pin(logical_channel[channel].sck);
    PINSEL_ConfigPin(&pin_cfg);
    if (logical_channel[channel].spi_mode & 0x02) {  // set SCK to inactive state
      GPIO_SetValue(LPC1768_pin_port(logical_channel[channel].sck), (1 << LPC1768_pin_pin(logical_channel[channel].sck)));
    else  
      GPIO_ClearValue(LPC1768_pin_port(logical_channel[channel].sck), (1 << LPC1768_pin_pin(logical_channel[channel].sck)));
    GPIO_SetDir(LPC1768_pin_port(logical_channel[channel].sck), (1 << LPC1768_pin_pin(logical_channel[channel].sck), 1));
    

    pin_cfg.Portnum = LPC1768_pin_port(logical_channel[channel].mosi);
    pin_cfg.Pinnum = LPC1768_pin_pin(logical_channel[channel].mosi);
    PINSEL_ConfigPin(&pin_cfg);
    GPIO_ClearValue(LPC1768_pin_port(logical_channel[channel].mosi), (1 << LPC1768_pin_pin(logical_channel[channel].mosi)));
    GPIO_SetDir(LPC1768_pin_port(logical_channel[channel].mosi), (1 << LPC1768_pin_pin(logical_channel[channel].mosi), 1));


    pin_cfg.Portnum = LPC1768_pin_port(logical_channel[channel].miso);
    pin_cfg.Pinnum = LPC1768_pin_pin(logical_channel[channel].miso);
    PINSEL_ConfigPin(&pin_cfg);
    GPIO_SetDir(LPC1768_pin_port(logical_channel[channel].miso), (1 << LPC1768_pin_pin(logical_channel[channel].miso), 0));
  }  
  return true;
} // initialise_pins


void setup_hardware_channel(uint8_t channel) {
  
  if (logical_channel[channel].spi_channel) {   // hardware SPI
    SSP_Cmd(HW_CHANNEL(channel).peripheral, DISABLE);

    SSP_CFG_Type HW_SPI_init; // data structure to hold init values

    HW_SPI_init.ClockRate = logical_channel[channel].frequency_used;
    HW_SPI_init.Databit = logical_channel[channel].num_bits;
    HW_SPI_init.CPHA = logical_channel[channel].spi_mode & 0x01;
    HW_SPI_init.CPOL = (logical_channel[channel].spi_mode >> 1) & 0x01;
    HW_SPI_init.Mode = SSP_MASTER_MODE;
    HW_SPI_init.FrameFormat = SSP_FRAME_SPI;
    SSP_Init(LPC_SSP0, &HW_SPI_init);  // puts the values into the proper bits in the SSP0 registers
    HW_CHANNEL(channel)->CR0 &= ~0x0F;
    HW_CHANNEL(channel)->CR0 |= (logical_channel[channel].num_bits - 1) & 0x0F;
    SSP_Cmd(HW_CHANNEL(channel).peripheral, ENABLE);
  }
  else {
    #define LPC_PORT_OFFSET         (0x0020)
    #define LPC_PIN(pin)            (1UL << pin)
    #define LPC_GPIO(port)          ((volatile LPC_GPIO_TypeDef *)(LPC_GPIO0_BASE + LPC_PORT_OFFSET * port))
    
    if (logical_channel[channel].bit_order) {
      logical_channel[channel].bit_test = _BV(logical_channel[channel].num_bits;
      logical_channel[channel].shift_direction = SHIFT_LEFT;
    }
    else {
      logical_channel[channel].bit_test = 1;
      logical_channel[channel].shift_direction = SHIFT_RIGHT;
    }
    
    if (logical_channel[channel].spi_mode & 0x02) {
      logical_channel[channel].clock_active_register = &LPC_GPIO(LPC1768_PIN_PORT(pin))->FIOCLR;
      logical_channel[channel].clock_inactive_register = &LPC_GPIO(LPC1768_PIN_PORT(pin))->FIOSET;
    }
    else {
      logical_channel[channel].clock_active_register = &LPC_GPIO(LPC1768_PIN_PORT(pin))->FIOSET;
      logical_channel[channel].clock_inactive_register = &LPC_GPIO(LPC1768_PIN_PORT(pin))->FIOCLR;
    }  
  }  

    
  
} //  setup_hardware_channel


void get_frequency_used(uint8_t channel) { 
  
  // table to convert Marlin spiRates (0-5 plus default) into bit rates
  uint32_t Marlin_speed[7]; // CPSR is always 2
  Marlin_speed[0] = 8333333; //(SCR:  2)  desired: 8,000,000  actual: 8,333,333  +4.2%  SPI_FULL_SPEED
  Marlin_speed[1] = 4166667; //(SCR:  5)  desired: 4,000,000  actual: 4,166,667  +4.2%  SPI_HALF_SPEED
  Marlin_speed[2] = 2083333; //(SCR: 11)  desired: 2,000,000  actual: 2,083,333  +4.2%  SPI_QUARTER_SPEED
  Marlin_speed[3] = 1000000; //(SCR: 24)  desired: 1,000,000  actual: 1,000,000         SPI_EIGHTH_SPEED
  Marlin_speed[4] =  500000; //(SCR: 49)  desired:   500,000  actual:   500,000         SPI_SPEED_5
  Marlin_speed[5] =  250000; //(SCR: 99)  desired:   250,000  actual:   250,000         SPI_SPEED_6
  Marlin_speed[6] =  125000; //(SCR:199)  desired:   125,000  actual:   125,000         Default from HAL.h

  if (logical_channel[channel].frequency < 7)
    logical_channel[channel].frequency = Marlin_speed[logical_channel[channel].frequency];
  else  
    logical_channel[channel].frequency_used = logical_channel[channel].frequency;
} 

int8_t num_logical_channels = -1;

int8_t create_logical_spi_channel(int8_t spi_channel, pin_t cs_pin, bool cs_polarity, uint32_t frequency, uint8_t spi_mode, bool bit_order) {

  if (!bit_order)  // LPC1768 SPI controller doesn't support reverse bit order
    return -1;

  num_logical_channels ++;
  logical_channel[num_logical_channels].spi_channel = spi_channel;    // 0: SW spi, 1: LCD SD card , 2+ system dependent
  logical_channel[num_logical_channels].sck = -1;
  logical_channel[num_logical_channels].mosi = -1;
  logical_channel[num_logical_channels].miso = -1;
  logical_channel[num_logical_channels].chip_select = cs_pin;          // -1 - not used
  logical_channel[num_logical_channels].chip_select_polarity = cs_polarity;  // 0 - active low
  logical_channel[num_logical_channels].frequency = frequency;
  logical_channel[num_logical_channels].spi_mode = spi_mode;
  logical_channel[num_logical_channels].bit_order = bit_order;
  logical_channel[num_logical_channels].busy = false;
  logical_channel[num_logical_channels].num_bits = 8;
  get_frequency_used(num_logical_channels);  // get settings for CR0 & CPSR
  return num_logical_channels;
} 

int8_t create_logical_spi_channel(pin_t sck, pin_t mosi, pin_t miso, pin_t cs, bool cs_polarity, uint32_t frequency, uint8_t spi_mode, bool bit_order);
  // make sure software SPI doesn't try to use any of the hardware SPI pins
  if (sck == P0_15 || sck == P0_7 ||
     mosi == P0_18 || mosi == P0_9 ||
     miso == P0_17 || miso == P0_8)
    return -1;
 
  num_logical_channels ++;
  logical_channel[num_logical_channels].spi_channel = 0;    // 0: SW spi, 1: LCD SD card , 2+ system dependent
  logical_channel[num_logical_channels].sck = sck;
  logical_channel[num_logical_channels].mosi = mosi;
  logical_channel[num_logical_channels].miso = miso;
  logical_channel[num_logical_channels].chip_select = cs_pin;          // -1 - not used
  logical_channel[num_logical_channels].chip_select_polarity = cs_polarity;  // 0 - active low
  logical_channel[num_logical_channels].frequency = frequency;
  logical_channel[num_logical_channels].spi_mode = spi_mode;
  logical_channel[num_logical_channels].bit_order = bit_order;
  logical_channel[num_logical_channels].CR0 = 0;  // not used on software SPI
  logical_channel[num_logical_channels].CPSR = 0; // not used on software SPI
  logical_channel[num_logical_channels].busy = false;
  logical_channel[num_logical_channels].num_bits = 8;
  logical_channel[num_logical_channels].sck_mask = 1 << LPC1768_pin_pin(sck);
  logical_channel[num_logical_channels].miso_read_reg = &LPC_GPIO(LPC1768_PIN_PORT(miso))->FIOPIN 
  logical_channel[num_logical_channels].mosi_mask = 1 << LPC1768_pin_pin(mosi);
  logical_channel[num_logical_channels].miso_read_reg = &LPC_GPIO(LPC1768_PIN_PORT(miso))->FIOPIN 
  logical_channel[num_logical_channels].miso_mask = 1 << LPC1768_pin_pin(miso);
  logical_channel[num_logical_channels].miso_read_reg = &LPC_GPIO(LPC1768_PIN_PORT(miso))->FIOPIN 
  get_frequency_used(num_logical_channels);
  return num_logical_channels;
} 

void HAL_SW_SPI_transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length);

void HAL_SPI_transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length) {
  
  if (logical_channel[channel].spi_channel) {  // hardware spi
  
    hardware_channel[logical_channel[channel].spi_channel].xfer_config.tx_data = (void *)buffer_write;
    hardware_channel[logical_channel[channel].spi_channel].xfer_config.rx_data = (void *)buffer_read;
    hardware_channel[logical_channel[channel].spi_channel].xfer_config.length = length;

    (void)SSP_ReadWrite(hardware_channel[logical_channel[channel].spi_channel].peripheral, &hardware_channel[logical_channel[channel].spi_channel].xfer_config, SSP_TRANSFER_POLLING); //SSP_TRANSFER_INTERRUPT
  }
  else 
    HAL_LPC1768_SW_SPI_transfer(channel, *buffer_write, *buffer_read, length);
}


int8_t active_logical_channel = -1;
int8_t active_hardware_channel = -1;

bool begin_transmision(int8_t channel) {
  if ((active_logical_channel == channel) && logical_channel[channel].spi_channel == active_hardware_channel))
    return true;   // already setup & running
  if (active_logical_channel >= 0) 
    return false;  // another channel is active
  active_logical_channel = channel;
  active_hardware_channel = logical_channel[channel].spi_channel;
  logical_channel[channel].busy = true;
  hardware_channel[logical_channel[channel].spi_channel].in_use = true; 
    
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
   hardware_channel[logical_channel[channel].spi_channel].in_use = false;
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
extern "C" void ssp_irq_handler(uint8_t hw_channel) {

  SSP_DATA_SETUP_Type *xf_setup;
  uint32_t tmp;
  uint8_t dataword;

  // Disable all SSP interrupts
  SSP_IntConfig(hardware_channel[hw_channel].peripheral, SSP_INTCFG_ROR | SSP_INTCFG_RT | SSP_INTCFG_RX | SSP_INTCFG_TX, DISABLE);

  dataword = (SSP_GetDataSize(hardware_channel[hw_channel].peripheral) > 8) ? 1 : 0;

  xf_setup = &hardware_channel[hw_channel].xfer_config;
  // save status
  tmp = SSP_GetRawIntStatusReg(hardware_channel[hw_channel].peripheral);
  xf_setup->status = tmp;

  // Check overrun error
  if (tmp & SSP_RIS_ROR) {
    // Clear interrupt
    SSP_ClearIntPending(hardware_channel[hw_channel].peripheral, SSP_INTCLR_ROR);
    // update status
    xf_setup->status |= SSP_STAT_ERROR;
    // Set Complete Flag
    hardware_channel[hw_channel].xfer_complete = SET;
    if(!hardware_channel[hw_channel].active_channel->ssel_override) clear_ssel(hardware_channel[hw_channel].active_channel);
    return;
  }

  if ((xf_setup->tx_cnt != xf_setup->length) || (xf_setup->rx_cnt != xf_setup->length)) {
    /* check if RX FIFO contains data */
    while ((SSP_GetStatus(hardware_channel[hw_channel].peripheral, SSP_STAT_RXFIFO_NOTEMPTY)) && (xf_setup->rx_cnt != xf_setup->length)) {
      // Read data from SSP data
      tmp = SSP_ReceiveData(hardware_channel[hw_channel].peripheral);

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

    while ((SSP_GetStatus(hardware_channel[hw_channel].peripheral, SSP_STAT_TXFIFO_NOTFULL)) && (xf_setup->tx_cnt != xf_setup->length)) {
      // Write data to buffer
      if (xf_setup->tx_data == nullptr) {
        if (dataword == 0) {
          SSP_SendData(hardware_channel[hw_channel].peripheral, 0xFF);
          xf_setup->tx_cnt++;
        } else {
          SSP_SendData(hardware_channel[hw_channel].peripheral, 0xFFFF);
          xf_setup->tx_cnt += 2;
        }
      } else {
        if (dataword == 0) {
          SSP_SendData(hardware_channel[hw_channel].peripheral, (*(uint8_t *) ((uint32_t) xf_setup->tx_data + xf_setup->tx_cnt)));
          xf_setup->tx_cnt++;
        } else {
          SSP_SendData(hardware_channel[hw_channel].peripheral, (*(uint16_t *) ((uint32_t) xf_setup->tx_data + xf_setup->tx_cnt)));
          xf_setup->tx_cnt += 2;
        }
      }

      // Check overrun error
      if (SSP_GetRawIntStatus(hardware_channel[hw_channel].peripheral, SSP_INTSTAT_RAW_ROR)) {
        // update status
        xf_setup->status |= SSP_STAT_ERROR;
        // Set Complete Flag
        hardware_channel[hw_channel].xfer_complete = SET;
        if(!hardware_channel[hw_channel].active_channel->ssel_override) clear_ssel(hardware_channel[hw_channel].active_channel);
        return;
      }

      // Check for any data available in RX FIFO
      while ((SSP_GetStatus(hardware_channel[hw_channel].peripheral, SSP_STAT_RXFIFO_NOTEMPTY)) && (xf_setup->rx_cnt != xf_setup->length)) {
        // Read data from SSP data
        tmp = SSP_ReceiveData(hardware_channel[hw_channel].peripheral);

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
    SSP_IntConfig(hardware_channel[hw_channel].peripheral, SSP_INTCFG_ROR | SSP_INTCFG_RT | SSP_INTCFG_RX | SSP_INTCFG_TX, ENABLE);
  } else {
    // Save status
    xf_setup->status = SSP_STAT_DONE;
    // Set Complete Flag
    hardware_channel[hw_channel].xfer_complete = SET;
    if(!hardware_channel[hw_channel].active_channel->ssel_override) clear_ssel(hardware_channel[hw_channel].active_channel);
  }
} //ssp_irq_handler



/////////////////   software SPI

#define nop() __asm__ __volatile__("nop;\n\t":::)

void __delay_4cycles(uint32_t cy) __attribute__ ((weak));

FORCE_INLINE void __delay_4cycles(uint32_t cy) { // +1 cycle
  #if ARCH_PIPELINE_RELOAD_CYCLES<2
    #define EXTRA_NOP_CYCLES "nop"
  #else
    #define EXTRA_NOP_CYCLES ""
  #endif

  __asm__ __volatile__(
    ".syntax unified" "\n\t" // is to prevent CM0,CM1 non-unified syntax

    "loop%=:" "\n\t"
    " subs %[cnt],#1" "\n\t"
    EXTRA_NOP_CYCLES "\n\t"
    " bne loop%=" "\n\t"
    : [cnt]"+r"(cy) // output: +r means input+output
    : // input:
    : "cc" // clobbers:
  );
}

uint8_t sw_SPI_send(uint8_t channel, uint8_t data) {
  uint8_t i, data_return;

uint32_t a = 20;
uint32_t b = 20;

#define READ_MISO(channel) ((*logical_channel[channel].miso_read_reg & logical_channel[channel].miso_mask) ? 1 : 0)

  if ((logical_channel[channel].spi_mode & 0x01) { // CPHA 1 - data changes on leading edge (slave clocks in data on trailing edge)
      data_return = 0;
      for (i = 0; i < logical_channel[channel].num_bits; i ++) {
        
      __delay_4cycles(a);
      
      if (data & bit_test)
        *logical_channel[channel].mosi_set_reg = logical_channel[channel].mosi_bit_mask;
      else  
        *logical_channel[channel].mosi_clr_reg = logical_channel[channel].mosi_bit_mask;
        
      *logical_channel[channel].clock_active_register = logical_channel[channel].sck_bit_mask; 
      
      data = data SHIFT 1;
      
      data_return = data_return SHIFT 1;    
      
      __delay_4cycles(b);
       
      *logical_channel[channel].clock_inactive_register = logical_channel[channel].sck_bit_mask;
      
      data_return |= READ_MISO(channel);
    }  
    return data_return; 
  }  
  else { // CPHA 0 - data changes on trailing edge (slave clocks in data on leading edge)
    data_return = 0;
    for (i = 0; i < logical_channel[channel].num_bits; i ++) {
      
      if (data & bit_test)
        *logical_channel[channel].mosi_set_reg = logical_channel[channel].mosi_bit_mask;
      else  
        *logical_channel[channel].mosi_clr_reg = logical_channel[channel].mosi_bit_mask;

      data_return = data_return SHIFT 1;

      __delay_4cycles(a); 
      
      *logical_channel[channel].clock_active_register = logical_channel[channel].sck_bit_mask; 
      
      data_return |= READ_MISO(channel);
      
      data = data SHIFT 1;
      
      __delay_4cycles(b);

      *logical_channel[channel].clock_inactive_register = logical_channel[channel].sck_bit_mask;
      
    }  
    return data_return;  
  }
} // sw_SPI_send


void HAL_SW_SPI_transfer(uint8_t channel, const uint8_t *buffer_write, uint8_t *buffer_read, uint32_t length) {
  uint32_t i;
  if (buffer_write = nullptr) {
    for (uint32_t i; i < length; i++) { buffer_read[i] = sw_SPI_send(channel, 0xFF);}
  }
  else {
    if (buffer_read = nullptr) { 
      for (uint32_t i; i < length; i++) {sw_SPI_send(channel, buffer_write[i]);}
    }  
  }  
  else
    for (uint32_t i; i < length; i++) {buffer_read[i] = sw_SPI_send(channel, buffer_write[i]);}
  
} // HAL_SW_SPI_transfer  

}  // SPI
}  // HAL

extern "C" void SSP0_IRQHandler(void) {
  HAL::SPI::ssp_irq_handler(1);
}

extern "C" void SSP1_IRQHandler(void) {
  HAL::SPI::ssp_irq_handler(2);
}


#endif
