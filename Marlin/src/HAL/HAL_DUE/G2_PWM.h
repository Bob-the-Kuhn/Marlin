/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * This module is based on the LPC1768_PWM.h file from PR #7500.
 * It is hardwired for the PRINTRBOARD_G2 Motor Current needs.
 */

/**
 * The class Servo uses the PWM class to implement its functions.
 *
 * The PWM1 module is only used to generate interrups at specified times. It
 * is NOT used to directly toggle pins. The ISR writes to the pin assigned to
 * that interrupt
 *
 * All PWMs use the same repetition rate - 20mS because that's the normal servo rate
 *
 * The data structures are setup to minimize the computation done by the ISR which
 * minimizes ISR execution time.  Execution times are 1.7 to 1.9 microseconds.
 *
 * Two tables are used.  active_table is used by the ISR.  Changes to the table are
 * are done by copying the active_table into the work_table, updating the work_table
 * and then swapping the two tables.  Swapping is done by manipulating pointers.
 *
 * Immediately after the swap the ISR uses the work_table until the start of the
 * next 20mS cycle. During this transition the "work_table" is actually the table
 * that was being used before the swap.  The "active_table" contains the data that
 * will start being used at the start of the next 20mS period.  This keeps the pins
 * well behaved during the transition.
 *
 * The ISR's priority is set to the maximum otherwise other ISRs can cause considerable
 * jitter in the PWM high time.
 */

#include "../../inc/MarlinConfig.h"

#define PWM_PERIOD_US  100  // base repetition rate in micro seconds

typedef struct {       // holds the data needed by the ISR to control the Vref pin
  volatile uint32_t* set_register;
  volatile uint32_t* clr_register;
  uint32_t write_mask;
} PWM_map;

#define G2_VREF(I) (uint32_t)(I * 5 * 0.15)   // desired Vref * 1000 (scaled so don't loose accuracy in next step)

#define G2_VREF_COUNT(Q) (uint32_t)map(constrain(Q, 500, 3.3 * 1000), 0, 3.3 * 1000, 0, PWM_PERIOD_US)  // under 500  the results are very non-linear

volatile uint32_t* SODR_A = &PIOA->PIO_SODR;
volatile uint32_t* SODR_B = &PIOB->PIO_SODR;
volatile uint32_t* CODR_A = &PIOA->PIO_CODR;
volatile uint32_t* CODR_B = &PIOB->PIO_CODR;

#define _PIN(IO) (DIO ## IO ## _PIN)

#define PWM_MAP_INIT_ROW(IO,ZZ) { ZZ == 'A' ? SODR_A : SODR_B,  ZZ == 'A' ? CODR_A : CODR_B, 1 << _PIN(IO) }


#define PWM_MAP_INIT {  PWM_MAP_INIT_ROW(MOTOR_CURRENT_PWM_X_PIN, 'B'), \
                        PWM_MAP_INIT_ROW(MOTOR_CURRENT_PWM_Y_PIN, 'B'), \
                        PWM_MAP_INIT_ROW(MOTOR_CURRENT_PWM_Z_PIN, 'B'), \
                        PWM_MAP_INIT_ROW(MOTOR_CURRENT_PWM_E_PIN, 'A'), \
                     };

#define NUM_PWMS 4

PWM_map ISR_table[NUM_PWMS] = PWM_MAP_INIT;

volatile uint8_t PWM1_ISR_index = 0;

#define IR_BIT(p) (p >= 0 && p <= 3 ? p : p + 4 )
#define COPY_ACTIVE_TABLE    for (uint8_t i = 0; i < 6 ; i++) work_table[i] = active_table[i]

#define PWM_MR0 19999  // base repetition rate minus one count - 20mS
#define PWM_PR 24      // prescaler value - prescaler divide by 24 + 1  -  1 MHz output
#define PWM_PCLKSEL0 0x00   // select clock source for prescaler - defaults to 25MHz on power up
                            // 0: 25MHz, 1: 100MHz, 2: 50MHz, 3: 12.5MHZ to PWM1 prescaler
#define MR0_MARGIN 200 // if channel value too close to MR0 the system locks up

bool PWM_table_swap = false;  // flag to tell the ISR that the tables have been swapped

void Stepper::digipot_init() {

  OUT_WRITE(MOTOR_CURRENT_PWM_X_PIN, 0);  // init pins
  OUT_WRITE(MOTOR_CURRENT_PWM_Y_PIN, 0);
  OUT_WRITE(MOTOR_CURRENT_PWM_Z_PIN, 0);
  OUT_WRITE(MOTOR_CURRENT_PWM_E_PIN, 0);

  #define WPKEY          (0x50574D << 8) // “PWM” in ASCII
  #define WPCMD_DIS_SW   0  // command to disable Write Protect SW
  #define WPRG_ALL       (PWM_WPCR_WPRG0 | PWM_WPCR_WPRG1 | PWM_WPCR_WPRG2 | PWM_WPCR_WPRG3 | PWM_WPCR_WPRG4 | PWM_WPCR_WPRG5)  // all Write Protect Groups

  #define PWM_CLOCK_F    F_CPU / 1000000UL   // set clock to 1MHz

  PMC->PMC_PCER1 = PMC_PCER1_PID36;                       // enable PWM controller clock (disabled on power up)

  PWM->PWM_WPCR = WPKEY | WPRG_ALL | WPCMD_DIS_SW;        // enable setting of all PWM registers
  PWM->PWM_CLK = PWM_CLOCK_F;                             // enable CLK_A and set it to 1MHz, leave CLK_B disabled
  PWM->PWM_CH_NUM[0].PWM_CMR = 0b1011;                    // set channel 0 to Clock A input & to left aligned
  PWM->PWM_CH_NUM[1].PWM_CMR = 0b1011;                    // set channel 1 to Clock A input & to left aligned
  PWM->PWM_CH_NUM[2].PWM_CMR = 0b1011;                    // set channel 2 to Clock A input & to left aligned
  PWM->PWM_CH_NUM[3].PWM_CMR = 0b1011;                    // set channel 3 to Clock A input & to left aligned
  PWM->PWM_CH_NUM[4].PWM_CMR = 0b1011;                    // set channel 4 to Clock A input & to left aligned

  PWM->PWM_CH_NUM[0].PWM_CPRD = PWM_PERIOD_US;                    // set channel 0 Period

  PWM->PWM_IER2 = PWM_IER1_CHID0;                          // generate interrupt when counter0 overflows
  PWM->PWM_IER2 = PWM_IER2_CMPM0 | PWM_IER2_CMPM1 | PWM_IER2_CMPM2 | PWM_IER2_CMPM3 | PWM_IER2_CMPM4;        // generate interrupt on compare event

  PWM->PWM_CMP[1].PWM_CMPV = 0x010000000ll | G2_VREF_COUNT(G2_VREF(motor_current_setting[0]));   // interrupt when counter0 == CMPV - used to set Motor 1 PWM inactive
  PWM->PWM_CMP[2].PWM_CMPV = 0x010000000ll | G2_VREF_COUNT(G2_VREF(motor_current_setting[0]));   // interrupt when counter0 == CMPV - used to set Motor 2 PWM inactive
  PWM->PWM_CMP[3].PWM_CMPV = 0x010000000ll | G2_VREF_COUNT(G2_VREF(motor_current_setting[1]));   // interrupt when counter0 == CMPV - used to set Motor 3 PWM inactive
  PWM->PWM_CMP[4].PWM_CMPV = 0x010000000ll | G2_VREF_COUNT(G2_VREF(motor_current_setting[2]));   // interrupt when counter0 == CMPV - used to set Motor 4 PWM inactive

  PWM->PWM_CMP[1].PWM_CMPM = 0x0001;  // enable compare event
  PWM->PWM_CMP[2].PWM_CMPM = 0x0001;  // enable compare event
  PWM->PWM_CMP[3].PWM_CMPM = 0x0001;  // enable compare event
  PWM->PWM_CMP[4].PWM_CMPM = 0x0001;  // enable compare event

  PWM->PWM_SCM = PWM_SCM_UPDM_MODE0 | PWM_SCM_SYNC0 | PWM_SCM_SYNC1 | PWM_SCM_SYNC2 | PWM_SCM_SYNC3 | PWM_SCM_SYNC4; // sync 1-4 with 0, use mode 0 for updates

  PWM->PWM_ENA = PWM_ENA_CHID0 | PWM_ENA_CHID1 | PWM_ENA_CHID2 | PWM_ENA_CHID3 | PWM_ENA_CHID4;         // enable the channels used by G2
  PWM->PWM_IER1 = PWM_IER1_CHID0 | PWM_IER1_CHID1 | PWM_IER1_CHID2 | PWM_IER1_CHID3 | PWM_IER1_CHID4;        // enable interrupts for the channels used by G2

  NVIC_EnableIRQ(PWM_IRQn);     // Enable interrupt handler
  NVIC_SetPriority(PWM_IRQn, NVIC_EncodePriority(0, 10, 0));  // normal priority for PWM module (can stand some jitter on the Vref signals)
}

void Stepper::digipot_current(const uint8_t driver, const int16_t current) {

  if (!(PWM->PWM_CH_NUM[0].PWM_CPRD == PWM_PERIOD_US)) digipot_init();  // Init PWM system if needed

  switch (driver) {
    case 0: PWM->PWM_CMP[1].PWM_CMPVUPD = 0x010000000ll | G2_VREF_COUNT(G2_VREF(current));    // update X & Y
            PWM->PWM_CMP[2].PWM_CMPVUPD = 0x010000000ll | G2_VREF_COUNT(G2_VREF(current));
            PWM->PWM_CMP[1].PWM_CMPMUPD = 0x0001;  // enable compare event
            PWM->PWM_CMP[2].PWM_CMPMUPD = 0x0001;  // enable compare event
            PWM->PWM_SCUC = PWM_SCUC_UPDULOCK; // tell the PWM controller to update the values on the next cycle
            break;
    case 1: PWM->PWM_CMP[3].PWM_CMPVUPD = 0x010000000ll | G2_VREF_COUNT(G2_VREF(current));    // update Z
            PWM->PWM_CMP[3].PWM_CMPMUPD = 0x0001;  // enable compare event
            PWM->PWM_SCUC = PWM_SCUC_UPDULOCK; // tell the PWM controller to update the values on the next cycle
            break;
    default:PWM->PWM_CMP[4].PWM_CMPVUPD = 0x010000000ll | G2_VREF_COUNT(G2_VREF(current));    // update E
            PWM->PWM_CMP[4].PWM_CMPMUPD = 0x0001;  // enable compare event
            PWM->PWM_SCUC = PWM_SCUC_UPDULOCK; // tell the PWM controller to update the values on the next cycle
            break;
  }
}

////////////////////////////////////////////////////////////////////////////////

#define HAL_G2_PWM_ISR  void PWM_Handler()

volatile uint32_t PWM_ISR1_STATUS, PWM_ISR2_STATUS;

HAL_G2_PWM_ISR {
  PWM_ISR1_STATUS = PWM->PWM_ISR1;
  PWM_ISR2_STATUS = PWM->PWM_ISR2;
  if (PWM_ISR1_STATUS & PWM_IER1_CHID0) {                           // CHAN_0 interrupt
    *ISR_table[0].set_register = ISR_table[0].write_mask;                                          // set X to active
    *ISR_table[1].set_register = ISR_table[1].write_mask;                                          // set Y to active
    *ISR_table[2].set_register = ISR_table[2].write_mask;                                          // set Z to active
    *ISR_table[3].set_register = ISR_table[3].write_mask;                                          // set E to active
  }
  else {
    if (PWM_ISR2_STATUS & PWM_IER2_CMPM1)  *ISR_table[0].clr_register = ISR_table[0].write_mask;   // set X to inactive
    if (PWM_ISR2_STATUS & PWM_IER2_CMPM2)  *ISR_table[1].clr_register = ISR_table[1].write_mask;   // set Y to inactive
    if (PWM_ISR2_STATUS & PWM_IER2_CMPM3)  *ISR_table[2].clr_register = ISR_table[2].write_mask;   // set Z to inactive
    if (PWM_ISR2_STATUS & PWM_IER2_CMPM4)  *ISR_table[3].clr_register = ISR_table[3].write_mask;   // set E to inactive
  }
  return;
}
