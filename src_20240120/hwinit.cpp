/*
 * This file is part of the CANBootloader project.
 *
 * Copyright (C) 2022 WDR Automatisering https://wdrautomatisering.nl/
 * Copyright (C) 2023 Johannes Huebner https://openinverter.org
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
 */

#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/nvic.h>
#include "hwinit.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup()
{
   rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL6);
   rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);
   rcc_osc_on(RCC_PLL);
   rcc_wait_for_osc_ready(RCC_PLL);
   rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);
   rcc_osc_on(RCC_LSI);

   rcc_apb1_frequency = 24000000;
   rcc_apb2_frequency = 24000000;

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_CRC);
   rcc_periph_clock_enable(RCC_CAN1);
   rcc_periph_clock_enable(RCC_CAN2);		// CAN2
   rcc_periph_clock_enable(RCC_USART3);

   rcc_wait_for_osc_ready(RCC_LSI);
   iwdg_set_period_ms(2000);
   //iwdg_start();
}

void clock_teardown()
{
   rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);
   rcc_osc_off(RCC_PLL);
}

#define SETBIT(port,_bit)     port |= 1UL << _bit
#define CLEARBIT(port,_bit)   port &= ~(1UL << _bit)
#define _SET(add,_bit)        MMIO32(add) |= 1UL << _bit
#define _CLEAR(add,_bit)      MMIO32(add) &= ~(1UL << _bit)

void can2FilterInit_a()
{
	CAN_FMR(CAN1) |= CAN_FMR_FINIT;
	CAN_FA1R(CAN1) &= ~(1 << 14);
	CAN_FS1R(CAN1) &= ~(1 << 14);
	CAN_FM1R(CAN1) &= ~(1 << 14);
	CAN_FiR1(CAN1, 14) = 0L;
	CAN_FiR2(CAN1, 14) = 0L;
	CAN_FA1R(CAN1) &= ~(1UL << 14);

	CAN_FMR(CAN1) &= ~CAN_FMR_FINIT;
	CAN_ESR(CAN2) = 0L;
	CAN_RF0R(CAN2) |= (1UL << 5);

}

void can2FilterInit_b()
{
	//CAN_FMR(CAN1) |= CAN_FMR_FINIT;
	_SET(CAN_FMR(CAN1),0);
	//CAN_FA1R(CAN1) &= ~(1 << 14);
	_CLEAR(CAN_FA1R(CAN1),14);
	//CAN_FS1R(CAN1) &= ~(1 << 14);
	_CLEAR(CAN_FS1R(CAN1),14);
	//CAN_FM1R(CAN1) &= ~(1 << 14);
	_CLEAR(CAN_FM1R(CAN1),14);
	CAN_FiR1(CAN1, 14) = 0L;
	CAN_FiR2(CAN1, 14) = 0L;
	CAN_FA1R(CAN1) &= ~(1UL << 14);

	//CAN_FMR(CAN1) &= ~CAN_FMR_FINIT;
	_CLEAR(CAN_FMR(CAN1),0);
	CAN_ESR(CAN2) = 0L;
	//CAN_RI0R(CAN2) |= (1UL << 5);
	_SET(CAN_RF0R(CAN2),5);

}

void can2_setup(int masterId)
{   
   // GPIO Configuration for CAN2 RX and TX
    // Adjust these pins based on your specific microcontroller
    gpio_set_mode(GPIO_BANK_CAN2_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN2_RX);
    gpio_set(GPIO_BANK_CAN2_RX, GPIO_CAN2_RX);
    gpio_set_mode(GPIO_BANK_CAN2_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN2_TX);

    // CAN2 Initialization
    can_init(CAN2,
            false,          // TTCM: Time triggered comm mode?
            true,           // ABOM: Automatic bus-off management?
            false,          // AWUM: Automatic wakeup mode?
            false,          // NART: No automatic retransmission?
            false,          // RFLM: Receive FIFO locked mode?
            false,          // TXFP: Transmit FIFO priority?
            CAN_BTR_SJW_1TQ,
            CAN_BTR_TS1_6TQ,
            CAN_BTR_TS2_1TQ,
            24,				   // BRP: Baud rate prescaler  125kb
            false,
            false);

    
     can_filter_id_list_16bit_init(14, masterId << 5, 0, 0, 0, 0, true);

    // NVIC Configuration for CAN2 (if using interrupts)
	
     nvic_enable_irq(NVIC_CAN2_RX0_IRQ);
	 nvic_set_priority(NVIC_CAN2_RX0_IRQ, 0xf << 4);
     can_enable_irq(CAN2, CAN_IER_FMPIE0);
   
}

void can_setup(int masterId)
{
   gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
   gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);
   // Configure CAN pin: TX
   gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);

   // CAN cell init.
   // Setting the bitrate to 500KBit. APB1 = 24MHz,
   // prescaler = 6 -> 4MHz time quanta frequency.
   // 1tq sync + 6tq bit segment1 (TS1) + 1tq bit segment2 (TS2) =
   // 8time quanto per bit period, therefor 4MHz/16 = 500kHz
   //
   can_init(CAN1,
            false,          // TTCM: Time triggered comm mode?
            true,           // ABOM: Automatic bus-off management?
            false,          // AWUM: Automatic wakeup mode?
            false,          // NART: No automatic retransmission?
            false,          // RFLM: Receive FIFO locked mode?
            false,          // TXFP: Transmit FIFO priority?
            CAN_BTR_SJW_1TQ,
            CAN_BTR_TS1_6TQ,
            CAN_BTR_TS2_1TQ,
            24,				   // BRP: Baud rate prescaler    6 = 500k  24 = 125k
            false,
            false);

   //register master ID
   can_filter_id_list_16bit_init(0, masterId << 5, 0, 0, 0, 0, true);
   nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
   can_enable_irq(CAN1, CAN_IER_FMPIE0);
   
   can2_setup(masterId);
}
  


void can_teardown()
{
   can_reset(CAN1);
   nvic_disable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
   can_reset(CAN2);
   nvic_disable_irq(NVIC_CAN2_RX0_IRQ);
}

void usart_setup()
{
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART3, 9600);
    usart_set_databits(USART3, 8);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_enable_rx_interrupt(USART3);

    /* Finally enable the USART. */
    usart_enable(USART3);
    nvic_enable_irq(NVIC_USART3_IRQ);
}

void usart_teardown()
{
    nvic_disable_irq(NVIC_USART3_IRQ);
    rcc_periph_reset_pulse(RST_USART3);
}

//Left over for future usage in an inverter style device..
// Todo: read an application configuration based on fingerprint?
//
void initialize_pins()
{
   uint32_t flashSize = desig_get_flash_size();
   uint32_t pindefAddr = FLASH_BASE + flashSize * 1024 - PINDEF_BLKNUM * PINDEF_BLKSIZE;
   const struct pincommands* pincommands = (struct pincommands*)pindefAddr;

   uint32_t crc = crc_calculate_block(((uint32_t*)pincommands), PINDEF_NUMWORDS);

   if (crc == pincommands->crc)
   {
      for (int idx = 0; idx < NUM_PIN_COMMANDS && pincommands->pindef[idx].port > 0; idx++)
      {
         uint8_t cnf = pincommands->pindef[idx].inout ? GPIO_CNF_OUTPUT_PUSHPULL : GPIO_CNF_INPUT_PULL_UPDOWN;
         uint8_t mode = pincommands->pindef[idx].inout ? GPIO_MODE_OUTPUT_50_MHZ : GPIO_MODE_INPUT;
         gpio_set_mode(pincommands->pindef[idx].port, mode, cnf, pincommands->pindef[idx].pin);

         if (pincommands->pindef[idx].level)
         {
            gpio_set(pincommands->pindef[idx].port, pincommands->pindef[idx].pin);
         }
      }
   }
}
