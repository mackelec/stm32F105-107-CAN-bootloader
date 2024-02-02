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
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/scb.h>
#include "hwinit.h"

#define FLASH_START         0x08000000
#define SMALLEST_PAGE_WORDS 256
#define PROGRAM_WORDS       256
#define APP_FLASH_START     0x08001800
#define BOOTLOADER_MAGIC    0xAA
#define DELAY_100           (1 << 17)
#define NODECANID           0x2DE
#define MASTERCANID         0x2DD

enum states
{
   MAGIC, PAGECOUNT, PAGE, CRC, PROGRAM, DONE
};

static volatile states state = MAGIC;
static uint32_t page_buffer[PROGRAM_WORDS];
static bool usartUpdate = false;
static uint8_t can_port = 2;

//Check 1k of flash whether it contains only 0xFF = erased
static bool check_erased(uint32_t* baseAddress)
{
   uint32_t check = 0xFFFFFFFF;

   for (int i = 0; i < SMALLEST_PAGE_WORDS; i++, baseAddress++)
      check &= *baseAddress;

   return check == 0xFFFFFFFF;
}

//If the device has 2k pages we must only call flash_erase_page()
//on every other call of write_flash().
//Therefor we check if the the flash region we attempt to write
//is already erased (in case of 2k pages) or not (1k pages)
static void write_flash(uint32_t addr, uint32_t *pageBuffer)
{
   if (!check_erased((uint32_t*)addr))
      flash_erase_page(addr);

   for (uint32_t idx = 0; idx < PROGRAM_WORDS; idx++)
   {
      flash_program_word(addr + idx * 4, pageBuffer[idx]);
   }
}

static void send_byte(uint8_t b)
{
   switch (can_port)
   {
	   case 1:can_transmit(CAN1, NODECANID, false, false, 1, &b);break;
	   case 2:can_transmit(CAN2, NODECANID, false, false, 1, &b);break;
   }
   //can_transmit(CAN1, NODECANID, false, false, 1, &b);
   if (usartUpdate) usart_send_blocking(USART3, b);
}

static void send_can_hello()
{
   uint32_t data[] = { '3', DESIG_UNIQUE_ID2 };
	//can_transmit(CAN1, NODECANID, false, false, 8, (uint8_t*)data);
	can_transmit(CAN2, NODECANID, false, false, 8, (uint8_t*)data);
   
   //can_transmit(CAN1, NODECANID, false, false, 8, (uint8_t*)data);
}

static bool can_recv(uint8_t* data, uint8_t& len)
{
   uint32_t id;
   bool ext, rtr;
   uint8_t fmi;

   return can_receive(CAN1, 0, true, &id, &ext, &rtr, &fmi, &len, data, 0) > 0;
}

static bool can2_recv(uint8_t* data, uint8_t& len)
{
   uint32_t id;
   bool ext, rtr;
   uint8_t fmi;

   return can_receive(CAN2, 0, true, &id, &ext, &rtr, &fmi, &len, data, 0) > 0;
}

static void wait()
{
   for (volatile uint32_t i = DELAY_100; i > 0; i--);
}


extern "C" int main(void)
{
   uint32_t addr = APP_FLASH_START;

   clock_setup();
   initialize_pins();
   can_setup(MASTERCANID);
   usart_setup();

   send_can_hello();
   usart_send(USART3, '2'); //advertise version 2 as the protocol is unchanged

   wait();
   wait();
   

   if (state == PAGECOUNT || state == PAGE)
   {
      flash_unlock();

      while (state != DONE)
      {
         if (state == PROGRAM)
         {
            write_flash(addr, page_buffer);
            addr += sizeof(page_buffer);
            state = PAGE;
            send_byte('P');
         }
         iwdg_reset();
      }

      //Program the final page
      write_flash(addr, page_buffer);
      flash_lock();
   }

   //We are done lets tell the world this!!
   send_byte('D');
   usart_send(USART3, 'X');

   wait();
   
   can_teardown();
   usart_teardown();
   clock_teardown();

   void (*app_main)(void) = (void (*)(void)) *(volatile uint32_t*)(APP_FLASH_START + 4);
   SCB_VTOR = APP_FLASH_START;
   app_main();

   return 0;
}

static void handle_data(uint8_t* data, uint8_t len)
{
   uint32_t* words = (uint32_t*)data;
   static uint8_t numPages = 0;
   static uint32_t currentWord = 0;
   static uint32_t crc;

   switch (state)
   {
   case MAGIC:
      if ((usartUpdate && data[0] == BOOTLOADER_MAGIC) || words[0] == DESIG_UNIQUE_ID2)
      {
         send_byte('S');
         state = PAGECOUNT;
      }
      break;
   case PAGECOUNT:
      numPages = data[0];
      state = PAGE;
      currentWord = 0;
      send_byte('P');
      crc_reset();
      break;
   case PAGE:
      page_buffer[currentWord++] = words[0];
      page_buffer[currentWord++] = words[1];
      crc_calculate(words[0]);
      crc = crc_calculate(words[1]);

      if (currentWord == PROGRAM_WORDS)
      {
         state = CRC;
         send_byte('C');
      }
      else if (!usartUpdate)
      {
         send_byte('P');
      }
      break;
   case CRC:
      currentWord = 0;
      crc_reset();
      if (words[0] == crc)
      {
         numPages--;
         if (numPages == 0)
         {
            state = DONE;
         }
         else
         {
            state = PROGRAM;
         }
      }
      else
      {
         send_byte('E');
         state = PAGE;
      }
      break;
   case PROGRAM:
      //Flash programming done in main()
      break;
   case DONE:
      //Nothing to do!
      break;
   }

}

/* Interrupt service routines */
extern "C" void usb_lp_can_rx0_isr()
{
   uint8_t canData[8], len;

   can_recv(canData, len);
   //handle_data(canData, len);
}

extern "C" void can2_rx0_isr()
{
	uint8_t canData[8], len;
	can_port = 2;
	can2_recv(canData, len);
	handle_data(canData, len);
}

extern "C" void usart3_isr()
{
   static uint8_t buffer[8], currentByte = 0;

   uint8_t data = usart_recv(USART3);

   usartUpdate = true;

   switch (state)
   {
   case MAGIC:
   case PAGECOUNT:
      handle_data(&data, 1);
      break;
   case PAGE:
      buffer[currentByte++] = data;
      if (currentByte == 8)
      {
         currentByte = 0;
         handle_data(buffer, 8);
      }
      break;
   case CRC:
      buffer[currentByte++] = data;
      if (currentByte == 4)
      {
         currentByte = 0;
         handle_data(buffer, 4);
      }
      break;
   default:
      //Should never get here
      break;
   }
}
