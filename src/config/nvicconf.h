/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * nvicconf.c - Interrupt priority configuration
 *
 * The STM32 has 16 priorities to choose from where 0 is the
 * highest priority. They are now configured using no groups.
 *
 * Interrupt functions that call FreeRTOS FromISR functions
 * must have a interrupt number 10 and above which is currently
 * set by configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#ifndef NVIC_CONF_H_
#define NVIC_CONF_H_

/*
 Interrupt priority organisation in Crazyflie:

 In Cortex-M low priority number is higher priority. Hence priority 0 is the
 highest priority interrupt and priority 15 the lowest (STM32 implements
 4 priority bits)

 Interrupts bellow MAX_SYSCALL_INTERRUPT_PRIORITY cannot call any RTOS
 functions! They should be handled like some kind of softdevice, running above
 the OS.

 4 Interrupt level are defined
  - NVIC_LOW_PRI
  - NVIC_MID_PRI
  - NVIC_HIGH_PRI
  - NVIC_VERY_HIGH_PRI
 The aim is to simplify interrupt handling and to document why any special case
 is required.

 15 -
 14 -
 13 - NVIC_LOW_PRI
 12 - NVIC_ADC_PRI
 11 - NVIC_RADIO_PRI
 10 - NVIC_MID_PRI
  9 -
  8 -
  7 - NVIC_HIGH_PRI
  6 - NVIC_VERY_HIGH_PRI
  5 -                                     <-- MAX_SYSCALL_INTERRUPT_PRIORITY
  4 ! NVIC_I2C_PRI_LOW NVIC_TRACE_TIM_PRI --- Does not call any RTOS function
  3 ! NVIC_I2C_PRI_HIGH
  2 !
  1 !
  0 !
*/

// Standard interrupt levels
#define NVIC_LOW_PRI  13
#define NVIC_MID_PRI  10
#define NVIC_HIGH_PRI 7
#define NVIC_VERY_HIGH_PRI 7


#define NVIC_TRACE_TIM_PRI    4
#define NVIC_RADIO_PRI        11
#define NVIC_ADC_PRI          12
#define NVIC_CPPM_PRI         14
#define NVIC_SYSLINK_FLOW_PRI 4
#define NVIC_SYSLINK_UART_PRI 5
#define NVIC_SYSLINK_DMA_PRI  7
#define NVIC_SPI_PRI          7
#define NVIC_I2C_PRI          7
#define NVIC_MOTORS_PRI       10
#define NVIC_UART1_PRI        10
#define NVIC_UART1_DMA_PRI    10
#define NVIC_UART2_PRI        6
#define NVIC_UART2_DMA_PRI    10
#define NVIC_WS2812_PRI       13
#define NVIC_BMI088_SPI_PRI   7
#define NVIC_USB_BSP_PRI      10


// Priorities for external interrupts
#define EXTI0_PRI NVIC_LOW_PRI
#define EXTI1_PRI NVIC_LOW_PRI
#define EXTI2_PRI NVIC_LOW_PRI
#define EXTI3_PRI NVIC_LOW_PRI
#define EXTI4_PRI NVIC_SYSLINK_FLOW_PRI // this is used for flow control when STM sends data to the NRF on the syslink UART
#define EXTI9_5_PRI NVIC_LOW_PRI
#define EXTI15_10_PRI NVIC_MID_PRI // this serves the decks and sensors

#endif /* NVIC_CONF_H_ */
