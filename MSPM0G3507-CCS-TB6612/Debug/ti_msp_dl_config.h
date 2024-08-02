/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_4
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM17)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM17_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_1
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM13)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM13_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA0)
#define TIMER_0_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (1999U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_4_MHZ_9600_BAUD                                         (26)
#define UART_0_FBRD_4_MHZ_9600_BAUD                                          (3)





/* Port definition for Pin Group LED1 */
#define LED1_PORT                                                        (GPIOA)

/* Defines for PIN_0: GPIOA.28 with pinCMx 3 on package pin 35 */
#define LED1_PIN_0_PIN                                          (DL_GPIO_PIN_28)
#define LED1_PIN_0_IOMUX                                          (IOMUX_PINCM3)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for PIN_21: GPIOB.21 with pinCMx 49 on package pin 20 */
#define KEY_PIN_21_PIN                                          (DL_GPIO_PIN_21)
#define KEY_PIN_21_IOMUX                                         (IOMUX_PINCM49)
/* Port definition for Pin Group AIN1 */
#define AIN1_PORT                                                        (GPIOA)

/* Defines for PIN_12: GPIOA.12 with pinCMx 34 on package pin 5 */
#define AIN1_PIN_12_PIN                                         (DL_GPIO_PIN_12)
#define AIN1_PIN_12_IOMUX                                        (IOMUX_PINCM34)
/* Port definition for Pin Group AIN2 */
#define AIN2_PORT                                                        (GPIOA)

/* Defines for PIN_13: GPIOA.13 with pinCMx 35 on package pin 6 */
#define AIN2_PIN_13_PIN                                         (DL_GPIO_PIN_13)
#define AIN2_PIN_13_IOMUX                                        (IOMUX_PINCM35)
/* Port definition for Pin Group BIN1 */
#define BIN1_PORT                                                        (GPIOB)

/* Defines for Pin_Bin1: GPIOB.16 with pinCMx 33 on package pin 4 */
#define BIN1_Pin_Bin1_PIN                                       (DL_GPIO_PIN_16)
#define BIN1_Pin_Bin1_IOMUX                                      (IOMUX_PINCM33)
/* Port definition for Pin Group BIN2 */
#define BIN2_PORT                                                        (GPIOB)

/* Defines for Pin_Bin2: GPIOB.0 with pinCMx 12 on package pin 47 */
#define BIN2_Pin_Bin2_PIN                                        (DL_GPIO_PIN_0)
#define BIN2_Pin_Bin2_IOMUX                                      (IOMUX_PINCM12)
/* Port definition for Pin Group BEER */
#define BEER_PORT                                                        (GPIOA)

/* Defines for PIN_1: GPIOA.27 with pinCMx 60 on package pin 31 */
#define BEER_PIN_1_PIN                                          (DL_GPIO_PIN_27)
#define BEER_PIN_1_IOMUX                                         (IOMUX_PINCM60)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.15 with pinCMx 37 on package pin 8 */
// groups represented: ["ENCODERB","ENCODERA"]
// pins affected: ["E2A","E2B","E1A","E1B"]
#define GPIO_MULTIPLE_GPIOA_INT_IRQN                            (GPIOA_INT_IRQn)
#define GPIO_MULTIPLE_GPIOA_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO15)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_15)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM37)
/* Defines for E1B: GPIOA.16 with pinCMx 38 on package pin 9 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO16)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_16)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM38)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOA)

/* Defines for E2A: GPIOA.17 with pinCMx 39 on package pin 10 */
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO17)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_17)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM39)
/* Defines for E2B: GPIOA.22 with pinCMx 47 on package pin 18 */
#define ENCODERB_E2B_IIDX                                   (DL_GPIO_IIDX_DIO22)
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_22)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM47)
/* Port definition for Pin Group I2C */
#define I2C_PORT                                                         (GPIOA)

/* Defines for SCL: GPIOA.0 with pinCMx 1 on package pin 33 */
#define I2C_SCL_PIN                                              (DL_GPIO_PIN_0)
#define I2C_SCL_IOMUX                                             (IOMUX_PINCM1)
/* Defines for SDA: GPIOA.1 with pinCMx 2 on package pin 34 */
#define I2C_SDA_PIN                                              (DL_GPIO_PIN_1)
#define I2C_SDA_IOMUX                                             (IOMUX_PINCM2)
/* Port definition for Pin Group LINE4 */
#define LINE4_PORT                                                       (GPIOB)

/* Defines for PIN_41: GPIOB.8 with pinCMx 25 on package pin 60 */
#define LINE4_PIN_41_PIN                                         (DL_GPIO_PIN_8)
#define LINE4_PIN_41_IOMUX                                       (IOMUX_PINCM25)
/* Defines for PIN_42: GPIOB.7 with pinCMx 24 on package pin 59 */
#define LINE4_PIN_42_PIN                                         (DL_GPIO_PIN_7)
#define LINE4_PIN_42_IOMUX                                       (IOMUX_PINCM24)
/* Defines for PIN_43: GPIOB.6 with pinCMx 23 on package pin 58 */
#define LINE4_PIN_43_PIN                                         (DL_GPIO_PIN_6)
#define LINE4_PIN_43_IOMUX                                       (IOMUX_PINCM23)
/* Defines for PIN_44: GPIOB.15 with pinCMx 32 on package pin 3 */
#define LINE4_PIN_44_PIN                                        (DL_GPIO_PIN_15)
#define LINE4_PIN_44_IOMUX                                       (IOMUX_PINCM32)
/* Defines for PIN_45: GPIOB.12 with pinCMx 29 on package pin 64 */
#define LINE4_PIN_45_PIN                                        (DL_GPIO_PIN_12)
#define LINE4_PIN_45_IOMUX                                       (IOMUX_PINCM29)
/* Defines for PIN_46: GPIOB.17 with pinCMx 43 on package pin 14 */
#define LINE4_PIN_46_PIN                                        (DL_GPIO_PIN_17)
#define LINE4_PIN_46_IOMUX                                       (IOMUX_PINCM43)
/* Defines for PIN_47: GPIOB.20 with pinCMx 48 on package pin 19 */
#define LINE4_PIN_47_PIN                                        (DL_GPIO_PIN_20)
#define LINE4_PIN_47_IOMUX                                       (IOMUX_PINCM48)
/* Defines for PIN_48: GPIOB.13 with pinCMx 30 on package pin 1 */
#define LINE4_PIN_48_PIN                                        (DL_GPIO_PIN_13)
#define LINE4_PIN_48_IOMUX                                       (IOMUX_PINCM30)
/* Port definition for Pin Group KEY1 */
#define KEY1_PORT                                                        (GPIOA)

/* Defines for PIN_18: GPIOA.18 with pinCMx 40 on package pin 11 */
#define KEY1_PIN_18_PIN                                         (DL_GPIO_PIN_18)
#define KEY1_PIN_18_IOMUX                                        (IOMUX_PINCM40)
/* Defines for PIN_out: GPIOA.24 with pinCMx 54 on package pin 25 */
#define KEY1_PIN_out_PIN                                        (DL_GPIO_PIN_24)
#define KEY1_PIN_out_IOMUX                                       (IOMUX_PINCM54)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_UART_0_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
