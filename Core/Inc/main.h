/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TC1_CS_Pin GPIO_PIN_2
#define TC1_CS_GPIO_Port GPIOE
#define TC5_CS_Pin GPIO_PIN_3
#define TC5_CS_GPIO_Port GPIOE
#define TC2_CS_Pin GPIO_PIN_4
#define TC2_CS_GPIO_Port GPIOE
#define TC3_CS_Pin GPIO_PIN_5
#define TC3_CS_GPIO_Port GPIOE
#define TC4_CS_Pin GPIO_PIN_6
#define TC4_CS_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define TC6_CS_Pin GPIO_PIN_8
#define TC6_CS_GPIO_Port GPIOF
#define GRIPPER_Pin GPIO_PIN_9
#define GRIPPER_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define CONMUTADOR_Pin GPIO_PIN_0
#define CONMUTADOR_GPIO_Port GPIOC
#define SLAVE_MOSI_Pin GPIO_PIN_3
#define SLAVE_MOSI_GPIO_Port GPIOC
#define HMI_TX_Pin GPIO_PIN_2
#define HMI_TX_GPIO_Port GPIOA
#define HMI_RX_Pin GPIO_PIN_3
#define HMI_RX_GPIO_Port GPIOA
#define Master_SCK_Pin GPIO_PIN_5
#define Master_SCK_GPIO_Port GPIOA
#define Master_MOSI_Pin GPIO_PIN_7
#define Master_MOSI_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define SPC_Pin GPIO_PIN_1
#define SPC_GPIO_Port GPIOB
#define TC_1_Pin GPIO_PIN_13
#define TC_1_GPIO_Port GPIOF
#define TC_2_Pin GPIO_PIN_14
#define TC_2_GPIO_Port GPIOF
#define TC_3_Pin GPIO_PIN_15
#define TC_3_GPIO_Port GPIOF
#define TC_4_Pin GPIO_PIN_0
#define TC_4_GPIO_Port GPIOG
#define TC_5_Pin GPIO_PIN_1
#define TC_5_GPIO_Port GPIOG
#define TC_6_Pin GPIO_PIN_7
#define TC_6_GPIO_Port GPIOE
#define TC1_ready_Pin GPIO_PIN_10
#define TC1_ready_GPIO_Port GPIOE
#define TC2_ready_Pin GPIO_PIN_11
#define TC2_ready_GPIO_Port GPIOE
#define TC3_ready_Pin GPIO_PIN_12
#define TC3_ready_GPIO_Port GPIOE
#define TC4_ready_Pin GPIO_PIN_13
#define TC4_ready_GPIO_Port GPIOE
#define TC5_ready_Pin GPIO_PIN_14
#define TC5_ready_GPIO_Port GPIOE
#define TC6_ready_Pin GPIO_PIN_15
#define TC6_ready_GPIO_Port GPIOE
#define SLAVE_SCK_Pin GPIO_PIN_10
#define SLAVE_SCK_GPIO_Port GPIOB
#define SLAVE_CS_Pin GPIO_PIN_12
#define SLAVE_CS_GPIO_Port GPIOB
#define SPI_SLAVE_EN_Pin GPIO_PIN_13
#define SPI_SLAVE_EN_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define USB_Rx_Pin GPIO_PIN_8
#define USB_Rx_GPIO_Port GPIOD
#define USB_Tx_Pin GPIO_PIN_9
#define USB_Tx_GPIO_Port GPIOD
#define DMS_Pin GPIO_PIN_2
#define DMS_GPIO_Port GPIOG
#define PARO_Pin GPIO_PIN_3
#define PARO_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define WiFi_Tx_Pin GPIO_PIN_6
#define WiFi_Tx_GPIO_Port GPIOC
#define WiFi_Rx_Pin GPIO_PIN_7
#define WiFi_Rx_GPIO_Port GPIOC
#define BT_Tx_Pin GPIO_PIN_9
#define BT_Tx_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DHT11_Pin GPIO_PIN_2
#define DHT11_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define BT_Rx_Pin GPIO_PIN_7
#define BT_Rx_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
