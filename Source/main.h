/**
  ******************************************************************************
  * File Name          : main.h
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __main_H
#define __main_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
 
   
/* Definition for USARTx's NVIC */

#define BRD_REV         "Rev K.5"               // PCB Revision          
#define VERSION_NUM     "K.5.9"                 // Monitor Revision
#define REL_DATE        "Nov 6, 2017"
//#define LEGACY_BANNER   "Rev G+ REV C"        // OLD.....Needed to allow Legacy Design to work
#define LEGACY_BANNER   "K.5.9 11/6/17"        // Needed to allow Legacy Design to work

/** 
  * @brief  HAL Status structures definition  
  */  
typedef enum 
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

/* Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
