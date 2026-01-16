/**
  ******************************************************************************
  * @file    telus.h
  * @brief   Telus communication module header
  * @note    Handles UART4 communication with Telus system.
  *          Receives "idata\r\n" commands and responds with CSV-formatted
  *          CalcReport data via DMA.
  ******************************************************************************
  */
#ifndef __TELUS_H__
#define __TELUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Public function prototypes ------------------------------------------------*/

void telus_init(void);
void telus_service(void);
void telus_tx_complete(void);

#ifdef __cplusplus
}
#endif
#endif /* __TELUS_H__ */
