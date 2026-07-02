/**
  ******************************************************************************
  * @file    telos.h
  * @brief   TELOS communication module header
  * @note    Handles USART2 communication with TELOS system.
  *          Receives "idata\r\n" commands and responds with CSV-formatted
  *          CalcReport data via DMA.
  ******************************************************************************
  */
#ifndef __TELOS_H__
#define __TELOS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Public function prototypes ------------------------------------------------*/

void telos_init(void);
void telos_service(void);
void telos_tx_complete(void);
void telos_tx_error(void);

#ifdef __cplusplus
}
#endif
#endif /* __TELOS_H__ */
