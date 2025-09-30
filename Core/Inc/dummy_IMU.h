/**
  ******************************************************************************
  * @file    dummy_IMU.h
  * @brief   Dummy IMU functions
  * @note    Placeholder for IMU functionality
  ******************************************************************************
  */

#ifndef INC_DUMMY_IMU_H_
#define INC_DUMMY_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Exported Types -----------------------------------------------------------*/
typedef enum {
    IMU_STATE_STOPPED,
    IMU_STATE_RUNNING
} IMU_State_t;

/* Struct used to test Timing from the Python dummy IMU sensor */
typedef struct {
  char identifier[6];           /* ASCII: Identifies the sensor */
  uint32_t packet_number;       /* Tagged packet number */
} IMU_Test_t;

/* Function Prototypes -------------------------------------------------------*/
void dummy_IMU_init(void);
void dummy_IMU_start(uint64_t *start_time);
void dummy_IMU_stop(uint64_t *stop_time);
bool dummy_IMU_is_running(void);

void dummy_IMU_get_test_data(IMU_Test_t *data);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* INC_DUMMY_IMU_H_ */