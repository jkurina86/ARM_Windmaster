/**
  ******************************************************************************
  * @file    dummy_WM.h
  * @brief   Dummy WindMaster functions
  * @note    Placeholder for WindMaster functionality
  ******************************************************************************
  */

#ifndef INC_DUMMY_WM_H_
#define INC_DUMMY_WM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Exported Types -----------------------------------------------------------*/
typedef enum {
    WM_STATE_STOPPED,
    WM_STATE_RUNNING
} WM_State_t;

/* WindMaster Data Structure: Based on Mode 10 - Binary, UVW, Long in WM manual pg. 41 */
/*
typedef struct {
  char status;
  int16_t U_axis_speed;   // U-axis wind speed
  int16_t V_axis_speed;   // V-axis wind speed
  int16_t W_axis_speed;   // W-axis wind speed
  int16_t SoS;            // Speed of Sound
  int16_t A1;             // Analog Input 1
  int16_t A2;             // Analog Input 2
  int16_t A3;             // Analog Input 3
  int16_t A4;             // Analog Input 4
  int16_t Temp;           // Temperature from PRT
  uint64_t timestamp;     // Timestamp of the data
} WM_Data_t;
*/

/* Struct used to test Timing from the Python dummy WM sensor */
typedef struct {
  char identifier[5];     /* ASCII: Identifies the sensor */
  uint32_t packet_number; /* Tagged packet number */
} WM_Test_t;

/* Function Prototypes -------------------------------------------------------*/
void dummy_WM_init(void);
void dummy_WM_start(uint64_t *start_time);
void dummy_WM_stop(uint64_t *stop_time);
bool dummy_WM_is_running(void);
//void dummy_WM_get_data(WM_Data_t* data);
//const char* dummy_WM_log(WM_Data_t* data);

void dummy_WM_get_test_data(WM_Test_t* test);
bool parse_packet_test(uint8_t* buffer_start, uint16_t length, WM_Test_t* test);

#endif /* INC_DUMMY_WM_H_ */
