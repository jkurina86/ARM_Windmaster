/**
  ******************************************************************************
  * @file    calculations.c
  * @brief   The system calculator functions
  * @note    Contains all of the calculation functions and the calculator service task.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "calculations.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Private defines -----------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
uint8_t calc_task_flag = 0;  /* Flag to indicate when calculation task should run */
uint8_t wind_speed_mean_flag = 0; /* Flag to indicate when wind speed mean calculation should run */
uint8_t gust_ready_flag = 0; /* Flag to indicate when gust speed is ready */
uint8_t yield_flag = 0; /* Yield flag for cooperative multitasking */

/* Public variables ----------------------------------------------------------*/
// A 3-dimensional buffer containing 60 U, V, and W values for gust calculation (3 seconds at 20 Hz)
int16_t gust_uv_buffer[60][3] = {0};
uint8_t gust_buffer_index = 0;

uint32_t wind_speed_mean[20] = {0};
uint8_t wind_speed_mean_index = 0;

uint32_t gust_speed = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/** 
 * @brief  Calculate wind speed mean 
 * @param  None
 * @retval None 
 */
void calculate_wind_speed_mean(void)
{
    uint32_t sum_u = 0;
    uint32_t sum_v = 0;

    /* Sum U and V components from the gust buffer */
    for (uint8_t i = 0; i < 60; i++) {
        sum_u += gust_uv_buffer[i][0];
        sum_v += gust_uv_buffer[i][1];
    }

    /* Calculate mean U and V */
    int16_t mean_u = (int16_t)(sum_u / 60);
    int16_t mean_v = (int16_t)(sum_v / 60);

    /* Calculate wind speed mean using Pythagorean theorem */
    uint32_t speed_mean = (uint32_t)sqrt((mean_u * mean_u) + (mean_v * mean_v));

    /* Store in circular buffer */
    wind_speed_mean[wind_speed_mean_index] = speed_mean;
    wind_speed_mean_index = (wind_speed_mean_index + 1) % 20;

    /* If we have 20 wind speed mean values, we select the highest as the gust value */
    if (wind_speed_mean_index == 0) {
        for (uint8_t i = 0; i < 20; i++) {
            if (wind_speed_mean[i] > gust_speed) {
                gust_speed = wind_speed_mean[i];
            }
        }
        gust_ready_flag = 1;
    }

    /* Reset the flag */
    wind_speed_mean_flag = 0;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Calculation service task to be called in the main loop
 * @param  None
 * @retval None
 */
void calculation_service(void)
{
    /* Early exit if flags aren't set */
    if (calc_task_flag == 0) {
        return;
    }

    /* Perform wind speed mean calculation if flag is set */
    if (wind_speed_mean_flag == 1 && yield_flag == 0) {
        calculate_wind_speed_mean();
        yield_flag = 1; /* Set yield flag to allow cooperative multitasking */
    }





    /* Reset the calculation task flag */
    calc_task_flag = 0;
    yield_flag = 0; /* Reset yield flag to allow next calculation */
}

/**
 * @brief  Add U and V wind data to the gust buffer
 * @param  u: U component of wind speed
 * @param  v: V component of wind speed
 * @retval None
 */
void add_uvw_data(int16_t u, int16_t v, int16_t w)
{
    gust_uv_buffer[gust_buffer_index][0] = u;
    gust_uv_buffer[gust_buffer_index][1] = v;
    gust_uv_buffer[gust_buffer_index][2] = w;
    gust_buffer_index = (gust_buffer_index + 1) % 60;

    if (gust_buffer_index == 0) {
        /* Set the wind speed mean calculation flag */
        wind_speed_mean_flag = 1;
        calc_task_flag = 1;
    }
}


