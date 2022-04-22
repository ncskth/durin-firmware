#include <stdint.h>
#include "vl53l5cx_api.h"

/**
 * @brief This function is used to check if the sensor has data ready for reading. When an answer has been received from the sensor,
 * the HAL_I2C_MemRxCpltCallback will be called and there you can check the answer via vl53l5cx_check_data_ready_async_finish.
 * @param (VL53L5CX_Configuration) *p_dev : VL53L5CX configuration structure.
 * @return (uint8_t) status : 0 if request was sent to the sensor without error.
 */
uint8_t vl53l5cx_check_data_ready_async_start(VL53L5CX_Configuration *p_dev);

/**
 * @brief This function is used to check last request to the sensor was a "check data ready" request. If so, then it is safe to
 * use vl53l5cx_check_data_ready_async_finish to receive the response.
 * @param (VL53L5CX_Configuration) *p_dev : VL53L5CX configuration structure.
 * @return (uint8_t) 1 if a "check data ready" was the last request made to the sensor. Otherwise 0.
 */
uint8_t vl53l5cx_check_data_ready_async_in_progress(VL53L5CX_Configuration *p_dev);

/**
 * @brief This function is used to read the local buffer where sensor response was stored regarding if it has data ready to be read.
 * Only safe to use if vl53l5cx_check_data_ready_async_start was the last request sent to the sensor which can be
 * checked by using vl53l5cx_check_data_ready_async_in_progress.
 * @param (VL53L5CX_Configuration) *p_dev : VL53L5CX configuration structure.
 * @param (uint8_t) *p_result : Value of this pointer be updated to 0 if data
 * is not ready, or 1 if a new data is ready.
 * @return (uint8_t) is always VL53L5CX_STATUS_OK.
 */
uint8_t vl53l5cx_check_data_ready_async_finish(VL53L5CX_Configuration *p_dev,
		uint8_t *p_result);

/**
 * @brief This function is used to request ranging measurement from the sensor. When an answer has been received from the sensor,
 * the HAL_I2C_MemRxCpltCallback will be called and there you can check the answer via @vl53l5cx_get_ranging_data_async_finish.
 * This function should only be used if you are certain the sensor has data ready.
 * @param (VL53L5CX_Configuration) *p_dev : VL53L5CX configuration structure.
 * @return (uint8_t) status : 0 if request was sent to the sensor without error.
 */
uint8_t vl53l5cx_get_ranging_data_async_start(VL53L5CX_Configuration *p_dev);

/**
 * @brief This function is used to check if last request to the sensor was a "get ranging data" request. If so, then it is safe to
 * use @vl53l5cx_get_ranging_data_async_finish to receive the response.
 * @param (VL53L5CX_Configuration) *p_dev : VL53L5CX configuration structure.
 * @return (uint8_t) 1 if a "get ranging data" was the last request made to the sensor. Otherwise 0.
 */
uint8_t vl53l5cx_get_ranging_data_async_in_progress(VL53L5CX_Configuration *p_dev);

/**
 * @brief This function is used to read the local buffer where the sensor measurement was stored. Only safe to use if
 * vl53l5cx_get_ranging_data_async_start was the last request sent to the sensor which can be checked by using
 * vl53l5cx_get_ranging_data_async_in_progress.
 * @param (VL53L5CX_Configuration) *p_dev : VL53L5CX configuration structure.
 * @param (VL53L5CX_ResultsData) *p_results : VL53L5 results structure.
 * @return (uint8_t) always VL53L5CX_STATUS_OK.
 */
uint8_t vl53l5cx_get_ranging_data_async_finish(VL53L5CX_Configuration *p_dev,
		VL53L5CX_ResultsData *p_results);
