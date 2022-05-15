/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, the VL53L5CX Ultra Lite Driver may be distributed under the
* terms of 'BSD 3-clause "New" or "Revised" License', in which case the
* following provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "platform.h"

uint8_t RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value)
{
	RdMulti(p_platform, RegisterAdress, p_value, 1);
	return 0;
}

uint8_t WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value)
{
	WrMulti(p_platform, RegisterAdress, &value, 1);
	return 0;
}

uint8_t WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint32_t max_send_size = 255;
	uint32_t has_sent = 0;
	while (1) {
		uint32_t to_send;
		to_send = size - has_sent <= max_send_size ? size - has_sent : max_send_size;

		nbe_i2c_start_write(p_platform->nbe_i2c, p_platform->address, p_values + has_sent, NULL);
		uint8_t updated_address[2];
		updated_address[0] = ((RegisterAdress + has_sent) >> 8) & 0x00ff;
		updated_address[1] = (RegisterAdress + has_sent) & 0x00ff;
		nbe_i2c_write_preamble(p_platform->nbe_i2c, updated_address, 2);
		nbe_i2c_write(p_platform->nbe_i2c, to_send);
		nbe_i2c_stop(p_platform->nbe_i2c);
		nbe_i2c_commit(p_platform->nbe_i2c);
		while (nbe_i2c_is_busy(p_platform->nbe_i2c)){}
		has_sent += to_send;

		if (has_sent - size == 0) {
			break;
		}
	}
	return 0;
}

uint8_t WrMultiAsync(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t data_write[2];
	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	nbe_i2c_start_write(p_platform->nbe_i2c, p_platform->address, p_values, NULL);
	nbe_i2c_write_preamble(p_platform->nbe_i2c, data_write, 2);
	nbe_i2c_write(p_platform->nbe_i2c, size);
	nbe_i2c_stop(p_platform->nbe_i2c);
	nbe_i2c_commit(p_platform->nbe_i2c);
	return 0;
}


uint8_t RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{

	uint16_t has_read = 0;
	uint16_t max_read_size = 255; 
	while (1) {
		uint16_t to_read =  size - has_read <= max_read_size ? size - has_read : max_read_size;
		RdMultiAsync(p_platform, RegisterAdress + has_read, p_values + has_read, to_read);
		while (nbe_i2c_is_busy(p_platform->nbe_i2c)){}
		has_read += to_read;
		if (has_read - size == 0) {
			break;
		}

	}
	return 0;
}

uint8_t RdMultiAsync(
	VL53L5CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size)
{
	uint8_t data_write[2];
	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;

	nbe_i2c_start_write(p_platform->nbe_i2c, p_platform->address, NULL, p_values);
	nbe_i2c_write_preamble(p_platform->nbe_i2c, data_write, 2);
	nbe_i2c_start(p_platform->nbe_i2c);
	uint8_t byte = i2c_first_byte_read(p_platform->address);
	nbe_i2c_write_preamble(p_platform->nbe_i2c, &byte, 1);
	if (size > 1) {
		if (size > 256) {
			nbe_i2c_read_ack(p_platform->nbe_i2c, 255);
			nbe_i2c_read_ack(p_platform->nbe_i2c, size - 256);

		} else {
			nbe_i2c_read_ack(p_platform->nbe_i2c, size - 1);
		}
	}
	nbe_i2c_read_nak(p_platform->nbe_i2c, 1);
	nbe_i2c_stop(p_platform->nbe_i2c);
	nbe_i2c_commit(p_platform->nbe_i2c);
	return 0;
}

uint8_t Reset_Sensor(VL53L5CX_Platform *p_platform)
{
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	WaitMs(p_platform, 100);
  
	return 0;
}

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t WaitMs(
		VL53L5CX_Platform *p_platform,
               uint32_t TimeMs)
{
	vTaskDelay(TimeMs / portTICK_PERIOD_MS);
	return 0;
}