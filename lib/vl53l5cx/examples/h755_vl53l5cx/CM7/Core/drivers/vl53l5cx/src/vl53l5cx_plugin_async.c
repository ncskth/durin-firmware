#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_async.h"

uint8_t vl53l5cx_check_data_ready_async_start(VL53L5CX_Configuration *p_dev)
{
	if (p_dev->state != VL53L5CX_IDLE)
	{
		return VL53L5CX_MCU_ERROR;;
	}
	uint8_t status = VL53L5CX_STATUS_OK;
	status |= RdMultiAsync(&(p_dev->platform), 0x0, p_dev->temp_buffer, 4);
	if (status == 0)
	{
		p_dev->state = VL53L5CX_READING_DATA_AVAILABLE;
	}
	return status;
}

uint8_t vl53l5cx_check_data_ready_async_in_progress(VL53L5CX_Configuration *p_dev)
{
	return p_dev->state == VL53L5CX_READING_DATA_AVAILABLE;
}

uint8_t vl53l5cx_check_data_ready_async_finish(VL53L5CX_Configuration *p_dev,
		uint8_t *p_isReady)
{
	if ((p_dev->temp_buffer[0] != p_dev->streamcount)
			&& (p_dev->temp_buffer[0] != (uint8_t) 255)
			&& (p_dev->temp_buffer[1] == (uint8_t) 0x5)
			&& ((p_dev->temp_buffer[2] & (uint8_t) 0x5) == (uint8_t) 0x5)
			&& ((p_dev->temp_buffer[3] & (uint8_t) 0x10) == (uint8_t) 0x10))
	{
		*p_isReady = (uint8_t) 1;
		p_dev->streamcount = p_dev->temp_buffer[0];
	}
	else
	{
		*p_isReady = 0;
	}
	p_dev->state = VL53L5CX_IDLE;
	return VL53L5CX_STATUS_OK;
}

uint8_t vl53l5cx_get_ranging_data_async_start(VL53L5CX_Configuration *p_dev)
{
	if (p_dev->state != VL53L5CX_IDLE)
	{
		return VL53L5CX_MCU_ERROR;
	}
	uint8_t status = VL53L5CX_STATUS_OK;
	status |= RdMultiAsync(&(p_dev->platform), 0x0, p_dev->temp_buffer,
			p_dev->data_read_size);
	if (status == VL53L5CX_STATUS_OK)
	{
		p_dev->state = VL53L5CX_READING_RANGING_MEASUREMENT;
	}
	return status;
}

uint8_t vl53l5cx_get_ranging_data_async_in_progress(VL53L5CX_Configuration *p_dev)
{
	return VL53L5CX_READING_RANGING_MEASUREMENT == p_dev->state;
}

uint8_t vl53l5cx_get_ranging_data_async_finish(VL53L5CX_Configuration *p_dev,
		VL53L5CX_ResultsData *p_results)
{
	union Block_header *bh_ptr;
	uint32_t i, j, msize;
	p_dev->streamcount = p_dev->temp_buffer[0];
	SwapBuffer(p_dev->temp_buffer, (uint16_t) p_dev->data_read_size);

	/* Start conversion at position 16 to avoid headers */
	for (i = (uint32_t) 16; i < (uint32_t) p_dev->data_read_size; i +=
			(uint32_t) 4)
	{
		bh_ptr = (union Block_header*) &(p_dev->temp_buffer[i]);
		if ((bh_ptr->type > (uint32_t) 0x1) && (bh_ptr->type < (uint32_t) 0xd))
		{
			msize = bh_ptr->type * bh_ptr->size;
		}
		else
		{
			msize = bh_ptr->size;
		}

		switch (bh_ptr->idx)
		{
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
		case VL53L5CX_AMBIENT_RATE_IDX:
			(void) memcpy(p_results->ambient_per_spad,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
		case VL53L5CX_SPAD_COUNT_IDX:
			(void) memcpy(p_results->nb_spads_enabled,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
		case VL53L5CX_NB_TARGET_DETECTED_IDX:
			(void) memcpy(p_results->nb_target_detected,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
		case VL53L5CX_SIGNAL_RATE_IDX:
			(void) memcpy(p_results->signal_per_spad,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
		case VL53L5CX_RANGE_SIGMA_MM_IDX:
			(void) memcpy(p_results->range_sigma_mm,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
		case VL53L5CX_DISTANCE_IDX:
			(void) memcpy(p_results->distance_mm,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
		case VL53L5CX_REFLECTANCE_EST_PC_IDX:
			(void) memcpy(p_results->reflectance,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
		case VL53L5CX_TARGET_STATUS_IDX:
			(void) memcpy(p_results->target_status,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
		case VL53L5CX_MOTION_DETEC_IDX:
			(void) memcpy(&p_results->motion_indicator,
					&(p_dev->temp_buffer[i + (uint32_t) 4]), msize);
			break;
#endif
		default:
			break;
		}
		i += msize;
	}

#ifndef VL53L5CX_USE_RAW_FORMAT

	/* Convert data into their real format */
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
	for (i = 0; i < (uint32_t) VL53L5CX_RESOLUTION_8X8; i++)
	{
		p_results->ambient_per_spad[i] /= (uint32_t) 2048;
	}
#endif

	for (i = 0;
			i
					< (uint32_t) (VL53L5CX_RESOLUTION_8X8
							* VL53L5CX_NB_TARGET_PER_ZONE); i++)
	{
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
		p_results->distance_mm[i] /= 4;
		if (p_results->distance_mm[i] < 0)
		{
			p_results->distance_mm[i] = 0;
		}
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
		p_results->range_sigma_mm[i] /= (uint16_t) 128;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
		p_results->signal_per_spad[i] /= (uint32_t) 2048;
#endif
	}

	/* Set target status to 255 if no target is detected for this zone */
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
	for (i = 0; i < (uint32_t) VL53L5CX_RESOLUTION_8X8; i++)
	{
		if (p_results->nb_target_detected[i] == (uint8_t) 0)
		{
			for (j = 0; j < (uint32_t)
			VL53L5CX_NB_TARGET_PER_ZONE; j++)
			{
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
				p_results->target_status[((uint32_t) VL53L5CX_NB_TARGET_PER_ZONE
						* (uint32_t) i) + j] = (uint8_t) 255;
#endif
			}
		}
	}
#endif

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
	for (i = 0; i < (uint32_t) 32; i++)
	{
		p_results->motion_indicator.motion[i] /= (uint32_t) 65535;
	}
#endif

#endif
	p_dev->state = VL53L5CX_IDLE;
	return VL53L5CX_STATUS_OK;
}
