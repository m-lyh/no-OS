/***************************************************************************//**
 *   @file   ad7091r5.c
 *   @brief  Implementation of ad69x Driver.
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ad7091r5.h"
#include <string.h>
#include "stdio.h"
#include "stdlib.h"
#include "error.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * Read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7091r5_spi_reg_read(struct ad7091r5_dev *dev,
			    uint16_t reg_addr,
			    uint8_t *reg_data)
{
	int32_t ret;
	uint8_t buf[3];


	return ret;
}

/**
 * Write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @@eturn SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7091r5_spi_reg_write(struct ad7091r5_dev *dev,
			     uint16_t reg_addr,
			     uint8_t reg_data)
{
	int32_t ret;
	uint8_t buf[3];


	return ret;
}

/**
 * SPI read from device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7091r5_spi_read_mask(struct ad7091r5_dev *dev,
			     uint16_t reg_addr,
			     uint8_t mask,
			     uint8_t *data)
{
	uint8_t reg_data[3];
	int32_t ret;


	return ret;
}

/**
 * SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7091r5_spi_write_mask(struct ad7091r5_dev *dev,
			      uint16_t reg_addr,
			      uint8_t mask,
			      uint8_t data)
{
	uint8_t reg_data;
	int32_t ret;

	return 0;
}

/**
 * @brief Initialize GPIO driver handlers for the GPIOs in the system.
 *        ad7091r5_init() helper function.
 * @param [out] dev - ad7091r5_dev device handler.
 * @param [in] init_param - Pointer to the initialization structure.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
static int32_t ad7091r5_init_gpio(struct ad7091r5_dev *dev,
				struct ad7091r5_init_param *init_param)
{
	int32_t ret;

	ret = gpio_get_optional(&dev->gpio_resetn, init_param->gpio_resetn);
	if (ret != SUCCESS)
		return ret;

	/** Reset to configure pins */
	if (init_param->gpio_resetn) {
		ret = gpio_direction_output(dev->gpio_resetn, GPIO_LOW);
		if (ret != SUCCESS)
			return ret;

		mdelay(100);
		ret = gpio_set_value(dev->gpio_resetn, GPIO_HIGH);
		if (ret != SUCCESS)
			return ret;

		mdelay(100);
	}

	return SUCCESS;
}

/**
 * Initialize the device.
 * @param [out] device - The device structure.
 * @param [in] init_param - The structure that contains the device initial
 * 		parameters.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t ad7091r5_init(struct ad7091r5_dev **device,
		    struct ad7091r5_init_param *init_param)
{
	struct ad7091r5_dev *dev;
	int32_t ret;
	uint8_t data = 0;

	dev = (struct ad7091r5_dev *)malloc(sizeof(*dev));
	if (!dev)
		return FAILURE;


	ret = ad7091r5_init_gpio(dev, init_param);
	if (ret != SUCCESS)
		return FAILURE;

	ret = i2c_init(&dev->i2c_desc, init_param->i2c_init);
	if (ret != SUCCESS)
		return FAILURE;


	*device = dev;

	return ret;

	return SUCCESS;
}

/**
 * @brief Free the memory allocated by ad7091r5_init().
 * @param [in] dev - Pointer to the device handler.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise
 */
int32_t ad7091r5_remove(struct ad7091r5_dev *dev)
{
	int32_t ret;

	if (!dev)
		return FAILURE;

	ret = i2c_remove(dev->i2c_desc);
	if (ret != SUCCESS)
		return ret;

	ret = gpio_remove(dev->gpio_resetn);
	if (ret != SUCCESS)
		return ret;

	free(dev);

	return ret;
}
