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
#include "delay.h"
#include "util.h"

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
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r5_i2c_reg_read(struct ad7091r5_dev *dev,
			     uint8_t reg_addr,
			     uint16_t *reg_data)
{
	uint8_t buf[2];
	int32_t ret;

	ret = i2c_write(dev->i2c_desc, &reg_addr, 1, 0);
	if (ret < 0)
		return ret;

	ret = i2c_read(dev->i2c_desc, buf, 2, 0);
	if (ret < 0)
		return ret;

	*reg_data = (buf[1] << 8) | buf[0];

	return ret;
}

/**
 * Write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r5_i2c_reg_write(struct ad7091r5_dev *dev,
			      uint8_t reg_addr,
			      uint16_t reg_data)
{
	uint8_t buf[3];

	buf[0] = reg_addr;
	buf[1] = (reg_data & 0xFF00) >> 8;
	buf[2] = reg_data & 0xFF;

	return i2c_write(dev->i2c_desc, buf, ARRAY_SIZE(buf), 0);
}

/**
 * Multibyte read from device. A register read begins with the address
 * and autoincrements for each aditional byte in the transfer.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @param count - Number of bytes to read.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r5_i2c_reg_read_multiple(struct ad7091r5_dev *dev,
				      uint8_t reg_addr,
				      uint8_t *reg_data,
				      uint16_t count)
{
	uint8_t buf[512];
	int32_t ret;

	if (count > 512)
		return FAILURE;

	buf[0] = reg_addr;
	memset(&buf[1], 0x00, count - 1);

	ret = i2c_write(dev->i2c_desc, buf, 1, 0);
	if (ret < 0)
		return ret;

	ret = i2c_read(dev->i2c_desc, buf, count, 0);
	if (ret < 0)
		return ret;

	memcpy(reg_data, buf, count);

	return ret;
}


/**
 * SPI read from device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r5_i2c_read_mask(struct ad7091r5_dev *dev,
			     uint8_t reg_addr,
			     uint8_t mask,
			     uint8_t *data)
{
	uint8_t reg_data;
	int32_t ret;

	ret = ad7091r5_i2c_reg_read(dev, reg_addr, &reg_data);
	*data = (reg_data & mask);

	return ret;
}

/**
 * SPI write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad7091r5_spi_write_mask(struct ad7091r5_dev *dev,
			      uint8_t reg_addr,
			      uint8_t mask,
			      uint8_t data)
{
	uint8_t reg_data;
	int32_t ret;

	ret = ad7091r5_i2c_reg_read(dev, reg_addr, &reg_data);
	reg_data &= ~mask;
	reg_data |= data;
	ret |= ad7091r5_i2c_reg_write(dev, reg_addr, reg_data);

	return ret;
}

static int ad7091r_set_mode(struct ad7091r_state *st, enum ad7091r_mode mode)
{
	int ret;

	switch (mode) {
	case AD7091R_MODE_SAMPLE:
		ret = regmap_update_bits(st->map, AD7091R_REG_CONF,
				REG_CONF_MODE_MASK, 0);
		break;
	case AD7091R_MODE_COMMAND:
		ret = regmap_update_bits(st->map, AD7091R_REG_CONF,
				REG_CONF_MODE_MASK, REG_CONF_CMD);
		break;
	case AD7091R_MODE_AUTOCYCLE:
		ret = regmap_update_bits(st->map, AD7091R_REG_CONF,
				REG_CONF_MODE_MASK, REG_CONF_AUTO);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (!ret)
		st->mode = mode;
	return ret;
}

static int ad7091r_set_channel(struct ad7091r_state *st, unsigned channel)
{
	unsigned int foo;
	int ret;

	/* AD7091R_REG_CHANNEL is a 8-bit register */
	ret = regmap_write(st->map, AD7091R_REG_CHANNEL,
			BIT(channel) | (BIT(channel) << 8));
	if (ret)
		return ret;

	/* There is a latency of one conversion before the channel conversion
	 * sequence is updated */
	return regmap_read(st->map, AD7091R_REG_RESULT, &foo);
}

static int ad7091r_read_one(struct iio_dev *iio_dev,
		unsigned channel, unsigned int *read_val)
{
	struct ad7091r_state *st = iio_priv(iio_dev);
	unsigned val;
	int ret;

	/* TODO: locking */
	ret = ad7091r_set_channel(st, channel);
	if (ret)
		return ret;

	ret = regmap_read(st->map, AD7091R_REG_RESULT, &val);
	if (ret)
		return ret;

	if (REG_RESULT_CH_ID(val) != channel)
		return -EIO;

	*read_val = REG_RESULT_CONV_RESULT(val);
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

		udelay(1);
		ret = gpio_set_value(dev->gpio_resetn, GPIO_HIGH);
		if (ret != SUCCESS)
			return ret;

		udelay(1);
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
