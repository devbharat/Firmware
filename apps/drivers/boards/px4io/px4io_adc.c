/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4io_adc.c
 *
 * Board-specific ADC functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include <stm32_internal.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32_adc.h"
#include "px4io_internal.h"

int adc_devinit(void);

#define ADC1_NCHANNELS 1

// XXX SUPER-HACK!
#define GPIO_ADC1_IN4       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* The PX4FMU board has two ADC channels: ADC1 ports 4-5
 */

/* Identifying number of each ADC channel: Variable Resistor. */

#ifdef CONFIG_STM32_ADC1
static const uint8_t  g_chanlist[ADC1_NCHANNELS] = {4};

/* Configurations of pins used byte each ADC channels */
static const uint32_t g_pinlist[ADC1_NCHANNELS]  = {GPIO_ADC1_IN4};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: adc_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/adc.
 *
 ************************************************************************************/

int adc_devinit(void)
{
	static bool initialized = false;
	struct adc_dev_s *adc;
	int ret;
	int i;

	/* Check if we have already initialized */

	if (!initialized) {

		for (i = 0; i < ADC1_NCHANNELS; i++) {
			/* Configure the pins as analog inputs for the selected channels */
			stm32_configgpio(g_pinlist[i]);
		}

		/*
		 * Call stm32_adcinitialize() to get an instance of the ADC interface
		 * multiple channels only supported with dma!
		 */
		adc = stm32_adcinitialize(1, (g_chanlist), 1);

		if (adc == NULL) {
			adbg("ERROR: Failed to get ADC interface\n");
			return -ENODEV;
		}

		/* Register the ADC driver at "/dev/adc0" */
		ret = adc_register("/dev/adc0", adc);

		if (ret < 0) {
			adbg("adc_register failed for /dev/adc0: %d\n", ret);
			return ret;
		}

		/* Now we are initialized */
		initialized = true;
	}

	return OK;
}
