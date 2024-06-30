/****************************************************************************
 * arch/risc-v/src/bl808/bl808_gpadc.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/analog/adc.h>

#include "hardware/bl808_gpadc.h"
#include "riscv_internal.h"
#include "chip.h"
#include "bl808_gpadc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BL808_GPADC_TOTAL_NCHANNELS 18
#define BL808_GPADC_SCAN_MAX_CHANNELS 12

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl808_gpadc_s
{
  const struct adc_callback_s *callback;
  bool channel_enable[BL808_GPADC_TOTAL_NCHANNELS];
  uint8_t nchannels;
};

enum bl808_gpadc_channel_e
{
  GPADC_CH0,
  GPADC_CH1,
  GPADC_CH2,
  GPADC_CH3,
  GPADC_CH4,
  GPADC_CH5,
  GPADC_CH6,
  GPADC_CH7,
  GPADC_CH8,
  GPADC_CH9,
  GPADC_CH10,
  GPADC_CH11,
  GPADC_CH_DAC_OUTA,
  GPADC_CH_DAC_OUTB,
  GPADC_CH_HALF_VBAT,
  GPADC_CH_TSEN,
  GPADC_CH_VREF,
  GPADC_CH_GND
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bl808_gpadc_bind(struct adc_dev_s *dev,
                            const struct adc_callback_s *callback);
static void bl808_gpadc_reset(struct adc_dev_s *dev);
static int bl808_gpadc_setup(struct adc_dev_s *dev);
static void bl808_gpadc_shutdown(struct adc_dev_s *dev);
static void bl808_gpadc_rxint(struct adc_dev_s *dev,
                              bool enable);
static int bl808_gpadc_ioctl(struct adc_dev_s *dev,
                             int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bl808_gpadc_s gpadc_priv =
{
  .callback = NULL,

  /* channel_enable determines which channels will be part of the scan */

  .channel_enable =
  {
    [GPADC_CH0] = true,
    [GPADC_CH1] = true,
    [GPADC_CH2] = true,
    [GPADC_CH3] = true,
    [GPADC_CH4] = true,
    [GPADC_CH5] = true,
    [GPADC_CH6] = true,
    [GPADC_CH7] = true,
    [GPADC_CH8] = true,
    [GPADC_CH9] = true,
    [GPADC_CH10] = true,
    [GPADC_CH11] = true,
    [GPADC_CH_DAC_OUTA] = false,
    [GPADC_CH_DAC_OUTB] = false,
    [GPADC_CH_HALF_VBAT] = false,
    [GPADC_CH_TSEN] = false,
    [GPADC_CH_VREF] = false,
    [GPADC_CH_GND] = false,
  },

  .nchannels = 12
};
  
static struct adc_ops_s gpadc_ops =
{
  .ao_bind = bl808_gpadc_bind,
  .ao_reset = bl808_gpadc_reset,
  .ao_setup = bl808_gpadc_setup,
  .ao_shutdown = bl808_gpadc_shutdown,
  .ao_rxint = bl808_gpadc_rxint,
  .ao_ioctl = bl808_gpadc_ioctl
};

static int bl808_gpadc_bind(struct adc_dev_s *dev,
                            const struct adc_callback_s *callback)
{
  ((struct bl808_gpadc_s *)(dev->ad_priv))->callback = callback;

  return OK;
}

static void bl808_gpadc_reset(struct adc_dev_s *dev)
{
  modifyreg32(BL808_GPADC_CMD, 0, GPADC_SOFT_RST);

  /*
  for (int i = 0; i < 10; i++)
    {
      asm("nop");
    }
  */
  
  modifyreg32(BL808_GPADC_CMD, GPADC_SOFT_RST, 0);
}

static int bl808_gpadc_setup(struct adc_dev_s *dev)
{
  struct bl808_gpadc_s *priv = dev->ad_priv;
  
  /* The setup process below is mostly taken from bouffalo_sdk */
  
  /* Disable and reenable ADC */
  
  modifyreg32(BL808_GPADC_CMD, GPADC_GLOBAL_EN, 0);
  modifyreg32(BL808_GPADC_CMD, 0, GPADC_GLOBAL_EN);

  /* Soft reset */
  
  modifyreg32(BL808_GPADC_CMD, 0, GPADC_SOFT_RST);

  /*
  for (int i = 0; i < 10; i++)
    {
      asm("nop");
    }
  */
  
  modifyreg32(BL808_GPADC_CMD, GPADC_SOFT_RST, 0);

  modifyreg32(BL808_GPADC_CONFIG1, 0,
              (2 << GPADC_V18_SEL_SHIFT)
              | (1 << GPADC_V11_SEL_SHIFT)
              | GPADC_SCAN_EN
              | GPADC_CLK_ANA_INV);

  /* Use GND as negative channel for now */
  
  modifyreg32(BL808_GPADC_CMD, 0, GPADC_NEG_GND);

  modifyreg32(BL808_GPADC_CONFIG, 0,
	      GPADC_RDY_CLR
	      | GPADC_FIFO_OVERRUN_CLR
	      | GPADC_FIFO_UNDERRUN_CLR);

  /* Set scan channels */
  
  modifyreg32(BL808_GPADC_SCAN_POS1,
	      0xffffffff, 0);
  modifyreg32(BL808_GPADC_SCAN_POS2,
	      0xffffffff, 0);
  
  uint8_t enabled_channels = 0;
  for (int channel_idx = 0;
       channel_idx < BL808_GPADC_TOTAL_NCHANNELS;
       channel_idx++)
    {
      if (priv->channel_enable[channel_idx])
	{
	  if (enabled_channels < 6)
	    {
	      modifyreg32(BL808_GPADC_SCAN_POS1, 0,
			  (channel_idx
			   << GPADC_SCAN_POS_SHIFT(enabled_channels)));
	    }
	  else if (enabled_channels < BL808_GPADC_SCAN_MAX_CHANNELS)
	    {
	      modifyreg32(BL808_GPADC_SCAN_POS2, 0,
			  (channel_idx
			   << GPADC_SCAN_POS_SHIFT(enabled_channels)));
	    }
	  else
	    {
	      /* Attempt to enable more channels than possible */

	      PANIC();
	    }
	  enabled_channels++;
	}
    }

  modifyreg32(BL808_GPADC_CONFIG1, 0,
	      (enabled_channels << GPADC_SCAN_LENGTH_SHIFT));

  return OK;
}

static void bl808_gpadc_shutdown(struct adc_dev_s *dev)
{
}

static void bl808_gpadc_rxint(struct adc_dev_s *dev,
                              bool enable)
{
}

static int bl808_gpadc_ioctl(struct adc_dev_s *dev,
                             int cmd, unsigned long arg)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bl808_adc_init(void)
{
  struct adc_dev_s *dev = zalloc(sizeof(struct adc_dev_s));
  dev->ad_ops  = &gpadc_ops;
  dev->ad_priv = &gpadc_priv;
  
  int ret = adc_register("/dev/gpadc", dev);

  return ret;
}
