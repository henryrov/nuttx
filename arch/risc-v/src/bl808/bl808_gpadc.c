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
#include <stdio.h>
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
  GPADC_CH_TSEN,
  GPADC_CH_VREF = 16,
  GPADC_CH_HALF_VBAT = 18,
  GPADC_CH_GND = 23
};

struct bl808_gpadc_s
{
  const struct adc_callback_s *callback;
  enum bl808_gpadc_channel_e enabled_channels[BL808_GPADC_SCAN_MAX_CHANNELS];
  uint8_t nchannels;
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

  .enabled_channels =
  {
    GPADC_CH_VREF,
    GPADC_CH_TSEN,
    GPADC_CH_HALF_VBAT,
    GPADC_CH_GND,
    GPADC_CH4,
    GPADC_CH5,
    GPADC_CH6,
    GPADC_CH7,
    GPADC_CH8,
    GPADC_CH9,
    GPADC_CH10,
    GPADC_CH11,
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

static int __gpadc_interrupt(int irq, void *context, void *arg)
{
  struct adc_dev_s *dev = (struct adc_dev_s *)arg;
  struct bl808_gpadc_s *priv = dev->ad_priv;
  uint32_t status = getreg32(BL808_GPADC_CONFIG);

  if (status & GPADC_RDY)
    {
      uint8_t count = (status & GPADC_FIFO_DATA_COUNT_MASK)
	>> GPADC_FIFO_DATA_COUNT_SHIFT;
      up_putc('0' + count);

      while (count != 0)
	{
	  uint32_t result = getreg32(BL808_GPADC_DMA_RDATA);
	  uint32_t channel = (result & (0x1f << 21)) >> 21;
	  uint32_t adc_val = result & 0xffff;

	  int ret = priv->callback->au_receive(dev, channel, adc_val);
	  if (ret)
	    {
	      up_putc('z');
	    }

	  status = getreg32(BL808_GPADC_CONFIG);
	  count = (status & GPADC_FIFO_DATA_COUNT_MASK)
	    >> GPADC_FIFO_DATA_COUNT_SHIFT;
	  up_putc('0' + count);
	}

      modifyreg32(BL808_GPADC_CONFIG, 0,
	      GPADC_FIFO_CLR);
      
      modifyreg32(BL808_GPADC_CONFIG, 0,
		  GPADC_RDY_CLR);
    }
  else
    {
      up_putc('x');
    }
  return OK;
}

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
  up_putc('s');
  struct bl808_gpadc_s *priv = dev->ad_priv;
  
  /* The setup process below is mostly taken from bouffalo_sdk */
  
  /* Disable and reenable ADC */
  
  modifyreg32(BL808_GPADC_CMD, GPADC_GLOBAL_EN, 0);

  /* Soft reset */
  
  modifyreg32(BL808_GPADC_CMD, 0, GPADC_SOFT_RST);

  /*
  for (int i = 0; i < 10; i++)
    {
      asm("nop");
    }
  */
  
  
  modifyreg32(BL808_GPADC_CMD, GPADC_SOFT_RST, 0);

  modifyreg32(BL808_GPADC_CONFIG1, GPADC_CONT_CONV_EN,
              (2 << GPADC_V18_SEL_SHIFT)
              | (1 << GPADC_V11_SEL_SHIFT)
              | GPADC_SCAN_EN
              | GPADC_CLK_ANA_INV);

  modifyreg32(BL808_GPADC_CONFIG2, 0,
	      (2 << GPADC_DLY_SEL_SHIFT)
	      | GPADC_VBAT_EN
	      | GPADC_TS_EN);

  /* Use GND as negative channel for now */
  
  modifyreg32(BL808_GPADC_CMD, 0, GPADC_NEG_GND);

  modifyreg32(BL808_GPADC_CONFIG, 0,
	      GPADC_RDY_CLR
	      | GPADC_FIFO_OVERRUN_CLR
	      | GPADC_FIFO_UNDERRUN_CLR);

  modifyreg32(BL808_GPADC_CONFIG,
	      GPADC_RDY_CLR
	      | GPADC_FIFO_OVERRUN_CLR
	      | GPADC_FIFO_UNDERRUN_CLR, 0);

  /* Set scan channels */
  
  modifyreg32(BL808_GPADC_SCAN_POS1,
	      0xffffffff, 0);
  modifyreg32(BL808_GPADC_SCAN_POS2,
	      0xffffffff, 0);
  
  for (int channel_idx = 0;
       channel_idx < priv->nchannels;
       channel_idx++)
    {
      
      if (channel_idx < 6)
	{
	  modifyreg32(BL808_GPADC_SCAN_POS1, 0,
		      (priv->enabled_channels[channel_idx]
		       << GPADC_SCAN_POS_SHIFT(channel_idx)));
	}
      else
	{
	  modifyreg32(BL808_GPADC_SCAN_POS2, 0,
		      (priv->enabled_channels[channel_idx]
		       << GPADC_SCAN_POS_SHIFT(channel_idx)));
	}
    }
  
  modifyreg32(BL808_GPADC_CONFIG1, 0,
	      ((priv->nchannels - 1) << GPADC_SCAN_LENGTH_SHIFT));

  modifyreg32(BL808_GPADC_CONFIG, 0,
	      GPADC_FIFO_CLR);

  modifyreg32(BL808_GPADC_CMD, 0, GPADC_GLOBAL_EN);

  return OK;
}

static void bl808_gpadc_shutdown(struct adc_dev_s *dev)
{
}

static void bl808_gpadc_rxint(struct adc_dev_s *dev,
                              bool enable)
{
  if (enable)
    {
      irq_attach(BL808_IRQ_GPADC, __gpadc_interrupt, (void *)dev);
      up_enable_irq(BL808_IRQ_GPADC);

      /* Clear and then set bit to start conversion */

      modifyreg32(BL808_GPADC_CMD, 0, GPADC_CONV_START);

      up_putc('r');

      /*
      for (;;) {
	uint32_t status = getreg32(BL808_GPADC_CONFIG);
	uint32_t count = (status & GPADC_FIFO_DATA_COUNT_MASK) >> GPADC_FIFO_DATA_COUNT_SHIFT;
	
	if (count) {
	  printf("%d\n", count);
	}
      }
      */
    }
  else
    {
      up_disable_irq(BL808_IRQ_GPADC);
      irq_detach(BL808_IRQ_GPADC);

      /* Stop conversion */

      modifyreg32(BL808_GPADC_CMD, GPADC_CONV_START, 0);
    }
}

static int bl808_gpadc_ioctl(struct adc_dev_s *dev,
                             int cmd, unsigned long arg)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bl808_gpadc_init(void)
{
  up_putc('a');
  struct adc_dev_s *dev = kmm_zalloc(sizeof(struct adc_dev_s));
  dev->ad_ops  = &gpadc_ops;
  dev->ad_priv = &gpadc_priv;
  
  int ret = adc_register("/dev/gpadc", dev);

  return ret;
}
