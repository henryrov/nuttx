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

#define BL808_TOTAL_NCHANNELS 18
#define BL808_SCAN_MAX_CHANNELS 12

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl808_gpadc_s
{
  const struct adc_callback_s *callback;
  bool channel_enable[BL808_TOTAL_NCHANNELS];
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

static struct gpadc_priv =
{
  .callback = NULL;
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
  }
}
  
static struct adc_ops_s gpadc_ops =
{
  .ao_bind = bl808_gpadc_bind,
  .ao_reset = bl808_gpadc_reset,
  .ao_setup = bl808_gpadc_setup,
  .ao_shutdown = bl808_gpadc_shutdown,
  .ao_rxint = bl808_gpadc_rxint,
  .ao_ioctl = bl808_gpadc_ioctl
}

static int bl808_gpadc_bind(struct adc_dev_s *dev,
                            const struct adc_callback_s *callback)
{
  dev->priv->callback = callback;

  return OK;
}

static void bl808_gpadc_reset(struct adc_dev_s *dev)
{
  /* Nothing for now */
}

static int bl808_gpadc_setup(struct adc_dev_s *dev)
{
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
  gpadc = kmm_zalloc(sizeof(struct adc_dev_s));
  dev->ad_ops  = &gpadc_ops;
  dev->ad_priv = gpadc_priv;
  
  ret = adc_register("/dev/gpadc", gpadc_dev);
}
