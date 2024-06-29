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

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bl808_gpadc_s
{
  const struct adc_callback_s *callback;
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
