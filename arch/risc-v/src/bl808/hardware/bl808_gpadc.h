 /****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_gpadc.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GPADC_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GPADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* gpadc base */
#define BL808_GPADC_CONFIG_OFFSET    (0x0)
#define BL808_GPADC_DMA_RDATA_OFFSET (0x4)
#define BL808_GPADC_PIR_TRAIN_OFFSET (0x20

/* aon base */
#define BL808_GPADC_CMD_OFFSET        (0x90C)
#define BL808_GPADC_CONFIG1_OFFSET    (0x910)
#define BL808_GPADC_CONFIG2_OFFSET    (0x914)
#define BL808_GPADC_SCN_POS1_OFFSET   (0x918)
#define BL808_GPADC_SCN_POS2_OFFSET   (0x91C)
#define BL808_GPADC_SCN_NEG1_OFFSET   (0x920)
#define BL808_GPADC_SCN_NEG2_OFFSET   (0x924)
#define BL808_GPADC_STATUS_OFFSET     (0x928)
#define BL808_GPADC_ISR_OFFSET        (0x92C)
#define BL808_GPADC_RESULT_OFFSET     (0x930)
#define BL808_GPADC_RAW_RESULT_OFFSET (0x934)
#define BL808_GPADC_DEFINE_OFFSET     (0x938)

/* Register definitions *****************************************************/

#define BL808_GPADC_CONFIG    (BL808_GPADC_BASE + BL808_GPADC_CONFIG_OFFSET)
#define BL808_GPADC_DMA_RDATA (BL808_GPADC_BASE + BL808_GPADC_DMA_RDATA_OFFSET)
#define BL808_GPADC_PIR_TRAIN (BL808_GPADC_BASE + BL808_GPADC_PIR_TRAIN_OFFSET)

#define BL808_GPADC_CMD (BL808_AON_BASE + BL808_GPADC_CMD_OFFSET)
#define BL808_GPADC_CONFIG1 (BL808_AON_BASE + BL808_GPADC_CONFIG1_OFFSET)
#define BL808_GPADC_CONFIG2 (BL808_AON_BASE + BL808_GPADC_CONFIG2_OFFSET)
#define BL808_GPADC_SCN_POS1 (BL808_AON_BASE + BL808_GPADC_SCN_POS_OFFSET)
#define BL808_GPADC_SCN_P0S2 (BL808_AON_BASE + BL808_GPADC_SCN_PS2_OFFSET)
#define BL808_GPADC_SCN_NEG1 (BL808_AON_BASE + BL808_GPADC_SCN_NEG1_OFFSET)
#define BL808_GPADC_SCN_NEG2 (BL808_AON_BASE + BL808_GPADC_SCN_NEG2_OFFSET)
#define BL808_GPADC_STATUS (BL808_AON_BASE + BL808_GPADC_STATUS_OFFSET)
#define BL808_GPADC_ISR (BL808_AON_BASE + BL808_GPADC_ISR_OFFSET)
#define BL808_GPADC_RESULT (BL808_AON_BASE + BL808_GPADC_RESULT_OFFSET)
#define BL808_GPADC_RAW_RESULT (BL808_AON_BASE + BL808_GPADC_RAW_RESULT_OFFSET)
#define BL808_GPADC_DEFINE (BL808_AON_BASE + BL808_GPADC_DEFINE_OFFSET)

/* Register bit definitions *************************************************/



#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GPADC_H */
