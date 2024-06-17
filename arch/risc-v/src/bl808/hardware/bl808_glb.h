/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_glb.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "bl808_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BL808_GPIO_CFG_OFFSET              0x0008c4  /* gpio_cfg0 */

/* Register definitions *****************************************************/

#define BL808_GPIO_CFG(n)        (BL808_GLB_BASE + BL808_GPIO_CFG_OFFSET + 4*n)

/* Register bit definitions *************************************************/

// bit definitions from lupyuen's wip-nuttx, branch gpio2

//// Check every bit
#define GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT           (8)
#define GPIO_CFGCTL0_GPIO_0_FUNC_SEL_MASK            (0x0f << GPIO_CFGCTL0_GPIO_0_FUNC_SEL_SHIFT)
#define GPIO_CFGCTL0_GPIO_0_OE                       (1 << 6)
#define GPIO_CFGCTL0_GPIO_0_PD                       (1 << 5)
#define GPIO_CFGCTL0_GPIO_0_PU                       (1 << 4)
#define GPIO_CFGCTL0_GPIO_0_DRV_SHIFT                (2)
#define GPIO_CFGCTL0_GPIO_0_DRV_MASK                 (0x03 << GPIO_CFGCTL0_GPIO_0_DRV_SHIFT)
#define GPIO_CFGCTL0_GPIO_0_SMT                      (1 << 1)
#define GPIO_CFGCTL0_GPIO_0_IE                       (1 << 0)
////

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_GLB_H */
