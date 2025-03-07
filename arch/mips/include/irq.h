/****************************************************************************
 * arch/mips/include/irq.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_MIPS_INCLUDE_IRQ_H
#define __ARCH_MIPS_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/* Include AVR architecture-specific IRQ definitions (including register
 * save structure and up_irq_save()/up_irq_restore() macros
 */

#ifdef CONFIG_ARCH_MIPS32
#  include <arch/mips32/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Return the current value of the stack pointer */

static inline_function uint32_t up_getsp(void)
{
  register uint32_t sp;
  __asm__
  (
    "\tadd  %0, $0, $29\n"
    : "=r"(sp)
  );
  return sp;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs holds a references to the current interrupt level
 * register storage structure.  It is non-NULL only during interrupt
 * processing.  Access to g_current_regs must be through the
 * [get/set]_current_regs for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

EXTERN volatile uint32_t *g_current_regs;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

static inline_function uint32_t *up_current_regs(void)
{
  return (uint32_t *)g_current_regs;
}

static inline_function void up_set_current_regs(uint32_t *regs)
{
  g_current_regs = regs;
}

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in the interrupt
 *   handler context.
 *
 ****************************************************************************/

#define up_interrupt_context() (up_current_regs() != NULL)

/****************************************************************************
 * Name: up_getusrpc
 ****************************************************************************/

#define up_getusrpc(regs) \
    (((uint32_t *)((regs) ? (regs) : up_current_regs()))[REG_EPC])

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

#define up_getusrsp(regs) \
    ((uintptr_t)((uint32_t*)(regs))[REG_SP])

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_IRQ_H */
