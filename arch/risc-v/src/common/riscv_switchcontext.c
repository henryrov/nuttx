/****************************************************************************
 * arch/risc-v/src/common/riscv_switchcontext.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <syscall.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"
#include "clock/clock.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_switch_context
 *
 * Description:
 *   A task is currently in the ready-to-run list but has been prepped
 *   to execute. Restore its context, and start execution.
 *
 * Input Parameters:
 *   tcb: Refers to the head task of the ready-to-run list
 *     which will be executed.
 *   rtcb: Refers to the running task which will be blocked.
 *
 ****************************************************************************/

void up_switch_context(struct tcb_s *tcb, struct tcb_s *rtcb)
{
  /* Are we in an interrupt handler? */

  if (up_interrupt_context())
    {
      /* Yes, then we have to do things differently.
       * Just copy the current_regs into the OLD rtcb.
       */

      riscv_savecontext(rtcb);

      /* Then switch contexts.  Any necessary address environment
       * changes will be made when the interrupt returns.
       */

      riscv_restorecontext(tcb);
    }

  /* No, then we will need to perform the user context switch */

  else
    {
      /* Then switch contexts */

      riscv_switchcontext();

      /* riscv_switchcontext forces a context switch to the task at the
       * head of the ready-to-run list.  It does not 'return' in the
       * normal sense.  When it does return, it is because the blocked
       * task is again ready to run and has execution priority.
       */
    }
}
