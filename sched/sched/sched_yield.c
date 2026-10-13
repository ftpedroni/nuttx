/****************************************************************************
 * sched/sched/sched_yield.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_yield
 *
 * Description:
 *   This function forces the calling task to give up the CPU (only to other
 *   tasks at the same priority).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 (OK) or -1 (ERROR) (errno is not set)
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_yield(void)
{
  FAR struct tcb_s *rtcb = this_task();
  int ret;

#ifdef CONFIG_SCHED_DEADLINE
  /* Same-priority rotation does not surrender an earliest deadline.
   * Give the rest of this job back and throttle until replenishment.
   * sched_lock() keeps the existing deferred-throttle path.
   */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_DEADLINE)
    {
      irqstate_t flags = enter_critical_section();

      if (nxsched_deadline_yield(rtcb))
        {
          up_switch_context(this_task(), rtcb);
        }

      leave_critical_section(flags);
      return OK;
    }
#endif

  /* This equivalent to just resetting the task priority to its current value
   * since this will cause the task to be rescheduled behind any other tasks
   * at the same priority.
   */

  ret = nxsched_set_priority(rtcb, rtcb->sched_priority);
  return ret < 0 ? ERROR : OK;
}
