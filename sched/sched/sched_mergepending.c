/****************************************************************************
 * sched/sched/sched_mergepending.c
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

#include <stdbool.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/queue.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_merge_pending
 *
 * Description:
 *   This function merges the prioritized g_pendingtasks list into the
 *   prioritized ready-to-run task list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if the head of the ready-to-run task list has changed indicating
 *     a context switch is needed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function.
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run task list is changed.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_DEADLINE

/****************************************************************************
 * Name: nxsched_merge_runnable
 *
 * Description:
 *   Merge a pending queue into a ready queue using the runnable order.
 *   This is the body of nxsched_merge_pending(). The test calls it with
 *   private queues and update_current false so up_update_task() is not
 *   invoked. Both queues must already be ordered by that same predicate.
 *   An ineligible deadline TCB is not spliced onto the ready queue.
 *
 *   Does not call nxsched_add_readytorun(). That function also applies
 *   the scheduler lock and would re-pending the task being merged.
 *
 ****************************************************************************/

bool nxsched_merge_runnable(FAR dq_queue_t *readylist,
                            FAR dq_queue_t *pendinglist,
                            bool update_current)
{
  FAR struct tcb_s *ptcb;
  FAR struct tcb_s *pnext;
  FAR struct tcb_s *rtcb;
  FAR struct tcb_s *rprev;
  bool ret = false;

  rtcb = (FAR struct tcb_s *)readylist->head;

  for (ptcb = (FAR struct tcb_s *)pendinglist->head; ptcb; ptcb = pnext)
    {
      pnext = ptcb->flink;

      if ((ptcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_DEADLINE &&
          !nxsched_deadline_eligible(ptcb))
        {
          ptcb->flink = NULL;
          ptcb->blink = NULL;
          continue;
        }

      /* Search ahead. Both lists use nxsched_runnable_precedes(), so a
       * later pending task cannot belong before the node just inserted.
       */

      for (;
           rtcb != NULL && !nxsched_runnable_precedes(ptcb, rtcb);
           rtcb = rtcb->flink)
        {
        }

      /* Idle remains on the ready list, so the search must stop. */

      DEBUGASSERT(rtcb);

      rprev = rtcb->blink;
      if (rprev == NULL)
        {
          ptcb->flink = rtcb;
          ptcb->blink = NULL;
          rtcb->blink = ptcb;
          readylist->head = (FAR dq_entry_t *)ptcb;
          rtcb->task_state = TSTATE_TASK_READYTORUN;
          ptcb->task_state = TSTATE_TASK_RUNNING;
          if (update_current)
            {
              up_update_task(ptcb);
            }

          ret = true;
        }
      else
        {
          ptcb->flink = rtcb;
          ptcb->blink = rprev;
          rprev->flink = ptcb;
          rtcb->blink = ptcb;
          ptcb->task_state = TSTATE_TASK_READYTORUN;
        }

      rtcb = ptcb;
    }

  pendinglist->head = NULL;
  pendinglist->tail = NULL;
  return ret;
}

bool nxsched_merge_pending(void)
{
  FAR struct tcb_s *rtcb = this_task();

  /* Do nothing if pre-emption is still disabled. */

  if (nxsched_islocked_tcb(rtcb))
    {
      return false;
    }

  return nxsched_merge_runnable(list_readytorun(), list_pendingtasks(),
                                true);
}

#else /* CONFIG_SCHED_DEADLINE */

bool nxsched_merge_pending(void)
{
  FAR struct tcb_s *ptcb;
  FAR struct tcb_s *pnext;
  FAR struct tcb_s *rtcb;
  FAR struct tcb_s *rprev;
  bool ret = false;

  /* Initialize the inner search loop */

  rtcb = this_task();

  /* Process every TCB in the g_pendingtasks list
   *
   * Do nothing if pre-emption is still disabled
   */

  if (!nxsched_islocked_tcb(rtcb))
    {
      for (ptcb = (FAR struct tcb_s *)list_pendingtasks()->head;
           ptcb;
           ptcb = pnext)
        {
          pnext = ptcb->flink;

          /* REVISIT:  Why don't we just remove the ptcb from pending task
           * list and call nxsched_add_readytorun?
           */

          /* Search the ready-to-run list to find the location to insert the
           * new ptcb. Each is list is maintained in ascending sched_priority
           * order.
           */

          for (;
               (ptcb->sched_priority <= rtcb->sched_priority);
               rtcb = rtcb->flink)
            {
            }

          /* Add the ptcb to the spot found in the list.  Check if the
           * ptcb goes at the ends of the ready-to-run list. This would be
           * error condition since the idle test must always be at the end of
           * the ready-to-run list!
           */

          DEBUGASSERT(rtcb);

          /* The ptcb goes just before rtcb */

          rprev = rtcb->blink;
          if (rprev == NULL)
            {
              /* Special case: Inserting ptcb at the head of the list */

              ptcb->flink       = rtcb;
              ptcb->blink       = NULL;
              rtcb->blink       = ptcb;
              list_readytorun()->head
                                = (FAR dq_entry_t *)ptcb;
              rtcb->task_state  = TSTATE_TASK_READYTORUN;
              ptcb->task_state  = TSTATE_TASK_RUNNING;
              up_update_task(ptcb);
              ret               = true;
            }
          else
            {
              /* Insert in the middle of the list */

              ptcb->flink       = rtcb;
              ptcb->blink       = rprev;
              rprev->flink      = ptcb;
              rtcb->blink       = ptcb;
              ptcb->task_state  = TSTATE_TASK_READYTORUN;
            }

          /* Set up for the next time through */

          rtcb = ptcb;
        }

      /* Mark the input list empty */

      list_pendingtasks()->head = NULL;
      list_pendingtasks()->tail = NULL;
    }

  return ret;
}

#endif /* CONFIG_SCHED_DEADLINE */
