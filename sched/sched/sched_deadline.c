/****************************************************************************
 * sched/sched/sched_deadline.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/sched.h>
#include <nuttx/irq.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* v0 is one exact size. A different size is rejected before any field
 * past sched_attr.size is read, and before any byte of a getattr buffer
 * is written.
 */

#define DEADLINE_ATTR_V0 ((uint32_t)sizeof(struct sched_attr))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: deadline_known_legacy
 *
 * Description:
 *   True for policies that sched_setscheduler() can already apply. Those
 *   requests are refused here rather than applied, so this file does not
 *   call nxsched_set_scheduler().
 *
 ****************************************************************************/

static bool deadline_known_legacy(uint32_t policy)
{
  switch (policy)
    {
      case SCHED_OTHER:
      case SCHED_FIFO:
        return true;

#if CONFIG_RR_INTERVAL > 0
      case SCHED_RR:
        return true;
#endif

#ifdef CONFIG_SCHED_SPORADIC
      case SCHED_SPORADIC:
        return true;
#endif

      default:
        return false;
    }
}

/****************************************************************************
 * Name: nxsched_deadline_validate
 *
 * Description:
 *   Structural check for a SCHED_DEADLINE v0 request. The caller has
 *   already accepted the pointer, the function flags, and the v0 size.
 *   Comparisons are unsigned. Nothing is multiplied, so values near
 *   UINT64_MAX cannot wrap. This function does not touch a TCB, a timer,
 *   a queue, or any admission total. OK means the request is well formed,
 *   not that a reservation exists.
 *
 ****************************************************************************/

static int nxsched_deadline_validate(FAR const struct sched_attr *attr)
{
  if (attr->sched_flags != 0 || attr->sched_nice != 0 ||
      attr->sched_priority != 0 ||
      attr->sched_policy != SCHED_DEADLINE)
    {
      return -EINVAL;
    }

  if (attr->sched_runtime == 0 || attr->sched_deadline == 0 ||
      attr->sched_period == 0)
    {
      return -EINVAL;
    }

  /* Implicit deadlines only: 0 < runtime <= deadline == period.
   * A constrained deadline is not a supported CBS contract.
   */

  if (attr->sched_runtime > attr->sched_deadline ||
      attr->sched_deadline != attr->sched_period)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: deadline_lookup
 *
 * Description:
 *   Find the target task. pid 0 is the caller. The pointer may name a
 *   live reservation; callers must not assume it is NULL.
 *
 ****************************************************************************/

static FAR struct tcb_s *deadline_lookup(pid_t pid)
{
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  flags = enter_critical_section();

  if (pid == 0)
    {
      tcb = this_task();
    }
  else
    {
      tcb = nxsched_get_tcb(pid);
    }

  leave_critical_section(flags);
  return tcb;
}

static bool deadline_is_queued_runnable(FAR struct tcb_s *tcb)
{
  return tcb->task_state == TSTATE_TASK_RUNNING ||
         tcb->task_state == TSTATE_TASK_READYTORUN ||
         tcb->task_state == TSTATE_TASK_PENDING;
}

/****************************************************************************
 * Name: deadline_activate
 *
 * Description:
 *   Publish a reservation that reserve() has already committed. On any
 *   failure before that commit, the previous policy and ledger are kept.
 *   A blocked task stays blocked so its next wake applies CBS once.
 *
 ****************************************************************************/

static int deadline_activate(pid_t pid, FAR const struct sched_attr *attr)
{
  FAR struct tcb_s *tcb;
  FAR struct tcb_s *prev;
  FAR struct deadline_s *dl;
  irqstate_t flags;
  bool already;
  int ret;

  tcb = deadline_lookup(pid);
  if (tcb == NULL)
    {
      return -ESRCH;
    }

  already = (tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_DEADLINE;
  ret = nxsched_deadline_reserve(tcb, attr->sched_runtime,
                                 attr->sched_deadline, attr->sched_period);
  if (ret < 0)
    {
      return ret;
    }

  flags = enter_critical_section();
  tcb = deadline_lookup(pid);
  if (tcb == NULL || tcb->deadline == NULL ||
      (tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      leave_critical_section(flags);
      return -ESRCH;
    }

  dl = tcb->deadline;
  dl->flags |= DEADLINE_FLAG_SCHEDULABLE;
  tcb->flags &= ~TCB_FLAG_POLICY_MASK;
  tcb->flags |= TCB_FLAG_SCHED_DEADLINE;

  /* An update keeps the current job. Only the first publish of a
   * runnable task opens a job here. A task in a user wait opens its
   * job in nxsched_deadline_woken().
   */

  if (!already && deadline_is_queued_runnable(tcb))
    {
      prev = this_task();
      nxsched_deadline_arm_job(tcb);
      nxsched_remove_readytorun(tcb);
      nxsched_add_readytorun(tcb);
      if (tcb->task_state == TSTATE_TASK_RUNNING &&
          (dl->flags & DEADLINE_FLAG_EXECUTING) == 0)
        {
          /* Stayed at the head, so no context switch will stamp it. */

          dl->flags |= DEADLINE_FLAG_EXECUTING;
          dl->accounted_ns = clock_systime_nsec();
          nxsched_deadline_arm_budget(tcb);
        }

      if (this_task() != prev)
        {
          up_switch_context(this_task(), prev);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_setattr
 *
 * Description:
 *   Validate struct sched_attr. A well-formed SCHED_DEADLINE request
 *   reserves bandwidth and publishes the policy, or leaves the task
 *   unchanged when admission fails. Existing policies are not applied
 *   through this entry point; callers keep using sched_setscheduler()
 *   and sched_setparam().
 *
 * Returned Value:
 *   OK, or a negated errno:
 *     EINVAL      Bad pointer, flags, size, or parameters.
 *     ESRCH       No task has this pid.
 *     ENOMEM      The reservation object could not be allocated.
 *     EBUSY       The admission cap does not fit this request.
 *     EOPNOTSUPP  A known legacy policy. The task is unchanged.
 *
 ****************************************************************************/

static int nxsched_setattr(pid_t pid, FAR const struct sched_attr *attr,
                           unsigned int flags)
{
  uint32_t attr_size;
  int ret;

  if (attr == NULL || flags != 0)
    {
      return -EINVAL;
    }

  /* Do not read past the size word until it matches v0. */

  attr_size = attr->size;
  if (attr_size != DEADLINE_ATTR_V0)
    {
      return -EINVAL;
    }

  if (deadline_lookup(pid) == NULL)
    {
      return -ESRCH;
    }

  if (attr->sched_policy == SCHED_DEADLINE)
    {
      ret = nxsched_deadline_validate(attr);
      if (ret < 0)
        {
          return ret;
        }

      return deadline_activate(pid, attr);
    }

  if (deadline_known_legacy(attr->sched_policy))
    {
      return -EOPNOTSUPP;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: sched_setattr
 *
 * Description:
 *   Public wrapper. See nxsched_setattr().
 *
 ****************************************************************************/

int sched_setattr(pid_t pid, FAR const struct sched_attr *attr,
                  unsigned int flags)
{
  int ret = nxsched_setattr(pid, attr, flags);

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: nxsched_getattr
 *
 * Description:
 *   Describe the current policy and ordinary priority. A deadline task
 *   reports its static runtime, deadline, and period. Remaining runtime
 *   and the absolute scheduling deadline are not copied.
 *
 * Returned Value:
 *   OK, or a negated errno. The buffer is not written on error.
 *     EINVAL  Bad pointer, flags, or size.
 *     ESRCH   No task has this pid.
 *
 ****************************************************************************/

static int nxsched_getattr(pid_t pid, FAR struct sched_attr *attr,
                           unsigned int size, unsigned int flags)
{
  FAR struct tcb_s *tcb;
  irqstate_t irqflags;
  int policy;

  if (attr == NULL || flags != 0 || size != DEADLINE_ATTR_V0)
    {
      return -EINVAL;
    }

  irqflags = enter_critical_section();

  if (pid == 0)
    {
      tcb = this_task();
    }
  else
    {
      tcb = nxsched_get_tcb(pid);
    }

  if (tcb == NULL)
    {
      leave_critical_section(irqflags);
      return -ESRCH;
    }

  policy = nxsched_get_scheduler(pid);

  if (policy < 0)
    {
      leave_critical_section(irqflags);
      return policy;
    }

  memset(attr, 0, sizeof(*attr));
  attr->size           = DEADLINE_ATTR_V0;
  attr->sched_policy   = (uint32_t)policy;
  attr->sched_priority = tcb->sched_priority;
  if (policy == SCHED_DEADLINE && tcb->deadline != NULL)
    {
      attr->sched_runtime  = tcb->deadline->runtime_ns;
      attr->sched_deadline = tcb->deadline->deadline_ns;
      attr->sched_period   = tcb->deadline->period_ns;
      attr->sched_priority = 0;
    }

  leave_critical_section(irqflags);
  return OK;
}

/****************************************************************************
 * Name: sched_getattr
 *
 * Description:
 *   Public wrapper. See nxsched_getattr().
 *
 ****************************************************************************/

int sched_getattr(pid_t pid, FAR struct sched_attr *attr,
                  unsigned int size, unsigned int flags)
{
  int ret = nxsched_getattr(pid, attr, size, flags);

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

#ifdef CONFIG_TESTING_DEADLINE

int sched_deadline_view(pid_t pid, FAR struct sched_deadline_view_s *view)
{
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  if (view == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  tcb = deadline_lookup(pid);
  if (tcb == NULL || tcb->deadline == NULL)
    {
      leave_critical_section(flags);
      return -ESRCH;
    }

  view->runtime_ns   = tcb->deadline->runtime_ns;
  view->deadline_ns  = tcb->deadline->deadline_ns;
  view->period_ns    = tcb->deadline->period_ns;
  view->remaining_ns = tcb->deadline->remaining_ns;
  view->abs_deadline = tcb->deadline->abs_deadline;
  view->overrun_ns   = tcb->deadline->overrun_ns;
  view->runnable_seq = tcb->deadline->runnable_seq;
  view->admitted_ppm = nxsched_deadline_admitted_ppm();
  view->sched        = tcb->deadline->sched;
  view->flags        = tcb->deadline->flags;
  leave_critical_section(flags);
  return OK;
}

uint32_t sched_deadline_admitted(void)
{
  return nxsched_deadline_admitted_ppm();
}

/****************************************************************************
 * Deadline ordering fixtures
 *
 * These TCBs are never placed on g_readytorun or g_pendingtasks.
 * sched_deadline_order_test() calls the production comparator,
 * nxsched_add_runnable(), and nxsched_merge_runnable(). It does not
 * call nxsched_add_readytorun(): that would run up_update_task() on a
 * synthetic head. Case 13 (option disabled) is not this image.
 *
 ****************************************************************************/

static struct tcb_s g_fix[8];
static struct deadline_s g_dl[4];

static void order_reset(void)
{
  memset(g_fix, 0, sizeof(g_fix));
  memset(g_dl, 0, sizeof(g_dl));
}

static FAR struct tcb_s *order_tcb(int slot, pid_t pid, uint8_t policy,
                                   uint8_t prio)
{
  FAR struct tcb_s *tcb = &g_fix[slot];

  tcb->pid = pid;
  tcb->flags = policy;
  tcb->sched_priority = prio;
  return tcb;
}

static void order_key(FAR struct tcb_s *tcb, int slot, uint64_t absdl,
                      uint64_t seq, bool valid)
{
  g_dl[slot].abs_deadline = absdl;
  g_dl[slot].runnable_seq = seq;
  g_dl[slot].flags = valid ? DEADLINE_FLAG_KEY_VALID : 0;
  g_dl[slot].sched = valid ? DL_SCHED_ELIGIBLE : DL_SCHED_INACTIVE;
  tcb->deadline = &g_dl[slot];
  tcb->flags = (tcb->flags & ~TCB_FLAG_POLICY_MASK) | TCB_FLAG_SCHED_DEADLINE;
}

static void order_anchor_idle(FAR dq_queue_t *list, FAR struct tcb_s *idle)
{
  idle->pid = 0;
  idle->flags = TCB_FLAG_SCHED_FIFO;
  idle->sched_priority = 0;
  idle->task_state = TSTATE_TASK_READYTORUN;
  idle->flink = NULL;
  idle->blink = (FAR struct tcb_s *)list->tail;
  if (list->tail == NULL)
    {
      list->head = (FAR dq_entry_t *)idle;
    }
  else
    {
      ((FAR struct tcb_s *)list->tail)->flink = idle;
    }

  list->tail = (FAR dq_entry_t *)idle;
}

static int order_fail(int id)
{
  printf("deadline order: FAIL case %d\n", id);
  return id;
}

static int order_check(FAR dq_queue_t *list, FAR const pid_t *expect,
                       int n, int id)
{
  FAR struct tcb_s *tcb;
  FAR struct tcb_s *prev = NULL;
  int i;

  tcb = (FAR struct tcb_s *)list->head;
  for (i = 0; i < n; i++)
    {
      if (tcb == NULL || tcb->pid != expect[i] || tcb->blink != prev)
        {
          printf("deadline order: FAIL case %d at %d\n", id, i);
          return -1;
        }

      prev = tcb;
      tcb = tcb->flink;
    }

  if (tcb != NULL ||
      (n == 0 && list->tail != NULL) ||
      (n > 0 && list->tail != (FAR dq_entry_t *)prev))
    {
      printf("deadline order: FAIL case %d tail\n", id);
      return -1;
    }

  return 0;
}

int sched_deadline_order_test(void)
{
  dq_queue_t ready;
  dq_queue_t pending;
  FAR struct tcb_s *a;
  FAR struct tcb_s *b;
  FAR struct tcb_s *c;
  FAR struct tcb_s *d;
  FAR struct tcb_s *idle;
  pid_t expect[6];
  pid_t self;
  uint64_t seq;

  /* 1. Higher FIFO priority precedes. */

  order_reset();
  dq_init(&ready);
  a = order_tcb(0, 10, TCB_FLAG_SCHED_FIFO, 100);
  b = order_tcb(1, 11, TCB_FLAG_SCHED_FIFO, 200);
  if (nxsched_runnable_precedes(a, b) || !nxsched_runnable_precedes(b, a))
    {
      return order_fail(1);
    }

  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  expect[0] = 11;
  expect[1] = 10;
  if (order_check(&ready, expect, 2, 1) != 0)
    {
      return order_fail(1);
    }

  printf("deadline order: ok case 1\n");

  /* 2. Equal FIFO priority keeps insertion order. */

  order_reset();
  dq_init(&ready);
  a = order_tcb(0, 10, TCB_FLAG_SCHED_FIFO, 100);
  b = order_tcb(1, 11, TCB_FLAG_SCHED_FIFO, 100);
  if (nxsched_runnable_precedes(a, b) || nxsched_runnable_precedes(b, a))
    {
      return order_fail(2);
    }

  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  expect[0] = 10;
  expect[1] = 11;
  if (order_check(&ready, expect, 2, 2) != 0)
    {
      return order_fail(2);
    }

  printf("deadline order: ok case 2\n");

  /* 3. RR and sporadic still follow numeric priority and FIFO ties. */

  order_reset();
  dq_init(&ready);
  a = order_tcb(0, 10, TCB_FLAG_SCHED_FIFO, 100);
  b = order_tcb(1, 11, TCB_FLAG_SCHED_RR, 150);
  c = order_tcb(2, 12, TCB_FLAG_SCHED_SPORADIC, 200);
  d = order_tcb(3, 13, TCB_FLAG_SCHED_SPORADIC, 50);
  idle = order_tcb(4, 14, TCB_FLAG_SCHED_FIFO, 100);
  if (!nxsched_runnable_precedes(b, a) ||
      !nxsched_runnable_precedes(c, b) ||
      nxsched_runnable_precedes(d, a) ||
      nxsched_runnable_precedes(idle, a))
    {
      return order_fail(3);
    }

  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  nxsched_add_runnable(c, &ready);
  nxsched_add_runnable(d, &ready);
  nxsched_add_runnable(idle, &ready);
  expect[0] = 12;
  expect[1] = 11;
  expect[2] = 10;
  expect[3] = 14;
  expect[4] = 13;
  if (order_check(&ready, expect, 5, 3) != 0)
    {
      return order_fail(3);
    }

  printf("deadline order: ok case 3\n");

  /* 4. An eligible deadline task precedes FIFO, RR, and sporadic at 255. */

  order_reset();
  dq_init(&ready);
  a = order_tcb(0, 10, TCB_FLAG_SCHED_FIFO, 255);
  b = order_tcb(1, 11, TCB_FLAG_SCHED_RR, 255);
  c = order_tcb(2, 12, TCB_FLAG_SCHED_SPORADIC, 255);
  d = order_tcb(3, 20, TCB_FLAG_SCHED_FIFO, 0);
  order_key(d, 0, 1000, 1, true);
  if (!nxsched_runnable_precedes(d, a) ||
      !nxsched_runnable_precedes(d, b) ||
      !nxsched_runnable_precedes(d, c))
    {
      return order_fail(4);
    }

  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  nxsched_add_runnable(c, &ready);
  nxsched_add_runnable(d, &ready);
  expect[0] = 20;
  expect[1] = 10;
  expect[2] = 11;
  expect[3] = 12;
  if (order_check(&ready, expect, 4, 4) != 0)
    {
      return order_fail(4);
    }

  printf("deadline order: ok case 4\n");

  /* 5. Smaller absolute deadline wins. Zero and the top of uint64_t too. */

  order_reset();
  dq_init(&ready);
  a = order_tcb(0, 30, TCB_FLAG_SCHED_FIFO, 1);
  b = order_tcb(1, 10, TCB_FLAG_SCHED_FIFO, 255);
  order_key(a, 0, 100, 5, true);
  order_key(b, 1, 200, 1, true);
  if (!nxsched_runnable_precedes(a, b) || nxsched_runnable_precedes(b, a))
    {
      return order_fail(5);
    }

  nxsched_add_runnable(b, &ready);
  nxsched_add_runnable(a, &ready);
  expect[0] = 30;
  expect[1] = 10;
  if (order_check(&ready, expect, 2, 5) != 0)
    {
      return order_fail(5);
    }

  order_key(a, 0, 0, 1, true);
  order_key(b, 1, UINT64_MAX, 1, true);
  if (!nxsched_runnable_precedes(a, b))
    {
      return order_fail(5);
    }

  order_key(a, 0, UINT64_MAX - 1, 1, true);
  order_key(b, 1, UINT64_MAX, 1, true);
  if (!nxsched_runnable_precedes(a, b) || nxsched_runnable_precedes(b, a))
    {
      return order_fail(5);
    }

  printf("deadline order: ok case 5\n");

  /* 6. Equal absolute deadlines: smaller admission sequence, then FIFO. */

  order_reset();
  dq_init(&ready);
  a = order_tcb(0, 10, TCB_FLAG_SCHED_FIFO, 255);
  b = order_tcb(1, 11, TCB_FLAG_SCHED_FIFO, 1);
  order_key(a, 0, 1000, 2, true);
  order_key(b, 1, 1000, 1, true);
  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  expect[0] = 11;
  expect[1] = 10;
  if (order_check(&ready, expect, 2, 6) != 0)
    {
      return order_fail(6);
    }

  order_reset();
  dq_init(&ready);
  a = order_tcb(0, 10, TCB_FLAG_SCHED_FIFO, 255);
  b = order_tcb(1, 11, TCB_FLAG_SCHED_FIFO, 1);
  order_key(a, 0, 1000, 2, true);
  order_key(b, 1, 1000, 1, true);
  nxsched_add_runnable(b, &ready);
  nxsched_add_runnable(a, &ready);
  if (order_check(&ready, expect, 2, 6) != 0)
    {
      return order_fail(6);
    }

  order_key(a, 0, 0, 9, true);
  order_key(b, 1, 0, 8, true);
  if (!nxsched_runnable_precedes(b, a))
    {
      return order_fail(6);
    }

  order_key(a, 0, UINT64_MAX, UINT64_MAX, true);
  order_key(b, 1, UINT64_MAX, 0, true);
  if (!nxsched_runnable_precedes(b, a) || nxsched_runnable_precedes(a, b))
    {
      return order_fail(6);
    }

  order_reset();
  dq_init(&ready);
  a = order_tcb(0, 10, TCB_FLAG_SCHED_FIFO, 10);
  b = order_tcb(1, 11, TCB_FLAG_SCHED_FIFO, 10);
  order_key(a, 0, 50, 4, true);
  order_key(b, 1, 50, 4, true);
  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  expect[0] = 10;
  expect[1] = 11;
  if (order_check(&ready, expect, 2, 6) != 0)
    {
      return order_fail(6);
    }

  printf("deadline order: ok case 6\n");

  /* 7. Missing or invalid key: not eligible, not inserted, not a winner. */

  order_reset();
  dq_init(&ready);
  dq_init(&pending);
  a = order_tcb(0, 10, TCB_FLAG_SCHED_FIFO, 100);
  nxsched_add_runnable(a, &ready);
  b = order_tcb(1, 11, TCB_FLAG_SCHED_DEADLINE, 255);
  if (nxsched_deadline_eligible(b) || nxsched_runnable_precedes(b, a) ||
      nxsched_add_runnable(b, &ready))
    {
      return order_fail(7);
    }

  expect[0] = 10;
  if (order_check(&ready, expect, 1, 7) != 0 ||
      b->flink != NULL || b->blink != NULL)
    {
      return order_fail(7);
    }

  order_key(b, 0, 1, 1, false);
  if (nxsched_deadline_eligible(b) || nxsched_add_runnable(b, &ready) ||
      nxsched_runnable_precedes(b, a))
    {
      return order_fail(7);
    }

  if (order_check(&ready, expect, 1, 7) != 0)
    {
      return order_fail(7);
    }

  c = order_tcb(2, 12, TCB_FLAG_SCHED_DEADLINE, 1);
  if (nxsched_set_priority(c, 50) != -EINVAL || c->sched_priority != 1)
    {
      return order_fail(7);
    }

  order_anchor_idle(&ready, order_tcb(3, 0, TCB_FLAG_SCHED_FIFO, 0));
  b->flink = NULL;
  b->blink = NULL;
  pending.head = (FAR dq_entry_t *)b;
  pending.tail = (FAR dq_entry_t *)b;
  if (nxsched_merge_runnable(&ready, &pending, false) ||
      pending.head != NULL || b->flink != NULL)
    {
      return order_fail(7);
    }

  expect[0] = 10;
  expect[1] = 0;
  if (order_check(&ready, expect, 2, 7) != 0)
    {
      return order_fail(7);
    }

  printf("deadline order: ok case 7\n");

  /* 8. A locked deadline task waits on the pending queue, then wins.
   * The absolute deadline does not make it lose to a normal task.
   * A later deadline loses only to an earlier deadline. An equal
   * priority FIFO task does not precede the runner.
   */

  order_reset();
  dq_init(&ready);
  dq_init(&pending);
  idle = order_tcb(0, 0, TCB_FLAG_SCHED_FIFO, 0);
  a = order_tcb(1, 10, TCB_FLAG_SCHED_FIFO, 100);
  b = order_tcb(2, 20, TCB_FLAG_SCHED_FIFO, 0);
  order_anchor_idle(&ready, idle);
  nxsched_add_runnable(a, &ready);
  a->task_state = TSTATE_TASK_RUNNING;
  order_key(b, 0, 1000, 7, true);
  seq = b->deadline->runnable_seq;
  if (!nxsched_runnable_precedes(b, a))
    {
      return order_fail(8);
    }

  nxsched_add_runnable(b, &pending);
  b->task_state = TSTATE_TASK_PENDING;
  expect[0] = 10;
  expect[1] = 0;
  if (order_check(&ready, expect, 2, 8) != 0 ||
      b->task_state != TSTATE_TASK_PENDING)
    {
      return order_fail(8);
    }

  if (!nxsched_merge_runnable(&ready, &pending, false) ||
      b->deadline->runnable_seq != seq || pending.head != NULL)
    {
      return order_fail(8);
    }

  expect[0] = 20;
  expect[1] = 10;
  expect[2] = 0;
  if (order_check(&ready, expect, 3, 8) != 0 ||
      b->task_state != TSTATE_TASK_RUNNING ||
      a->task_state != TSTATE_TASK_READYTORUN)
    {
      return order_fail(8);
    }

  order_reset();
  dq_init(&ready);
  idle = order_tcb(0, 0, TCB_FLAG_SCHED_FIFO, 0);
  a = order_tcb(1, 10, TCB_FLAG_SCHED_FIFO, 100);
  b = order_tcb(2, 21, TCB_FLAG_SCHED_FIFO, 0);
  c = order_tcb(3, 22, TCB_FLAG_SCHED_FIFO, 100);
  d = order_tcb(4, 23, TCB_FLAG_SCHED_FIFO, 0);
  order_anchor_idle(&ready, idle);
  nxsched_add_runnable(a, &ready);
  order_key(b, 0, 1000, 1, true);
  order_key(d, 1, UINT64_MAX, 2, true);
  if (nxsched_runnable_precedes(c, a) || nxsched_runnable_precedes(d, b) ||
      !nxsched_runnable_precedes(b, a) || !nxsched_runnable_precedes(d, a))
    {
      return order_fail(8);
    }

  /* d is later than b, so it must not become the head. It still
   * precedes the normal task and idle.
   */

  nxsched_add_runnable(b, &ready);
  if (nxsched_add_runnable(d, &ready))
    {
      return order_fail(8);
    }

  expect[0] = 21;
  expect[1] = 23;
  expect[2] = 10;
  expect[3] = 0;
  if (order_check(&ready, expect, 4, 8) != 0)
    {
      return order_fail(8);
    }

  printf("deadline order: ok case 8\n");

  /* 9. Pending merge keeps the deadline task ahead of priority 255. */

  order_reset();
  dq_init(&ready);
  idle = order_tcb(0, 0, TCB_FLAG_SCHED_FIFO, 0);
  a = order_tcb(1, 10, TCB_FLAG_SCHED_FIFO, 100);
  b = order_tcb(2, 11, TCB_FLAG_SCHED_FIFO, 255);
  c = order_tcb(3, 20, TCB_FLAG_SCHED_FIFO, 1);
  d = order_tcb(4, 21, TCB_FLAG_SCHED_FIFO, 255);
  order_key(c, 0, 100, 1, true);
  order_key(d, 1, 500, 2, true);
  order_anchor_idle(&ready, idle);
  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  nxsched_add_runnable(d, &ready);
  nxsched_add_runnable(c, &ready);
  expect[0] = 20;
  expect[1] = 21;
  expect[2] = 11;
  expect[3] = 10;
  expect[4] = 0;
  if (order_check(&ready, expect, 5, 9) != 0)
    {
      return order_fail(9);
    }

  order_reset();
  dq_init(&ready);
  dq_init(&pending);
  idle = order_tcb(0, 0, TCB_FLAG_SCHED_FIFO, 0);
  a = order_tcb(1, 10, TCB_FLAG_SCHED_FIFO, 100);
  b = order_tcb(2, 11, TCB_FLAG_SCHED_FIFO, 255);
  c = order_tcb(3, 20, TCB_FLAG_SCHED_FIFO, 1);
  d = order_tcb(4, 21, TCB_FLAG_SCHED_FIFO, 255);
  order_key(c, 0, 100, 1, true);
  order_key(d, 1, 500, 2, true);
  order_anchor_idle(&ready, idle);
  nxsched_add_runnable(a, &ready);
  a->task_state = TSTATE_TASK_RUNNING;
  nxsched_add_runnable(b, &pending);
  nxsched_add_runnable(d, &pending);
  nxsched_add_runnable(c, &pending);
  seq = c->deadline->runnable_seq;
  if (!nxsched_merge_runnable(&ready, &pending, false) ||
      c->deadline->runnable_seq != seq ||
      d->deadline->runnable_seq != 2 || pending.head != NULL)
    {
      return order_fail(9);
    }

  if (order_check(&ready, expect, 5, 9) != 0)
    {
      return order_fail(9);
    }

  printf("deadline order: ok case 9\n");

  /* 10. Raising a normal task to 255 does not pass a deadline task.
   * Changing a deadline task's sched_priority does not move it.
   */

  order_reset();
  dq_init(&ready);
  idle = order_tcb(0, 0, TCB_FLAG_SCHED_FIFO, 0);
  a = order_tcb(1, 10, TCB_FLAG_SCHED_FIFO, 10);
  b = order_tcb(2, 20, TCB_FLAG_SCHED_FIFO, 1);
  order_key(b, 0, 1000, 3, true);
  order_anchor_idle(&ready, idle);
  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  dq_rem((FAR dq_entry_t *)a, &ready);
  a->sched_priority = 255;
  nxsched_add_runnable(a, &ready);
  expect[0] = 20;
  expect[1] = 10;
  expect[2] = 0;
  if (order_check(&ready, expect, 3, 10) != 0)
    {
      return order_fail(10);
    }

  if (!nxsched_deadline_outranks_priority(b, 255, true) ||
      nxsched_deadline_outranks_priority(idle, 1, true) ||
      !nxsched_deadline_outranks_priority(a, 255, true) ||
      nxsched_deadline_outranks_priority(a, 255, false))
    {
      return order_fail(10);
    }

  c = order_tcb(3, 12, TCB_FLAG_SCHED_FIFO, 100);
  if (!nxsched_deadline_outranks_priority(c, 100, true) ||
      nxsched_deadline_outranks_priority(c, 100, false) ||
      nxsched_deadline_outranks_priority(c, 150, true))
    {
      return order_fail(10);
    }

  b->task_state = TSTATE_TASK_READYTORUN;
  seq = b->deadline->runnable_seq;
  self = this_task()->pid;
  if (nxsched_set_priority(b, 200) != OK || b->sched_priority != 200 ||
      b->deadline->runnable_seq != seq || this_task()->pid != self)
    {
      return order_fail(10);
    }

  if (order_check(&ready, expect, 3, 10) != 0)
    {
      return order_fail(10);
    }

  printf("deadline order: ok case 10\n");

  /* 11. Idle stays at the tail behind a deadline task and priority 255. */

  order_reset();
  dq_init(&ready);
  idle = order_tcb(0, 0, TCB_FLAG_SCHED_FIFO, 0);
  a = order_tcb(1, 11, TCB_FLAG_SCHED_FIFO, 255);
  b = order_tcb(2, 20, TCB_FLAG_SCHED_FIFO, 0);
  c = order_tcb(3, 12, TCB_FLAG_SCHED_FIFO, 1);
  order_key(b, 0, 1, 1, true);
  order_anchor_idle(&ready, idle);
  nxsched_add_runnable(a, &ready);
  nxsched_add_runnable(b, &ready);
  nxsched_add_runnable(c, &ready);
  expect[0] = 20;
  expect[1] = 11;
  expect[2] = 12;
  expect[3] = 0;
  if (order_check(&ready, expect, 4, 11) != 0 ||
      !is_idle_task((FAR struct tcb_s *)ready.tail) ||
      nxsched_runnable_precedes(idle, b) ||
      !nxsched_runnable_precedes(b, idle))
    {
      return order_fail(11);
    }

  printf("deadline order: ok case 11\n");

  /* 12. Merge states: one running head, everyone else ready, no duplicates.
   * The admission sequence is not rewritten.
   */

  order_reset();
  dq_init(&ready);
  dq_init(&pending);
  idle = order_tcb(0, 0, TCB_FLAG_SCHED_FIFO, 0);
  a = order_tcb(1, 10, TCB_FLAG_SCHED_FIFO, 100);
  b = order_tcb(2, 20, TCB_FLAG_SCHED_FIFO, 0);
  order_key(b, 0, 50, 4, true);
  order_anchor_idle(&ready, idle);
  nxsched_add_runnable(a, &ready);
  a->task_state = TSTATE_TASK_RUNNING;
  nxsched_add_runnable(b, &pending);
  b->task_state = TSTATE_TASK_PENDING;
  seq = b->deadline->runnable_seq;
  if (!nxsched_merge_runnable(&ready, &pending, false))
    {
      return order_fail(12);
    }

  expect[0] = 20;
  expect[1] = 10;
  expect[2] = 0;
  if (order_check(&ready, expect, 3, 12) != 0 ||
      b->task_state != TSTATE_TASK_RUNNING ||
      a->task_state != TSTATE_TASK_READYTORUN ||
      idle->task_state != TSTATE_TASK_READYTORUN ||
      b->deadline->runnable_seq != seq ||
      pending.head != NULL || pending.tail != NULL ||
      b->flink != a || a->flink != idle || idle->flink != NULL)
    {
      return order_fail(12);
    }

  printf("deadline order: ok case 12\n");
  printf("deadline order: PASS cases 1-12\n");
  return 0;
}

#endif /* CONFIG_TESTING_DEADLINE */
