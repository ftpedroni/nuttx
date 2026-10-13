/****************************************************************************
 * sched/sched/sched_deadline_cbs.c
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

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wdog.h>

#include "sched/sched.h"

#ifdef CONFIG_TESTING_DEADLINE
#include <stdio.h>
#include <semaphore.h>
#include <nuttx/signal.h>
#elif defined(CONFIG_SCHED_DEADLINE_TRACE)
#include <stdio.h>
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct deadline_u128_s
{
  uint64_t hi;
  uint64_t lo;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Release-order tie break. 0 is never issued. Assigned only while
 * interrupts are disabled, on a real release edge.
 */

static uint64_t g_dl_seq;

/* True while a tick may remove the running task. Yield and unlock always
 * may. A tickless reassess that forbids switches only marks the debt.
 */

static bool g_dl_switch_ok = true;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void deadline_arm(FAR struct tcb_s *tcb);
static void deadline_stamp_seq(FAR struct deadline_s *dl);

static struct deadline_u128_s deadline_mul_u64(uint64_t a, uint64_t b)
{
  uint64_t a0 = (uint32_t)a;
  uint64_t a1 = a >> 32;
  uint64_t b0 = (uint32_t)b;
  uint64_t b1 = b >> 32;
  uint64_t p00 = a0 * b0;
  uint64_t p01 = a0 * b1;
  uint64_t p10 = a1 * b0;
  uint64_t p11 = a1 * b1;
  uint64_t mid;
  struct deadline_u128_s out;

  mid = (p00 >> 32) + (uint32_t)p01 + (uint32_t)p10;
  out.lo = (p00 & 0xffffffffu) | (mid << 32);
  out.hi = p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32);
  return out;
}

static int deadline_cmp_u128(struct deadline_u128_s x,
                             struct deadline_u128_s y)
{
  if (x.hi != y.hi)
    {
      return x.hi > y.hi ? 1 : -1;
    }

  if (x.lo != y.lo)
    {
      return x.lo > y.lo ? 1 : -1;
    }

  return 0;
}

/* True when q * P > Q * span. Equality is false. Products are 128-bit. */

static bool deadline_ratio_exceeds(uint64_t q, uint64_t period,
                                   uint64_t runtime, uint64_t span)
{
  if (period == 0)
    {
      return true;
    }

  return deadline_cmp_u128(deadline_mul_u64(q, period),
                           deadline_mul_u64(runtime, span)) > 0;
}

static bool deadline_add_u64(uint64_t a, uint64_t b, FAR uint64_t *out)
{
  if (a > UINT64_MAX - b)
    {
      return false;
    }

  *out = a + b;
  return true;
}

#ifdef CONFIG_SCHED_DEADLINE_TRACE

/* Fixed diagnostic log. Recording does not change a CBS decision.
 * 6144 slots. overrun_ns is copied into the slot and is not used
 * to decide a charge or a replenishment. The slot array is a ring.
 * Once full, the oldest event is overwritten and g_dltrace_ov
 * counts that loss.
 * sched_deadline_trace_freeze() stops the ring at the first missed
 * job so that job and the events still in the ring stay put.
 * Five hyperperiods produced 2216 events, about 1230 per second.
 * Thirty hyperperiods are about 13000 events, more than 6144, so a
 * linear buffer would drop a late first miss. The ring holds about
 * five seconds, which covers a period-scale stall and the period
 * before it.
 */

#define DLTRACE_N 6144

enum
{
  DLTR_CHARGE = 1,
  DLTR_THROTTLE = 2,
  DLTR_PENDING = 3,
  DLTR_REPLENISH = 4,
  DLTR_BLOCK = 5,
  DLTR_WAKE = 6,
  DLTR_SWITCH = 7
};

struct dltrace_ev_s
{
  uint64_t ts;
  uint64_t remaining;
  uint64_t abs_deadline;
  uint64_t overrun;
  int16_t pid;
  uint8_t kind;
  uint8_t sched;
  uint32_t extra;
};

static struct dltrace_ev_s g_dltrace[DLTRACE_N];
static uint32_t g_dltrace_i;
static uint32_t g_dltrace_count;
static uint32_t g_dltrace_ov;
static uint32_t g_dltrace_frozen_drops;
static volatile uint8_t g_dltrace_frozen;

static uint32_t dltrace_index(uint32_t ord)
{
  uint32_t start;

  if (g_dltrace_count < DLTRACE_N)
    {
      return ord;
    }

  start = g_dltrace_i;
  return (start + ord) % DLTRACE_N;
}

static void dltrace_put(uint8_t kind, FAR struct tcb_s *tcb, uint32_t extra,
                        uint64_t remaining, uint64_t abs_deadline)
{
  irqstate_t flags;
  FAR struct dltrace_ev_s *ev;

  flags = enter_critical_section();
  if (g_dltrace_frozen)
    {
      g_dltrace_frozen_drops++;
      leave_critical_section(flags);
      return;
    }

  ev = &g_dltrace[g_dltrace_i];
  g_dltrace_i++;
  if (g_dltrace_i >= DLTRACE_N)
    {
      g_dltrace_i = 0;
    }

  if (g_dltrace_count < DLTRACE_N)
    {
      g_dltrace_count++;
    }
  else
    {
      g_dltrace_ov++;
    }

  ev->ts = clock_systime_nsec();
  ev->remaining = remaining;
  ev->abs_deadline = abs_deadline;
  ev->overrun = 0;
  ev->pid = (tcb != NULL) ? (int16_t)tcb->pid : -1;
  ev->kind = kind;
  ev->sched = 0;
  if (tcb != NULL && tcb->deadline != NULL)
    {
      ev->sched = tcb->deadline->sched;
      ev->overrun = tcb->deadline->overrun_ns;
    }

  ev->extra = extra;
  leave_critical_section(flags);
}

#else
#  define dltrace_put(kind, tcb, extra, remaining, abs_deadline) ((void)0)
#endif

static void deadline_begin(FAR struct tcb_s *tcb, uint64_t now)
{
  FAR struct deadline_s *dl = tcb->deadline;

  if (dl == NULL || dl->sched != DL_SCHED_ELIGIBLE ||
      (dl->flags & DEADLINE_FLAG_KEY_VALID) == 0)
    {
      return;
    }

  dl->flags |= DEADLINE_FLAG_EXECUTING;
  dl->accounted_ns = now;
  nxsched_deadline_arm_budget(tcb);
}

static void deadline_stamp_seq(FAR struct deadline_s *dl)
{
  g_dl_seq++;
  if (g_dl_seq == 0)
    {
      g_dl_seq = 1;
    }

  dl->runnable_seq = g_dl_seq;
}

static void deadline_arm_if_idle(FAR struct tcb_s *tcb)
{
  FAR struct deadline_s *dl = tcb->deadline;

  /* Synthetic CBS fixtures are not schedulable. Arming their stack
   * watchdog would fire after the fixture returns.
   */

  if (dl != NULL &&
      (dl->flags & DEADLINE_FLAG_SCHEDULABLE) != 0 &&
      !WDOG_ISACTIVE(&dl->replenish))
    {
      deadline_arm(tcb);
    }
}

static bool deadline_detach(FAR struct tcb_s *tcb, FAR dq_queue_t *ready,
                            FAR dq_queue_t *throttle, bool live)
{
  FAR struct deadline_s *dl = tcb->deadline;
  bool switched = false;

  dl->sched = DL_SCHED_THROTTLED;
  dl->flags &= ~(DEADLINE_FLAG_EXECUTING | DEADLINE_FLAG_THROTTLE_PENDING);
  dltrace_put(DLTR_THROTTLE, tcb, live ? 1u : 0u,
              dl->remaining_ns, dl->abs_deadline);

  if (live)
    {
      switched = nxsched_remove_readytorun(tcb);
      nxsched_add_blocked(tcb, TSTATE_WAIT_DLTHROTTLE);
      return switched;
    }

  if (ready != NULL)
    {
      dq_rem((FAR dq_entry_t *)tcb, ready);
      tcb->flink = NULL;
      tcb->blink = NULL;
      tcb->task_state = TSTATE_WAIT_DLTHROTTLE;
      if (throttle != NULL)
        {
          dq_addlast((FAR dq_entry_t *)tcb, throttle);
        }
    }

  return false;
}

static void deadline_depleted(FAR struct tcb_s *tcb)
{
  FAR struct deadline_s *dl = tcb->deadline;

  if (dl->sched != DL_SCHED_ELIGIBLE || dl->remaining_ns != 0)
    {
      return;
    }

  if (tcb->lockcount > 0 || !g_dl_switch_ok)
    {
      dl->flags |= DEADLINE_FLAG_THROTTLE_PENDING;
      dltrace_put(DLTR_PENDING, tcb, tcb->lockcount,
                  dl->remaining_ns, dl->abs_deadline);
      deadline_arm_if_idle(tcb);
      return;
    }

  deadline_detach(tcb, NULL, NULL,
                  (dl->flags & DEADLINE_FLAG_SCHEDULABLE) != 0);
  deadline_arm_if_idle(tcb);
}

static uint64_t deadline_account(FAR struct tcb_s *tcb, uint64_t now)
{
  FAR struct deadline_s *dl;
  uint64_t delta;
  uint64_t excess;

  if (tcb == NULL || tcb->deadline == NULL)
    {
      return 0;
    }

  dl = tcb->deadline;
  if ((dl->flags & DEADLINE_FLAG_EXECUTING) == 0 ||
      dl->sched != DL_SCHED_ELIGIBLE)
    {
      return 0;
    }

  if (now < dl->accounted_ns)
    {
      return 0;
    }

  delta = now - dl->accounted_ns;
  dl->accounted_ns = now;
  if (delta == 0)
    {
      return 0;
    }

  if (delta > dl->remaining_ns)
    {
      excess = delta - dl->remaining_ns;
      if (!deadline_add_u64(dl->overrun_ns, excess, &dl->overrun_ns))
        {
          dl->overrun_ns = UINT64_MAX;
        }

      dl->remaining_ns = 0;
    }
  else
    {
      dl->remaining_ns -= delta;
    }

  dltrace_put(DLTR_CHARGE, tcb,
              delta > UINT32_MAX ? UINT32_MAX : (uint32_t)delta,
              dl->remaining_ns, dl->abs_deadline);
  deadline_depleted(tcb);
  return delta;
}

/* One posted budget pays overrun_ns at most once, and at most one Q.
 * Debt-free: remaining becomes Q. Debt below Q: remaining is Q minus
 * that debt and the debt is cleared. Debt of Q or more: remaining stays
 * 0 and only one Q is subtracted. The caller has already decided that
 * this call is a real post. Unsigned subtraction happens only on the
 * side that is large enough, so neither field wraps.
 */

static void deadline_pay_grant(FAR struct deadline_s *dl)
{
  uint64_t q = dl->runtime_ns;

  if (dl->overrun_ns < q)
    {
      dl->remaining_ns = q - dl->overrun_ns;
      dl->overrun_ns = 0;
      return;
    }

  dl->remaining_ns = 0;
  if (q > 0)
    {
      dl->overrun_ns -= q;
    }
}

/* Called from nxsched_deadline_woken(). unused_code only suppresses
 * the unused-function warning when the test build does not reference it.
 */

static void unused_code deadline_wake(FAR struct tcb_s *tcb, uint64_t now)
{
  FAR struct deadline_s *dl = tcb->deadline;
  uint64_t span;

  if (dl == NULL ||
      (dl->sched != DL_SCHED_INACTIVE && dl->sched != DL_SCHED_BLOCKED))
    {
      return;
    }

  if (dl->abs_deadline <= now || dl->period_ns == 0)
    {
      if (!deadline_add_u64(now, dl->deadline_ns, &dl->abs_deadline))
        {
          dl->abs_deadline = UINT64_MAX;
        }

      deadline_pay_grant(dl);
    }
  else
    {
      span = dl->abs_deadline - now;
      if (deadline_ratio_exceeds(dl->remaining_ns, dl->period_ns,
                                 dl->runtime_ns, span))
        {
          if (!deadline_add_u64(now, dl->deadline_ns, &dl->abs_deadline))
            {
              dl->abs_deadline = UINT64_MAX;
            }

          deadline_pay_grant(dl);
        }
    }

  dl->sched = DL_SCHED_ELIGIBLE;
  dl->flags |= DEADLINE_FLAG_KEY_VALID;
}

/* Called from nxsched_deadline_blocked(). */

static void unused_code deadline_block(FAR struct tcb_s *tcb, uint64_t now)
{
  FAR struct deadline_s *dl;

  deadline_account(tcb, now);
  dl = tcb->deadline;
  if (dl == NULL)
    {
      return;
    }

  dl->flags &= ~(DEADLINE_FLAG_EXECUTING | DEADLINE_FLAG_THROTTLE_PENDING);

  /* Budget exhaustion in this account already throttled the task and
   * armed replenishment. That timer is the only way back. Cancelling
   * it here left the task parked on TSTATE_WAIT_DLTHROTTLE forever
   * after the semaphore post. A user wait that still has budget is
   * closed by the wake rule, so that timer must not stay armed.
   */

  if (dl->sched == DL_SCHED_THROTTLED)
    {
      dltrace_put(DLTR_BLOCK, tcb, 1u, dl->remaining_ns, dl->abs_deadline);
      deadline_arm_if_idle(tcb);
      return;
    }

  wd_cancel(&dl->replenish);
  if (dl->sched == DL_SCHED_ELIGIBLE)
    {
      dl->sched = DL_SCHED_BLOCKED;
    }

  dltrace_put(DLTR_BLOCK, tcb, 0u, dl->remaining_ns, dl->abs_deadline);
}

static void deadline_replenish_math(FAR struct deadline_s *dl, uint64_t now)
{
  uint64_t next;

  if (dl->period_ns == 0)
    {
      if (!deadline_add_u64(now, dl->deadline_ns, &dl->abs_deadline))
        {
          dl->abs_deadline = UINT64_MAX;
        }

      deadline_pay_grant(dl);
      return;
    }

  /* Early callback, including a second call at the stamp that just
   * posted now+period. No second credit.
   */

  if (dl->abs_deadline > now)
    {
      return;
    }

  /* One post over a span of one period. Missed periods are not banked.
   * A debt-free server gets Q, so remaining / (deadline - now) = Q / P.
   * Unpaid overrun is subtracted from that one Q and is not cleared
   * except by that subtraction. Overflow grants nothing and does not
   * touch the debt. It must not become UINT64_MAX credit.
   */

  if (!deadline_add_u64(now, dl->period_ns, &next))
    {
      return;
    }

  dl->abs_deadline = next;
  deadline_pay_grant(dl);
}

static void deadline_arm(FAR struct tcb_s *tcb);

static void deadline_replenish_apply(FAR struct tcb_s *tcb, uint64_t now)
{
  FAR struct deadline_s *dl = tcb->deadline;

  if (dl == NULL || dl->sched == DL_SCHED_STOPPED ||
      dl->sched == DL_SCHED_INACTIVE)
    {
      return;
    }

  deadline_replenish_math(dl, now);
  dl->replenish_count++;
  dltrace_put(DLTR_REPLENISH, tcb, 0u, dl->remaining_ns, dl->abs_deadline);
  if (dl->remaining_ns > 0)
    {
      dl->flags &= ~DEADLINE_FLAG_THROTTLE_PENDING;
    }

  /* A failed or not-yet-due replenishment leaves a depleted server
   * throttled. Zero credit is not eligibility.
   */

  if (dl->sched != DL_SCHED_THROTTLED || dl->remaining_ns == 0)
    {
      return;
    }

  dl->sched = DL_SCHED_ELIGIBLE;
  dl->flags |= DEADLINE_FLAG_KEY_VALID;
  deadline_stamp_seq(dl);
  if ((dl->flags & DEADLINE_FLAG_SCHEDULABLE) != 0 &&
      tcb->task_state == TSTATE_WAIT_DLTHROTTLE)
    {
      FAR struct tcb_s *prev = this_task();

      nxsched_remove_blocked(tcb);
      if (nxsched_add_readytorun(tcb))
        {
          up_switch_context(this_task(), prev);
        }
    }
}

static void deadline_replenish_cb(wdparm_t arg)
{
  FAR struct tcb_s *tcb = (FAR struct tcb_s *)arg;
  FAR struct deadline_s *dl;
  uint32_t gen;

  if (tcb == NULL || tcb->deadline == NULL)
    {
      return;
    }

  dl = tcb->deadline;
  if (dl->sched == DL_SCHED_STOPPED || dl->sched == DL_SCHED_INACTIVE)
    {
      return;
    }

  gen = dl->generation;
  deadline_replenish_apply(tcb, clock_systime_nsec());
  if (tcb->deadline != dl || dl->generation != gen ||
      dl->sched == DL_SCHED_STOPPED)
    {
      return;
    }

  /* One replenishment while the task is still on a user wait. Arming
   * again would add another runtime if that wait outlasts the new
   * deadline. A throttle queue or a runnable task keeps the periodic
   * arm. wd_start replaces a live timer, so this is never a second
   * concurrent watchdog.
   */

  if (dl->sched != DL_SCHED_THROTTLED &&
      tcb->task_state != TSTATE_TASK_PENDING &&
      tcb->task_state != TSTATE_TASK_READYTORUN &&
      tcb->task_state != TSTATE_TASK_RUNNING &&
      tcb->task_state != TSTATE_WAIT_DLTHROTTLE)
    {
      return;
    }

  deadline_arm(tcb);
}

static void deadline_arm(FAR struct tcb_s *tcb)
{
  FAR struct deadline_s *dl = tcb->deadline;
  uint64_t now = clock_systime_nsec();
  uint64_t delta;
  clock_t delay;

  if (dl->abs_deadline <= now)
    {
      delay = 1;
    }
  else
    {
      delta = dl->abs_deadline - now;
      if (delta / NSEC_PER_TICK >= (uint64_t)WDOG_MAX_DELAY)
        {
          delay = WDOG_MAX_DELAY;
        }
      else
        {
          delay = (clock_t)NSEC2TICK(delta);
          if (delay < 1)
            {
              delay = 1;
            }
        }
    }

  dl->generation++;
  wd_start(&dl->replenish, delay, deadline_replenish_cb, (wdparm_t)tcb);
}

static bool deadline_release(FAR struct tcb_s *tcb, FAR dq_queue_t *ready,
                             FAR dq_queue_t *throttle)
{
  FAR struct deadline_s *dl;
  bool live;

  if (tcb == NULL || tcb->deadline == NULL ||
      (tcb->flags & TCB_FLAG_POLICY_MASK) != TCB_FLAG_SCHED_DEADLINE)
    {
      return false;
    }

  dl = tcb->deadline;
  if ((dl->flags & DEADLINE_FLAG_THROTTLE_PENDING) == 0 ||
      tcb->lockcount != 0)
    {
      return false;
    }

  if (dl->remaining_ns > 0)
    {
      dl->flags &= ~DEADLINE_FLAG_THROTTLE_PENDING;
      return false;
    }

  live = ready == NULL && (dl->flags & DEADLINE_FLAG_SCHEDULABLE) != 0;
  return deadline_detach(tcb, ready, throttle, live);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void deadline_blocked_at(FAR struct tcb_s *tcb, uint64_t now)
{
  deadline_block(tcb, now);
}

void nxsched_deadline_blocked(FAR struct tcb_s *tcb)
{
  if (tcb == NULL ||
      (tcb->flags & TCB_FLAG_POLICY_MASK) != TCB_FLAG_SCHED_DEADLINE)
    {
      return;
    }

  deadline_blocked_at(tcb, clock_systime_nsec());
}

static void deadline_woken_at(FAR struct tcb_s *tcb, uint64_t now)
{
  FAR struct deadline_s *dl = tcb->deadline;
  uint8_t before;

  before = dl->sched;
  deadline_wake(tcb, now);

  /* Posted while still throttled. Do not become runnable. The caller
   * parks the TCB on the throttle queue. Rearm if the watchdog is
   * idle so the park cannot be permanent.
   */

  if (before == DL_SCHED_THROTTLED)
    {
      dltrace_put(DLTR_WAKE, tcb, 1u, dl->remaining_ns, dl->abs_deadline);
      deadline_arm_if_idle(tcb);
      return;
    }

  if (before != DL_SCHED_INACTIVE && before != DL_SCHED_BLOCKED)
    {
      return;
    }

  if (dl->sched != DL_SCHED_ELIGIBLE)
    {
      return;
    }

  /* The wait is over and this job has no budget left. Stay throttled
   * until replenishment. The caller parks the TCB.
   */

  if (dl->remaining_ns == 0)
    {
      dl->sched = DL_SCHED_THROTTLED;
      dl->flags &= ~(DEADLINE_FLAG_EXECUTING |
                     DEADLINE_FLAG_THROTTLE_PENDING);
      dltrace_put(DLTR_WAKE, tcb, 2u, dl->remaining_ns, dl->abs_deadline);
      deadline_arm_if_idle(tcb);
      return;
    }

  dltrace_put(DLTR_WAKE, tcb, 0u, dl->remaining_ns, dl->abs_deadline);
  deadline_stamp_seq(dl);
}

void nxsched_deadline_woken(FAR struct tcb_s *tcb)
{
  if (tcb == NULL ||
      (tcb->flags & TCB_FLAG_POLICY_MASK) != TCB_FLAG_SCHED_DEADLINE ||
      tcb->deadline == NULL)
    {
      return;
    }

  deadline_woken_at(tcb, clock_systime_nsec());
}

void nxsched_deadline_arm_job(FAR struct tcb_s *tcb)
{
  FAR struct deadline_s *dl;
  uint64_t now;

  if (tcb == NULL)
    {
      return;
    }

  dl = tcb->deadline;
  if (dl == NULL || dl->sched != DL_SCHED_INACTIVE)
    {
      return;
    }

  now = clock_systime_nsec();
  if (!deadline_add_u64(now, dl->deadline_ns, &dl->abs_deadline))
    {
      dl->abs_deadline = UINT64_MAX;
    }

  deadline_pay_grant(dl);
  dl->sched = dl->remaining_ns > 0 ? DL_SCHED_ELIGIBLE : DL_SCHED_THROTTLED;
  dl->flags |= DEADLINE_FLAG_KEY_VALID | DEADLINE_FLAG_SCHEDULABLE;
  deadline_stamp_seq(dl);
}

bool nxsched_deadline_yield(FAR struct tcb_s *tcb)
{
  FAR struct deadline_s *dl;
  FAR struct tcb_s *prev;

  if (tcb == NULL || tcb->deadline == NULL)
    {
      return false;
    }

  prev = this_task();
  deadline_account(tcb, clock_systime_nsec());
  dl = tcb->deadline;
  if (dl == NULL || dl->sched != DL_SCHED_ELIGIBLE)
    {
      return this_task() != prev;
    }

  /* Give back the rest of this job. Overrun stays locked-execution debt. */

  dl->remaining_ns = 0;
  deadline_depleted(tcb);
  return this_task() != prev;
}

static void deadline_tick_at(FAR struct tcb_s *tcb, bool switch_ok,
                             uint64_t now)
{
  /* Removing the task from the ready list is not a switch. Sim and the
   * common IRQ return path resume the context up_switch_context()
   * installs, the same way round-robin preemption does.
   */

  if (tcb != NULL &&
      (tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_DEADLINE)
    {
      g_dl_switch_ok = switch_ok;
      deadline_account(tcb, now);
      g_dl_switch_ok = true;
      if (switch_ok && this_task() != tcb)
        {
          up_switch_context(this_task(), tcb);
        }
    }
}

#ifdef CONFIG_TESTING_DEADLINE
static int g_t9_arm;
static pid_t g_t9_low;
static pid_t g_t9_high;
static struct sched_deadline_t9_s g_t9_wit;
static int g_t9_mark;
static uint64_t g_t9_from_before;
static uint64_t g_t9_to_before;
static struct wdog_s g_t9_wd;

static void t9_note_tick(FAR struct tcb_s *tcb)
{
  if (!g_t9_arm || !up_interrupt_context() || tcb == NULL ||
      tcb->pid != g_t9_low || tcb->deadline == NULL)
    {
      return;
    }

  g_t9_wit.tick_irq = 1;
  g_t9_wit.tick_running = (int)tcb->pid;
  g_t9_wit.tick_head = (int)this_task()->pid;
  g_t9_wit.tick_now = clock_systime_nsec();
  g_t9_wit.tick_rem = tcb->deadline->remaining_ns;
}

static void t9_note_switch_before(FAR struct tcb_s *from,
                                  FAR struct tcb_s *to)
{
  g_t9_mark = 0;
  if (!g_t9_arm || g_t9_wit.sw_irq || !up_interrupt_context())
    {
      return;
    }

  if (from == NULL || to == NULL || from->pid != g_t9_low ||
      to->pid != g_t9_high || running_task() != from ||
      this_task() != to)
    {
      return;
    }

  g_t9_from_before = (from->deadline != NULL) ?
                     from->deadline->remaining_ns : 0;
  g_t9_to_before = (to->deadline != NULL) ?
                   to->deadline->remaining_ns : 0;
  g_t9_wit.sw_from = (int)from->pid;
  g_t9_wit.sw_to = (int)to->pid;
  g_t9_wit.sw_running = (int)running_task()->pid;
  g_t9_wit.sw_head = (int)this_task()->pid;
  g_t9_wit.sw_now = clock_systime_nsec();
  g_t9_mark = 1;
}

static void t9_note_switch_after(FAR struct tcb_s *from,
                                 FAR struct tcb_s *to)
{
  if (!g_t9_mark || from == NULL || to == NULL)
    {
      return;
    }

  g_t9_wit.sw_from_before = g_t9_from_before;
  g_t9_wit.sw_from_after = (from->deadline != NULL) ?
                           from->deadline->remaining_ns : 0;
  g_t9_wit.sw_to_before = g_t9_to_before;
  g_t9_wit.sw_to_after = (to->deadline != NULL) ?
                         to->deadline->remaining_ns : 0;
  g_t9_wit.sw_irq = 1;
  g_t9_mark = 0;
  g_t9_arm = 0;
}
#endif

void nxsched_deadline_tick(bool switch_ok)
{
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  flags = enter_critical_section();
  tcb = running_task();
  if (tcb != NULL &&
      (tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_DEADLINE)
    {
      deadline_tick_at(tcb, switch_ok, clock_systime_nsec());
#ifdef CONFIG_TESTING_DEADLINE
      t9_note_tick(tcb);
#endif
    }

  leave_critical_section(flags);
}

static void deadline_switch_at(FAR struct tcb_s *from, FAR struct tcb_s *to,
                               uint64_t now)
{
  if (from != NULL &&
      (from->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_DEADLINE)
    {
      deadline_account(from, now);
      if (from->deadline != NULL)
        {
          from->deadline->flags &= ~DEADLINE_FLAG_EXECUTING;
        }
    }

  if (to != NULL &&
      (to->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_DEADLINE)
    {
      deadline_begin(to, now);
    }

  dltrace_put(DLTR_SWITCH, to,
              (from != NULL) ? (uint32_t)from->pid : 0u,
              (to != NULL && to->deadline != NULL) ?
              to->deadline->remaining_ns : 0,
              (to != NULL && to->deadline != NULL) ?
              to->deadline->abs_deadline : 0);
}

void nxsched_deadline_switch(FAR struct tcb_s *from, FAR struct tcb_s *to)
{
#ifdef CONFIG_TESTING_DEADLINE
  t9_note_switch_before(from, to);
#endif
  deadline_switch_at(from, to, clock_systime_nsec());
#ifdef CONFIG_TESTING_DEADLINE
  t9_note_switch_after(from, to);
#endif
}

bool nxsched_deadline_release_lock(FAR struct tcb_s *tcb)
{
  return deadline_release(tcb, NULL, NULL);
}

void nxsched_deadline_stop(FAR struct tcb_s *tcb)
{
  FAR struct deadline_s *dl;

  if (tcb == NULL)
    {
      return;
    }

  dl = tcb->deadline;
  if (dl == NULL)
    {
      return;
    }

  /* A second stop of a stack fixture is a no-op. A reserved object
   * still has to reach release, which is what frees it.
   */

  if (dl->sched != DL_SCHED_STOPPED || WDOG_ISACTIVE(&dl->replenish))
    {
      dl->generation++;
      wd_cancel(&dl->replenish);
      dl->flags &= ~(DEADLINE_FLAG_EXECUTING |
                     DEADLINE_FLAG_THROTTLE_PENDING);
      dl->sched = DL_SCHED_STOPPED;
    }

  nxsched_deadline_release(tcb);
}

clock_t nxsched_deadline_timeslice(FAR struct tcb_s *tcb)
{
  FAR struct deadline_s *dl;
  uint64_t ticks;

  if (tcb == NULL || tcb->deadline == NULL)
    {
      return CLOCK_MAX;
    }

  dl = tcb->deadline;
  if (dl->sched != DL_SCHED_ELIGIBLE ||
      (dl->flags & DEADLINE_FLAG_EXECUTING) == 0)
    {
      return CLOCK_MAX;
    }

  if (dl->remaining_ns == 0)
    {
      return (dl->flags & DEADLINE_FLAG_THROTTLE_PENDING) != 0 ? 1 : 0;
    }

  ticks = (dl->remaining_ns + (uint64_t)NSEC_PER_TICK - 1) /
          (uint64_t)NSEC_PER_TICK;
  if (ticks > (uint64_t)CLOCK_MAX)
    {
      return CLOCK_MAX;
    }

  return (clock_t)ticks;
}

/* Tickless has no periodic tick. Arm the existing scheduler interval for
 * the remaining budget so a CPU-bound task still reaches deadline_account().
 * Periodic mode charges from nxsched_process_timer() and does not need this.
 * Synthetic fixtures are not schedulable and must not arm a real timer.
 */

void nxsched_deadline_arm_budget(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SCHED_TICKLESS
  FAR struct deadline_s *dl;

  if (tcb == NULL)
    {
      return;
    }

  dl = tcb->deadline;
  if (dl == NULL || (dl->flags & DEADLINE_FLAG_SCHEDULABLE) == 0)
    {
      return;
    }

  nxsched_timer_start(clock_systime_ticks(), nxsched_deadline_timeslice(tcb));
#else
  UNUSED(tcb);
#endif
}

#ifdef CONFIG_SCHED_DEADLINE_TRACE

void sched_deadline_trace_reset(void)
{
  irqstate_t flags = enter_critical_section();

  g_dltrace_i = 0;
  g_dltrace_count = 0;
  g_dltrace_ov = 0;
  g_dltrace_frozen_drops = 0;
  g_dltrace_frozen = 0;
  leave_critical_section(flags);
}

void sched_deadline_trace_freeze(void)
{
  irqstate_t flags = enter_critical_section();

  g_dltrace_frozen = 1;
  leave_critical_section(flags);
}

void sched_deadline_trace_report(uint64_t t0, uint64_t t1)
{
  static const char * const name[] =
  {
    "?", "charge", "throttle", "pending", "replenish",
    "block", "wake", "switch"
  };

  uint32_t count[8];
  uint32_t ord;
  uint32_t shown;
  uint32_t window_n;
  uint32_t n;

  memset(count, 0, sizeof(count));
  n = g_dltrace_count;
  for (ord = 0; ord < n; ord++)
    {
      uint8_t kind = g_dltrace[dltrace_index(ord)].kind;

      if (kind < 8)
        {
          count[kind]++;
        }
    }

  printf("dltrace: used=%lu overwritten=%lu frozen=%u frozen_drops=%lu "
         "cap=%d charge=%lu throttle=%lu pending=%lu replenish=%lu "
         "block=%lu wake=%lu switch=%lu\n",
         (unsigned long)n, (unsigned long)g_dltrace_ov,
         (unsigned)g_dltrace_frozen,
         (unsigned long)g_dltrace_frozen_drops, DLTRACE_N,
         (unsigned long)count[DLTR_CHARGE],
         (unsigned long)count[DLTR_THROTTLE],
         (unsigned long)count[DLTR_PENDING],
         (unsigned long)count[DLTR_REPLENISH],
         (unsigned long)count[DLTR_BLOCK],
         (unsigned long)count[DLTR_WAKE],
         (unsigned long)count[DLTR_SWITCH]);

  if (t0 == 0 || t1 <= t0)
    {
      printf("dltrace: window=none\n");
      return;
    }

  window_n = 0;
  for (ord = 0; ord < n; ord++)
    {
      FAR const struct dltrace_ev_s *ev = &g_dltrace[dltrace_index(ord)];

      if (ev->ts >= t0 && ev->ts <= t1)
        {
          window_n++;
        }
    }

  printf("dltrace: window_ns=%llu..%llu events=%lu\n",
         (unsigned long long)t0, (unsigned long long)t1,
         (unsigned long)window_n);
  shown = 0;
  for (ord = 0; ord < n; ord++)
    {
      FAR const struct dltrace_ev_s *ev = &g_dltrace[dltrace_index(ord)];
      int mechanism;

      if (ev->ts < t0 || ev->ts > t1)
        {
          continue;
        }

      /* Throttle and replenish are the isolation evidence. Charge
       * lines are a sample. An omitted charge is counted, not
       * treated as a zero debit.
       */

      mechanism = ev->kind == DLTR_THROTTLE ||
                  ev->kind == DLTR_REPLENISH ||
                  ev->kind == DLTR_PENDING;
      if (!mechanism && shown >= 64)
        {
          continue;
        }

      printf("dltrace: ev ts=%llu kind=%s pid=%d sched=%u extra=%lu "
             "remaining=%llu abs=%llu overrun=%llu\n",
             (unsigned long long)ev->ts,
             ev->kind < 8 ? name[ev->kind] : "?",
             (int)ev->pid, (unsigned)ev->sched,
             (unsigned long)ev->extra,
             (unsigned long long)ev->remaining,
             (unsigned long long)ev->abs_deadline,
             (unsigned long long)ev->overrun);
      if (!mechanism)
        {
          shown++;
        }
    }

  if (shown < window_n)
    {
      printf("dltrace: window_listed_sample=%lu window_events=%lu\n",
             (unsigned long)shown,
             (unsigned long)window_n);
    }
}

#endif

#ifdef CONFIG_TESTING_DEADLINE

static int cbs_fail(int id)
{
  printf("deadline cbs: FAIL case %d\n", id);
  return id;
}

static void cbs_params(FAR struct deadline_s *dl, uint64_t q, uint64_t d,
                       uint64_t p)
{
  dl->runtime_ns = q;
  dl->deadline_ns = d;
  dl->period_ns = p;
}

int sched_deadline_cbs_test(void)
{
  struct tcb_s tcb;
  struct deadline_s dl;
  struct tcb_s fifo;
  struct tcb_s idle;
  dq_queue_t ready;
  dq_queue_t throttle;
  uint64_t seq;
  uint64_t overrun;
  uint32_t fires;
  clock_t start;
  pid_t before;

  /* 1. First wake sets d and q. */

  memset(&tcb, 0, sizeof(tcb));
  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  cbs_params(&dl, 100, 500, 500);
  dl.sched = DL_SCHED_INACTIVE;
  deadline_wake(&tcb, 1000);
  if (dl.sched != DL_SCHED_ELIGIBLE || dl.abs_deadline != 1500 ||
      dl.remaining_ns != 100 ||
      (dl.flags & DEADLINE_FLAG_KEY_VALID) == 0)
    {
      return cbs_fail(1);
    }

  printf("deadline cbs: ok case 1\n");

  /* 2. Inequality false: keep d and q. */

  memset(&dl, 0, sizeof(dl));
  cbs_params(&dl, 10, 1000, 1000);
  dl.sched = DL_SCHED_BLOCKED;
  dl.abs_deadline = 2000;
  dl.remaining_ns = 10;
  deadline_wake(&tcb, 1000);
  if (dl.abs_deadline != 2000 || dl.remaining_ns != 10 ||
      dl.sched != DL_SCHED_ELIGIBLE)
    {
      return cbs_fail(2);
    }

  printf("deadline cbs: ok case 2\n");

  /* 3. Strict inequality resets. */

  memset(&dl, 0, sizeof(dl));
  cbs_params(&dl, 10, 1000, 1000);
  dl.sched = DL_SCHED_BLOCKED;
  dl.abs_deadline = 1500;
  dl.remaining_ns = 11;
  deadline_wake(&tcb, 1000);
  if (dl.abs_deadline != 2000 || dl.remaining_ns != 10)
    {
      return cbs_fail(3);
    }

  printf("deadline cbs: ok case 3\n");

  /* 4. Equality does not reset. */

  memset(&dl, 0, sizeof(dl));
  cbs_params(&dl, 10, 1000, 1000);
  dl.sched = DL_SCHED_BLOCKED;
  dl.abs_deadline = 1500;
  dl.remaining_ns = 5;
  deadline_wake(&tcb, 1000);
  if (dl.abs_deadline != 1500 || dl.remaining_ns != 5)
    {
      return cbs_fail(4);
    }

  printf("deadline cbs: ok case 4\n");

  /* 5. d <= now resets. */

  memset(&dl, 0, sizeof(dl));
  cbs_params(&dl, 10, 400, 400);
  dl.sched = DL_SCHED_BLOCKED;
  dl.abs_deadline = 1000;
  dl.remaining_ns = 5;
  deadline_wake(&tcb, 1000);
  if (dl.abs_deadline != 1400 || dl.remaining_ns != 10)
    {
      return cbs_fail(5);
    }

  printf("deadline cbs: ok case 5\n");

  /* 6 and 7. Charge once across tick and switch-out. Blocked time is 0. */

  memset(&dl, 0, sizeof(dl));
  cbs_params(&dl, 100, 1000, 1000);
  dl.sched = DL_SCHED_ELIGIBLE;
  dl.flags = DEADLINE_FLAG_KEY_VALID;
  dl.remaining_ns = 100;
  dl.abs_deadline = 10000;
  deadline_begin(&tcb, 0);
  if (deadline_account(&tcb, 3) != 3 || dl.remaining_ns != 97)
    {
      return cbs_fail(6);
    }

  if (deadline_account(&tcb, 5) != 2 || dl.remaining_ns != 95)
    {
      return cbs_fail(6);
    }

  deadline_block(&tcb, 5);
  if (dl.sched != DL_SCHED_BLOCKED ||
      deadline_account(&tcb, 11) != 0 || dl.remaining_ns != 95)
    {
      return cbs_fail(7);
    }

  deadline_wake(&tcb, 11);
  deadline_begin(&tcb, 11);
  if (deadline_account(&tcb, 14) != 3 || dl.remaining_ns != 92)
    {
      return cbs_fail(6);
    }

  printf("deadline cbs: ok case 6\n");
  printf("deadline cbs: ok case 7\n");

  /* 8. q == 0 throttles once. */

  memset(&dl, 0, sizeof(dl));
  memset(&tcb, 0, sizeof(tcb));
  tcb.deadline = &dl;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  cbs_params(&dl, 5, 100, 100);
  dl.sched = DL_SCHED_ELIGIBLE;
  dl.flags = DEADLINE_FLAG_KEY_VALID;
  dl.remaining_ns = 5;
  dl.abs_deadline = 1000;
  deadline_begin(&tcb, 0);
  if (deadline_account(&tcb, 5) != 5 || dl.sched != DL_SCHED_THROTTLED ||
      dl.remaining_ns != 0 ||
      (dl.flags & DEADLINE_FLAG_EXECUTING) != 0)
    {
      return cbs_fail(8);
    }

  if (deadline_account(&tcb, 9) != 0 || dl.sched != DL_SCHED_THROTTLED)
    {
      return cbs_fail(8);
    }

  printf("deadline cbs: ok case 8\n");

  /* 9. Throttled task leaves the ready queue and cannot be reinserted. */

  memset(&tcb, 0, sizeof(tcb));
  memset(&idle, 0, sizeof(idle));
  memset(&dl, 0, sizeof(dl));
  dq_init(&ready);
  dq_init(&throttle);
  idle.pid = 0;
  idle.flags = TCB_FLAG_SCHED_FIFO;
  idle.sched_priority = 0;
  idle.task_state = TSTATE_TASK_READYTORUN;
  dq_addlast((FAR dq_entry_t *)&idle, &ready);
  tcb.deadline = &dl;
  tcb.pid = 20;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  tcb.sched_priority = 1;
  cbs_params(&dl, 10, 100, 100);
  dl.sched = DL_SCHED_ELIGIBLE;
  dl.flags = DEADLINE_FLAG_KEY_VALID;
  dl.abs_deadline = 50;
  dl.runnable_seq = 1;
  dl.remaining_ns = 10;
  if (!nxsched_add_runnable(&tcb, &ready))
    {
      return cbs_fail(9);
    }

  deadline_detach(&tcb, &ready, &throttle, false);
  if (ready.head != (FAR dq_entry_t *)&idle ||
      dl.sched != DL_SCHED_THROTTLED ||
      throttle.head != (FAR dq_entry_t *)&tcb ||
      tcb.task_state != TSTATE_WAIT_DLTHROTTLE ||
      nxsched_add_runnable(&tcb, &ready) ||
      ready.head != (FAR dq_entry_t *)&idle)
    {
      return cbs_fail(9);
    }

  printf("deadline cbs: ok case 9\n");

  /* 10. One replenishment. */

  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  tcb.task_state = TSTATE_TASK_INVALID;
  cbs_params(&dl, 40, 100, 100);
  dl.sched = DL_SCHED_THROTTLED;
  dl.abs_deadline = 1000;
  dl.remaining_ns = 0;
  deadline_replenish_apply(&tcb, 1000);
  if (dl.abs_deadline != 1100 || dl.remaining_ns != 40 ||
      dl.sched != DL_SCHED_ELIGIBLE)
    {
      return cbs_fail(10);
    }

  printf("deadline cbs: ok case 10\n");

  /* 11. A late callback posts one budget at now+period, not steps*Q. */

  memset(&dl, 0, sizeof(dl));
  cbs_params(&dl, 10, 1000, 1000);
  dl.sched = DL_SCHED_THROTTLED;
  dl.abs_deadline = 1000;
  dl.remaining_ns = 0;
  deadline_replenish_apply(&tcb, 5000);
  if (dl.abs_deadline != 6000 || dl.remaining_ns != 10 ||
      dl.sched != DL_SCHED_ELIGIBLE || dl.abs_deadline <= 5000)
    {
      return cbs_fail(11);
    }

  printf("deadline cbs: ok case 11\n");

  /* 12. Nested lock defers throttle and keeps the overrun. */

  memset(&tcb, 0, sizeof(tcb));
  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  tcb.lockcount = 2;
  cbs_params(&dl, 4, 100, 100);
  dl.sched = DL_SCHED_ELIGIBLE;
  dl.flags = DEADLINE_FLAG_KEY_VALID;
  dl.remaining_ns = 4;
  dl.abs_deadline = 10000;
  deadline_begin(&tcb, 0);
  if (deadline_account(&tcb, 7) != 7 ||
      (dl.flags & DEADLINE_FLAG_THROTTLE_PENDING) == 0 ||
      dl.sched != DL_SCHED_ELIGIBLE || dl.remaining_ns != 0 ||
      dl.overrun_ns != 3)
    {
      return cbs_fail(12);
    }

  tcb.lockcount = 1;
  if (deadline_release(&tcb, NULL, NULL) ||
      dl.sched != DL_SCHED_ELIGIBLE || dl.overrun_ns != 3)
    {
      return cbs_fail(12);
    }

  overrun = dl.overrun_ns;
  deadline_account(&tcb, 9);
  if (dl.overrun_ns != overrun + 2 || dl.remaining_ns != 0)
    {
      return cbs_fail(12);
    }

  printf("deadline cbs: ok case 12\n");

  /* 13. Unlock applies the throttle and does not clear the overrun. */

  memset(&idle, 0, sizeof(idle));
  dq_init(&ready);
  dq_init(&throttle);
  idle.pid = 0;
  idle.flags = TCB_FLAG_SCHED_FIFO;
  idle.task_state = TSTATE_TASK_READYTORUN;
  dq_addlast((FAR dq_entry_t *)&idle, &ready);
  tcb.pid = 21;
  tcb.sched_priority = 1;
  tcb.lockcount = 0;
  dl.flags |= DEADLINE_FLAG_KEY_VALID;
  nxsched_add_runnable(&tcb, &ready);
  overrun = dl.overrun_ns;
  if (!deadline_release(&tcb, &ready, &throttle) &&
      dl.sched == DL_SCHED_THROTTLED &&
      dl.overrun_ns == overrun &&
      (dl.flags & DEADLINE_FLAG_THROTTLE_PENDING) == 0 &&
      ready.head == (FAR dq_entry_t *)&idle &&
      throttle.head == (FAR dq_entry_t *)&tcb)
    {
      printf("deadline cbs: ok case 13\n");
    }
  else
    {
      return cbs_fail(13);
    }

  /* 14. Watchdog fires, stop cancels it, and a second stop is a no-op. */

  memset(&tcb, 0, sizeof(tcb));
  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  cbs_params(&dl, 10, NSEC_PER_TICK, NSEC_PER_TICK);
  dl.sched = DL_SCHED_THROTTLED;
  dl.abs_deadline = clock_systime_nsec();
  dl.remaining_ns = 0;
  deadline_arm(&tcb);
  start = clock_systime_ticks();
  while (clock_systime_ticks() - start < 8 && dl.replenish_count == 0)
    {
      nxsig_usleep(1000);
    }

  if (dl.replenish_count == 0 || dl.sched != DL_SCHED_ELIGIBLE)
    {
      nxsched_deadline_stop(&tcb);
      return cbs_fail(14);
    }

  nxsched_deadline_stop(&tcb);
  fires = dl.replenish_count;
  start = clock_systime_ticks();
  while (clock_systime_ticks() - start < 4)
    {
      nxsig_usleep(1000);
    }

  nxsched_deadline_stop(&tcb);
  if (dl.replenish_count != fires || dl.sched != DL_SCHED_STOPPED ||
      WDOG_ISACTIVE(&dl.replenish))
    {
      return cbs_fail(14);
    }

  printf("deadline cbs: ok case 14\n");

  /* 15. Replenishment does not wake a semaphore waiter. */

  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.task_state = TSTATE_WAIT_SEM;
  cbs_params(&dl, 10, 100, 100);
  dl.sched = DL_SCHED_BLOCKED;
  dl.abs_deadline = 10;
  dl.remaining_ns = 0;
  deadline_replenish_apply(&tcb, 10);
  if (tcb.task_state != TSTATE_WAIT_SEM || dl.sched != DL_SCHED_BLOCKED ||
      dl.remaining_ns != 10)
    {
      return cbs_fail(15);
    }

  printf("deadline cbs: ok case 15\n");

  /* 16. Overflowing products and tick rounding. */

  memset(&dl, 0, sizeof(dl));
  cbs_params(&dl, 1, 50, 2);
  dl.sched = DL_SCHED_BLOCKED;
  dl.abs_deadline = 1001;
  dl.remaining_ns = UINT64_MAX;
  deadline_wake(&tcb, 1000);
  if (dl.remaining_ns != 1 || dl.abs_deadline != 1050)
    {
      return cbs_fail(16);
    }

  memset(&dl, 0, sizeof(dl));
  cbs_params(&dl, UINT64_MAX, 80, 1);
  dl.sched = DL_SCHED_BLOCKED;
  dl.abs_deadline = 1002;
  dl.remaining_ns = 1;
  deadline_wake(&tcb, 1000);
  if (dl.remaining_ns != 1 || dl.abs_deadline != 1002)
    {
      return cbs_fail(16);
    }

  if (NSEC2TICK(1) != 1 || NSEC2TICK(NSEC_PER_TICK) != 1 ||
      NSEC2TICK(NSEC_PER_TICK + 1) != 2)
    {
      return cbs_fail(16);
    }

  printf("deadline cbs: ok case 16\n");

  /* 17. A priority change does not reorder an eligible deadline task. */

  memset(&tcb, 0, sizeof(tcb));
  memset(&fifo, 0, sizeof(fifo));
  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.pid = 40;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  tcb.sched_priority = 1;
  tcb.task_state = TSTATE_TASK_READYTORUN;
  cbs_params(&dl, 10, 100, 100);
  dl.sched = DL_SCHED_ELIGIBLE;
  dl.flags = DEADLINE_FLAG_KEY_VALID;
  dl.abs_deadline = 10;
  dl.runnable_seq = 3;
  fifo.pid = 41;
  fifo.flags = TCB_FLAG_SCHED_FIFO;
  fifo.sched_priority = 255;
  before = this_task()->pid;
  if (nxsched_set_priority(&tcb, 200) != OK || tcb.sched_priority != 200 ||
      dl.abs_deadline != 10 || dl.runnable_seq != 3 ||
      !nxsched_runnable_precedes(&tcb, &fifo) ||
      this_task()->pid != before)
    {
      return cbs_fail(17);
    }

  printf("deadline cbs: ok case 17\n");

  /* 18. CBS does not assign or clear the tie-break sequence.
   * Production assignment on a real block-to-runnable edge is not
   * implemented, so that half of the tie rule stays unverified.
   */

  seq = 9;
  dl.runnable_seq = seq;
  dl.remaining_ns = 20;
  dl.abs_deadline = 5000;
  deadline_begin(&tcb, 0);
  deadline_account(&tcb, 4);
  deadline_replenish_math(&dl, 4);
  if (dl.runnable_seq != seq)
    {
      return cbs_fail(18);
    }

  printf("deadline cbs: ok case 18\n");

  /* 19. The last slice charged at block stays throttled and keeps the
   * watchdog. Stop before that callback runs. A block that still has
   * budget cancels the watchdog and becomes BLOCKED.
   */

  memset(&tcb, 0, sizeof(tcb));
  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  cbs_params(&dl, 10, 1000, 1000);
  dl.sched = DL_SCHED_THROTTLED;
  dl.abs_deadline = clock_systime_nsec();
  dl.remaining_ns = 0;
  dl.replenish_count = 0;
  deadline_arm(&tcb);
  nxsched_deadline_stop(&tcb);
  fires = dl.replenish_count;
  start = clock_systime_ticks();
  while (clock_systime_ticks() - start < 4)
    {
      nxsig_usleep(1000);
    }

  if (dl.replenish_count != fires || WDOG_ISACTIVE(&dl.replenish) ||
      dl.sched != DL_SCHED_STOPPED)
    {
      return cbs_fail(19);
    }

  memset(&tcb, 0, sizeof(tcb));
  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  cbs_params(&dl, 10, 1000, 1000);
  dl.sched = DL_SCHED_ELIGIBLE;
  dl.flags = DEADLINE_FLAG_KEY_VALID;
  dl.remaining_ns = 10;
  dl.abs_deadline = clock_systime_nsec() + 60000000000ull;
  deadline_begin(&tcb, 0);
  deadline_arm(&tcb);
  if (!WDOG_ISACTIVE(&dl.replenish))
    {
      nxsched_deadline_stop(&tcb);
      return cbs_fail(19);
    }

  deadline_block(&tcb, 10);
  nxsched_deadline_woken(&tcb);
  if (dl.sched != DL_SCHED_THROTTLED || dl.remaining_ns != 0 ||
      nxsched_deadline_eligible(&tcb) || !WDOG_ISACTIVE(&dl.replenish))
    {
      nxsched_deadline_stop(&tcb);
      return cbs_fail(19);
    }

  nxsched_deadline_stop(&tcb);
  if (WDOG_ISACTIVE(&dl.replenish))
    {
      return cbs_fail(19);
    }

  memset(&tcb, 0, sizeof(tcb));
  memset(&dl, 0, sizeof(dl));
  tcb.deadline = &dl;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  cbs_params(&dl, 50, 1000, 1000);
  dl.sched = DL_SCHED_ELIGIBLE;
  dl.flags = DEADLINE_FLAG_KEY_VALID;
  dl.remaining_ns = 50;
  dl.abs_deadline = clock_systime_nsec() + 60000000000ull;
  deadline_begin(&tcb, 0);
  deadline_arm(&tcb);
  deadline_block(&tcb, 10);
  if (dl.sched != DL_SCHED_BLOCKED || dl.remaining_ns != 40 ||
      WDOG_ISACTIVE(&dl.replenish))
    {
      nxsched_deadline_stop(&tcb);
      return cbs_fail(19);
    }

  nxsched_deadline_stop(&tcb);
  printf("deadline cbs: ok case 19\n");

  /* 20. Replenishment while on a semaphore does not run the task.
   * Insertion is refused until that replenishment, then allowed.
   * A wake of the still-throttled task arms an idle watchdog and
   * stays off the ready queue.
   */

  memset(&tcb, 0, sizeof(tcb));
  memset(&idle, 0, sizeof(idle));
  memset(&dl, 0, sizeof(dl));
  dq_init(&ready);
  idle.pid = 0;
  idle.flags = TCB_FLAG_SCHED_FIFO;
  idle.task_state = TSTATE_TASK_READYTORUN;
  dq_addlast((FAR dq_entry_t *)&idle, &ready);
  tcb.deadline = &dl;
  tcb.pid = 22;
  tcb.flags = TCB_FLAG_SCHED_DEADLINE;
  tcb.task_state = TSTATE_WAIT_SEM;
  tcb.sched_priority = 1;
  cbs_params(&dl, 40, 100, 100);
  dl.sched = DL_SCHED_THROTTLED;
  dl.flags = DEADLINE_FLAG_KEY_VALID | DEADLINE_FLAG_SCHEDULABLE;
  dl.abs_deadline = clock_systime_nsec() + 60000000000ull;
  dl.remaining_ns = 0;
  if (nxsched_add_runnable(&tcb, &ready))
    {
      return cbs_fail(20);
    }

  nxsched_deadline_woken(&tcb);
  if (dl.sched != DL_SCHED_THROTTLED || !WDOG_ISACTIVE(&dl.replenish) ||
      nxsched_deadline_eligible(&tcb))
    {
      nxsched_deadline_stop(&tcb);
      return cbs_fail(20);
    }

  nxsched_deadline_stop(&tcb);
  dl.flags = DEADLINE_FLAG_KEY_VALID;
  dl.sched = DL_SCHED_THROTTLED;
  dl.abs_deadline = 1000;
  dl.remaining_ns = 0;
  tcb.task_state = TSTATE_WAIT_SEM;
  deadline_replenish_apply(&tcb, 1000);
  if (tcb.task_state != TSTATE_WAIT_SEM || dl.sched != DL_SCHED_ELIGIBLE ||
      dl.remaining_ns != 40 || dl.abs_deadline != 1100 ||
      !nxsched_add_runnable(&tcb, &ready) ||
      ready.head != (FAR dq_entry_t *)&tcb)
    {
      return cbs_fail(20);
    }

  dq_rem((FAR dq_entry_t *)&tcb, &ready);
  nxsched_deadline_woken(&tcb);
  if (dl.sched != DL_SCHED_ELIGIBLE || dl.remaining_ns != 40 ||
      dl.abs_deadline != 1100)
    {
      return cbs_fail(20);
    }

  printf("deadline cbs: ok case 20\n");
  printf("deadline cbs: PASS cases 1-20\n");
  return 0;
}

static int time_fail(int id)
{
  printf("deadline time: FAIL T%d\n", id);
  return id;
}

static uint64_t time_roundup(uint64_t n, uint64_t d)
{
  return (n + d - 1) / d;
}

static void time_open(FAR struct tcb_s *tcb, FAR struct deadline_s *dl,
                      uint64_t q, uint64_t d, uint64_t p, uint64_t now)
{
  memset(tcb, 0, sizeof(*tcb));
  memset(dl, 0, sizeof(*dl));
  tcb->deadline = dl;
  tcb->flags = TCB_FLAG_SCHED_DEADLINE;
  tcb->pid = 70;
  cbs_params(dl, q, d, p);
  dl->sched = DL_SCHED_ELIGIBLE;
  dl->flags = DEADLINE_FLAG_KEY_VALID;
  dl->remaining_ns = q;
  dl->abs_deadline = now + d;
  deadline_switch_at(NULL, tcb, now);
}

int sched_deadline_time_test(void)
{
  struct tcb_s a;
  struct tcb_s b;
  struct deadline_s dla;
  struct deadline_s dlb;
  struct tcb_s fifo;
  const uint64_t ms = 1000000ull;
  const uint64_t q = 10 * ms;
  pid_t self;
  uint32_t ppm;
  uint32_t gen;
  uint64_t left;

  /* T1. Open interval: tick, same stamp, later stamp. Switch at the
   * same stamp charges nothing and closes the interval.
   */

  time_open(&a, &dla, q, 40 * ms, 40 * ms, 100 * ms);
  deadline_tick_at(&a, false, 104 * ms);
  if (dla.remaining_ns != 6 * ms || dla.accounted_ns != 104 * ms ||
      dla.sched != DL_SCHED_ELIGIBLE)
    {
      return time_fail(1);
    }

  deadline_tick_at(&a, false, 104 * ms);
  if (dla.remaining_ns != 6 * ms || dla.overrun_ns != 0)
    {
      return time_fail(1);
    }

  deadline_tick_at(&a, false, 106 * ms);
  if (dla.remaining_ns != 4 * ms || dla.replenish_count != 0 ||
      dla.sched != DL_SCHED_ELIGIBLE)
    {
      return time_fail(1);
    }

  deadline_tick_at(&a, false, 50 * ms);
  if (dla.remaining_ns != 4 * ms || dla.accounted_ns != 106 * ms ||
      (dla.flags & DEADLINE_FLAG_EXECUTING) == 0)
    {
      return time_fail(1);
    }

  deadline_switch_at(&a, NULL, 106 * ms);
  if (dla.remaining_ns != 4 * ms ||
      (dla.flags & DEADLINE_FLAG_EXECUTING) != 0)
    {
      return time_fail(1);
    }

  deadline_tick_at(&a, false, 110 * ms);
  if (dla.remaining_ns != 4 * ms)
    {
      return time_fail(1);
    }

  printf("deadline time: ok T1 remaining=%llu accounted=%llu\n",
         (unsigned long long)dla.remaining_ns,
         (unsigned long long)dla.accounted_ns);

  /* T2. Both orders. A normal task is not debited. */

  time_open(&a, &dla, 50 * ms, 100 * ms, 100 * ms, 0);
  memset(&fifo, 0, sizeof(fifo));
  fifo.flags = TCB_FLAG_SCHED_FIFO;
  fifo.pid = 3;
  left = dla.remaining_ns;
  deadline_tick_at(&fifo, false, 1000);
  deadline_switch_at(&fifo, NULL, 1000);
  if (dla.remaining_ns != left)
    {
      return time_fail(2);
    }

  deadline_tick_at(&a, false, 1000);
  deadline_switch_at(&a, &fifo, 1000);
  if (dla.remaining_ns != 50 * ms - 1000 ||
      (dla.flags & DEADLINE_FLAG_EXECUTING) != 0)
    {
      return time_fail(2);
    }

  deadline_tick_at(&a, false, 1000);
  if (dla.remaining_ns != 50 * ms - 1000)
    {
      return time_fail(2);
    }

  time_open(&a, &dla, 50 * ms, 100 * ms, 100 * ms, 0);
  deadline_switch_at(&a, &fifo, 1000);
  deadline_tick_at(&a, false, 1000);
  if (dla.remaining_ns != 50 * ms - 1000)
    {
      return time_fail(2);
    }

  time_open(&a, &dla, 50 * ms, 100 * ms, 100 * ms, 0);
  deadline_tick_at(&a, false, 1000);
  deadline_switch_at(&a, &fifo, 2500);
  if (dla.remaining_ns != 50 * ms - 2500 || fifo.deadline != NULL)
    {
      return time_fail(2);
    }

  time_open(&a, &dla, 50 * ms, 100 * ms, 100 * ms, 0);
  deadline_tick_at(&a, false, 1);
  deadline_switch_at(&a, &fifo, 1);
  if (dla.remaining_ns != 50 * ms - 1)
    {
      return time_fail(2);
    }

  time_open(&a, &dla, 50 * ms, 100 * ms, 100 * ms, 999999);
  deadline_tick_at(&a, false, 1000000);
  deadline_tick_at(&a, false, 1000001);
  if (dla.remaining_ns != 50 * ms - 2 || dla.accounted_ns != 1000001)
    {
      return time_fail(2);
    }

  printf("deadline time: ok T2 charged_ns=2500 one_ns=1 ms_edge=2\n");

  /* T3. Exact budget, one throttle, same stamp does not repeat it.
   * The last nanosecond goes through deadline_account() with the
   * default g_dl_switch_ok, which is what deadline_tick_at() uses
   * when switch_ok is true. Calling the tick wrapper with switch_ok
   * true would also up_switch_context() onto this stack TCB.
   */

  time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
  deadline_tick_at(&a, false, 9 * ms);
  if (dla.remaining_ns != ms || dla.sched != DL_SCHED_ELIGIBLE ||
      dla.overrun_ns != 0)
    {
      return time_fail(3);
    }

  deadline_account(&a, 10 * ms);
  if (dla.remaining_ns != 0 || dla.overrun_ns != 0 ||
      dla.sched != DL_SCHED_THROTTLED ||
      nxsched_deadline_eligible(&a) ||
      (dla.flags & DEADLINE_FLAG_EXECUTING) != 0 ||
      (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) != 0 ||
      WDOG_ISACTIVE(&dla.replenish))
    {
      return time_fail(3);
    }

  deadline_tick_at(&a, false, 10 * ms);
  deadline_switch_at(&a, NULL, 10 * ms);
  deadline_tick_at(&a, false, 11 * ms);
  if (dla.remaining_ns != 0 || dla.overrun_ns != 0 ||
      dla.sched != DL_SCHED_THROTTLED || dla.replenish_count != 0)
    {
      return time_fail(3);
    }

  printf("deadline time: ok T3 throttled remaining=0 overrun=0\n");

  /* T4. Locked overrun, then one deferred throttle. No switch. */

  self = this_task()->pid;
  time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
  a.lockcount = 1;
  deadline_tick_at(&a, false, 13 * ms);
  if (dla.remaining_ns != 0 || dla.overrun_ns != 3 * ms ||
      dla.sched != DL_SCHED_ELIGIBLE ||
      (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) == 0 ||
      this_task()->pid != self)
    {
      return time_fail(4);
    }

  deadline_tick_at(&a, false, 13 * ms);
  if (dla.overrun_ns != 3 * ms ||
      (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) == 0)
    {
      return time_fail(4);
    }

  a.lockcount = 0;
  if (nxsched_deadline_release_lock(&a) ||
      dla.sched != DL_SCHED_THROTTLED || dla.overrun_ns != 3 * ms ||
      (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) != 0 ||
      nxsched_deadline_eligible(&a) || this_task()->pid != self)
    {
      return time_fail(4);
    }

  if (nxsched_deadline_release_lock(&a) || dla.overrun_ns != 3 * ms)
    {
      return time_fail(4);
    }

  printf("deadline time: ok T4 overrun=%llu throttled\n",
         (unsigned long long)dla.overrun_ns);

  /* T5. Exhaustion on the block path keeps an already armed watchdog.
   * Wake while throttled stays ineligible. Replenish then allows
   * eligibility. The synthetic TCB is not SCHEDULABLE, so detach does
   * not touch g_readytorun.
   */

  time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
  dla.abs_deadline = 60ull * 1000 * ms;
  deadline_arm(&a);
  if (!WDOG_ISACTIVE(&dla.replenish))
    {
      nxsched_deadline_stop(&a);
      return time_fail(5);
    }

  deadline_blocked_at(&a, 10 * ms);
  if (dla.sched != DL_SCHED_THROTTLED || dla.remaining_ns != 0 ||
      nxsched_deadline_eligible(&a) || !WDOG_ISACTIVE(&dla.replenish))
    {
      nxsched_deadline_stop(&a);
      return time_fail(5);
    }

  nxsched_deadline_stop(&a);
  if (WDOG_ISACTIVE(&dla.replenish) || dla.sched != DL_SCHED_STOPPED)
    {
      return time_fail(5);
    }

  memset(&a, 0, sizeof(a));
  memset(&dla, 0, sizeof(dla));
  a.deadline = &dla;
  a.flags = TCB_FLAG_SCHED_DEADLINE;
  a.task_state = TSTATE_WAIT_SEM;
  cbs_params(&dla, q, 40 * ms, 40 * ms);
  dla.sched = DL_SCHED_THROTTLED;
  dla.flags = DEADLINE_FLAG_KEY_VALID;
  dla.abs_deadline = 60ull * 1000 * ms;
  dla.remaining_ns = 0;
  deadline_arm(&a);
  deadline_woken_at(&a, 1000);
  if (dla.sched != DL_SCHED_THROTTLED || nxsched_deadline_eligible(&a) ||
      !WDOG_ISACTIVE(&dla.replenish) || a.task_state != TSTATE_WAIT_SEM)
    {
      nxsched_deadline_stop(&a);
      return time_fail(5);
    }

  nxsched_deadline_stop(&a);
  memset(&a, 0, sizeof(a));
  memset(&dla, 0, sizeof(dla));
  a.deadline = &dla;
  a.flags = TCB_FLAG_SCHED_DEADLINE;
  a.task_state = TSTATE_WAIT_SEM;
  cbs_params(&dla, q, 40 * ms, 40 * ms);
  dla.sched = DL_SCHED_THROTTLED;
  dla.flags = DEADLINE_FLAG_KEY_VALID;
  dla.abs_deadline = 40 * ms;
  dla.remaining_ns = 0;
  deadline_replenish_apply(&a, 40 * ms);
  deadline_woken_at(&a, 40 * ms);
  if (a.task_state != TSTATE_WAIT_SEM || dla.sched != DL_SCHED_ELIGIBLE ||
      dla.remaining_ns != q || dla.abs_deadline != 80 * ms)
    {
      return time_fail(5);
    }

  printf("deadline time: ok T5 block_keeps_wd wake_stays_throttled\n");

  /* T6. Due replenishment is one budget at now+period.
   * The old steps*Q result (160 ms, 30 ms) is the defect.
   */

  memset(&dla, 0, sizeof(dla));
  memset(&a, 0, sizeof(a));
  a.deadline = &dla;
  a.flags = TCB_FLAG_SCHED_DEADLINE;
  a.task_state = TSTATE_WAIT_SEM;
  cbs_params(&dla, q, 40 * ms, 40 * ms);
  dla.sched = DL_SCHED_THROTTLED;
  dla.abs_deadline = 40 * ms;
  dla.remaining_ns = 0;
  deadline_replenish_apply(&a, 125 * ms);
  if (dla.abs_deadline != 165 * ms || dla.remaining_ns != q ||
      dla.replenish_count != 1 || dla.sched != DL_SCHED_ELIGIBLE)
    {
      return time_fail(6);
    }

  deadline_replenish_apply(&a, 125 * ms);
  if (dla.abs_deadline != 165 * ms || dla.remaining_ns != q ||
      dla.replenish_count != 2 || dla.sched != DL_SCHED_ELIGIBLE)
    {
      return time_fail(6);
    }

  deadline_switch_at(NULL, &a, 125 * ms);
  deadline_account(&a, 135 * ms);
  if (dla.remaining_ns != 0 || dla.abs_deadline != 165 * ms ||
      dla.sched != DL_SCHED_THROTTLED)
    {
      return time_fail(6);
    }

  deadline_replenish_apply(&a, 125 * ms);
  if (dla.abs_deadline != 165 * ms || dla.remaining_ns != 0 ||
      dla.replenish_count != 3 || dla.sched != DL_SCHED_THROTTLED)
    {
      return time_fail(6);
    }

  memset(&dla, 0, sizeof(dla));
  dla.runtime_ns = q;
  dla.deadline_ns = 40 * ms;
  dla.period_ns = 40 * ms;
  dla.abs_deadline = 40 * ms;
  dla.sched = DL_SCHED_THROTTLED;
  deadline_replenish_math(&dla, 40 * ms);
  if (dla.abs_deadline != 80 * ms || dla.remaining_ns != q)
    {
      return time_fail(6);
    }

  memset(&dla, 0, sizeof(dla));
  dla.runtime_ns = q;
  dla.deadline_ns = 40 * ms;
  dla.period_ns = 40 * ms;
  dla.abs_deadline = 40 * ms;
  dla.remaining_ns = 0;
  deadline_replenish_math(&dla, 40 * ms + 1);
  if (dla.abs_deadline != 80 * ms + 1 || dla.remaining_ns != q)
    {
      return time_fail(6);
    }

  memset(&dla, 0, sizeof(dla));
  dla.runtime_ns = q;
  dla.period_ns = 40 * ms;
  dla.abs_deadline = 40 * ms;
  dla.remaining_ns = 7;
  deadline_replenish_math(&dla, 40 * ms - 1);
  if (dla.abs_deadline != 40 * ms || dla.remaining_ns != 7)
    {
      return time_fail(6);
    }

  memset(&dla, 0, sizeof(dla));
  dla.runtime_ns = q;
  dla.deadline_ns = 40 * ms;
  dla.period_ns = 40 * ms;
  dla.abs_deadline = 40 * ms;
  deadline_replenish_math(&dla, 80 * ms);
  if (dla.abs_deadline != 120 * ms || dla.remaining_ns != q)
    {
      return time_fail(6);
    }

  memset(&dla, 0, sizeof(dla));
  dla.runtime_ns = q;
  dla.period_ns = 40 * ms;
  dla.abs_deadline = 40 * ms;
  dla.remaining_ns = 8 * ms;
  deadline_replenish_math(&dla, 80 * ms);
  if (dla.abs_deadline != 120 * ms || dla.remaining_ns != q)
    {
      return time_fail(6);
    }

  memset(&dla, 0, sizeof(dla));
  dla.runtime_ns = q;
  dla.period_ns = 40 * ms;
  dla.abs_deadline = 40 * ms;
  deadline_replenish_math(&dla, 40 * ms + 10 * 40 * ms);
  if (dla.abs_deadline != 40 * ms + 11 * 40 * ms ||
      dla.remaining_ns != q)
    {
      return time_fail(6);
    }

  /* D < P. Span is P, not D. now+D with a full Q would exceed Q/P. */

  memset(&dla, 0, sizeof(dla));
  dla.runtime_ns = q;
  dla.deadline_ns = 20 * ms;
  dla.period_ns = 40 * ms;
  dla.abs_deadline = 20 * ms;
  dla.remaining_ns = 0;
  deadline_replenish_math(&dla, 125 * ms);
  if (dla.abs_deadline != 165 * ms || dla.remaining_ns != q)
    {
      return time_fail(6);
    }

  memset(&dla, 0, sizeof(dla));
  dla.runtime_ns = q;
  dla.period_ns = 40 * ms;
  dla.abs_deadline = 0;
  dla.remaining_ns = 0;
  dla.sched = DL_SCHED_THROTTLED;
  deadline_replenish_math(&dla, UINT64_MAX - 10);
  if (dla.abs_deadline != 0 || dla.remaining_ns != 0 ||
      dla.sched != DL_SCHED_THROTTLED)
    {
      return time_fail(6);
    }

  if (NSEC2TICK(0) != 0 || NSEC2TICK(1) != 1 ||
      (uint64_t)NSEC2TICK(NSEC_PER_TICK) != 1 ||
      (uint64_t)NSEC2TICK(NSEC_PER_TICK + 1) != 2 ||
      time_roundup(1000000, 1000000) != 1 ||
      time_roundup(1000001, 1000000) != 2 ||
      time_roundup(999999, 1000000) != 1 ||
      time_roundup(10000000, 10000000) != 1 ||
      time_roundup(10000001, 10000000) != 2)
    {
      return time_fail(6);
    }

  printf("deadline time: ok T6 abs=%llu remaining=%llu tick_ns=%ld\n",
         165ull * ms, (unsigned long long)q, (long)NSEC_PER_TICK);

  /* T7. Wake rule on both sides of the inequality, plus a wide product. */

  memset(&a, 0, sizeof(a));
  memset(&dla, 0, sizeof(dla));
  a.deadline = &dla;
  a.flags = TCB_FLAG_SCHED_DEADLINE;
  cbs_params(&dla, 10, 1000, 1000);
  dla.sched = DL_SCHED_BLOCKED;
  dla.abs_deadline = 2000;
  dla.remaining_ns = 10;
  deadline_woken_at(&a, 1000);
  if (dla.abs_deadline != 2000 || dla.remaining_ns != 10 ||
      dla.sched != DL_SCHED_ELIGIBLE)
    {
      return time_fail(7);
    }

  dla.sched = DL_SCHED_BLOCKED;
  dla.abs_deadline = 1500;
  dla.remaining_ns = 11;
  deadline_woken_at(&a, 1000);
  if (dla.abs_deadline != 2000 || dla.remaining_ns != 10)
    {
      return time_fail(7);
    }

  dla.sched = DL_SCHED_BLOCKED;
  dla.abs_deadline = 1000;
  dla.remaining_ns = 5;
  deadline_woken_at(&a, 1000);
  if (dla.abs_deadline != 2000 || dla.remaining_ns != 10)
    {
      return time_fail(7);
    }

  memset(&dla, 0, sizeof(dla));
  dla.sched = DL_SCHED_BLOCKED;
  dla.flags = 0;
  dla.runtime_ns = 1;
  dla.deadline_ns = 50;
  dla.period_ns = 0x100000000ull;
  dla.abs_deadline = 1001;
  dla.remaining_ns = 0x100000000ull;
  deadline_woken_at(&a, 1000);
  if (dla.abs_deadline != 1050 || dla.remaining_ns != 1)
    {
      return time_fail(7);
    }

  printf("deadline time: ok T7 preserved_and_reset\n");

  /* T8. Stop twice. Stale callback does not revive the task.
   * A reserved object is charged once and released once.
   */

  memset(&a, 0, sizeof(a));
  memset(&dla, 0, sizeof(dla));
  a.deadline = &dla;
  a.flags = TCB_FLAG_SCHED_DEADLINE;
  cbs_params(&dla, q, 40 * ms, 40 * ms);
  dla.sched = DL_SCHED_ELIGIBLE;
  dla.flags = DEADLINE_FLAG_KEY_VALID;
  dla.abs_deadline = 60ull * 1000 * ms;
  dla.remaining_ns = q;
  deadline_arm(&a);
  gen = dla.generation;
  nxsched_deadline_stop(&a);
  if (WDOG_ISACTIVE(&dla.replenish) || dla.sched != DL_SCHED_STOPPED ||
      dla.generation != gen + 1 || a.deadline != &dla)
    {
      return time_fail(8);
    }

  gen = dla.generation;
  left = dla.remaining_ns;
  nxsched_deadline_stop(&a);
  deadline_replenish_cb((wdparm_t)&a);
  if (dla.generation != gen || dla.sched != DL_SCHED_STOPPED ||
      dla.remaining_ns != left || WDOG_ISACTIVE(&dla.replenish))
    {
      return time_fail(8);
    }

  ppm = nxsched_deadline_admitted_ppm();
  memset(&a, 0, sizeof(a));
  a.pid = -1;
  a.task_state = TSTATE_TASK_INVALID;
  if (nxsched_deadline_reserve(&a, q, 40 * ms, 40 * ms) != OK ||
      nxsched_deadline_admitted_ppm() != ppm + 250000u)
    {
      nxsched_deadline_release(&a);
      return time_fail(8);
    }

  nxsched_deadline_stop(&a);
  deadline_replenish_cb((wdparm_t)&a);
  nxsched_deadline_stop(&a);
  if (a.deadline != NULL || nxsched_deadline_admitted_ppm() != ppm)
    {
      return time_fail(8);
    }

  printf("deadline time: ok T8 admitted_ppm=%lu\n", (unsigned long)ppm);

  /* T9. Switch charges `from`, not `to`. The IRQ choice of
   * running_task() versus g_readytorun.head is not entered here.
   */

  time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
  time_open(&b, &dlb, q, 40 * ms, 40 * ms, 0);
  a.pid = 71;
  b.pid = 72;
  deadline_switch_at(&a, &b, 4 * ms);
  if (dla.remaining_ns != q - 4 * ms || dlb.remaining_ns != q ||
      (dla.flags & DEADLINE_FLAG_EXECUTING) != 0 ||
      (dlb.flags & DEADLINE_FLAG_EXECUTING) == 0 ||
      dlb.accounted_ns != 4 * ms)
    {
      return time_fail(9);
    }

  deadline_tick_at(&a, false, 9 * ms);
  if (dla.remaining_ns != q - 4 * ms || dlb.remaining_ns != q)
    {
      return time_fail(9);
    }

  printf("deadline time: ok T9 from_charged=%llu to_uncharged\n",
         (unsigned long long)(4 * ms));
  printf("deadline time: UNVERIFIED T9 irq running_task versus ready head\n");

  /* T11. Debt is repaid once. These states are the SPEC.md ledger,
   * not a second copy of deadline_pay_grant.
   */

  {
    uint32_t ppm_before = nxsched_deadline_admitted_ppm();
    struct tcb_s fresh;

    time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
    deadline_account(&a, 10 * ms);
    if (dla.remaining_ns != 0 || dla.overrun_ns != 0 ||
        dla.sched != DL_SCHED_THROTTLED)
      {
        printf("deadline time: FAIL T11 baseline exhaust\n");
        return 11;
      }

    deadline_replenish_apply(&a, 40 * ms);
    if (dla.remaining_ns != q || dla.overrun_ns != 0 ||
        dla.abs_deadline != 80 * ms || dla.sched != DL_SCHED_ELIGIBLE)
      {
        printf("deadline time: FAIL T11 baseline grant\n");
        return 11;
      }

    time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
    a.lockcount = 1;
    deadline_account(&a, 13 * ms);
    deadline_account(&a, 13 * ms);
    if (dla.remaining_ns != 0 || dla.overrun_ns != 3 * ms ||
        dla.sched != DL_SCHED_ELIGIBLE ||
        (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) == 0)
      {
        printf("deadline time: FAIL T11 locked 13\n");
        return 11;
      }

    a.lockcount = 0;
    if (nxsched_deadline_release_lock(&a) ||
        dla.sched != DL_SCHED_THROTTLED || dla.overrun_ns != 3 * ms ||
        dla.remaining_ns != 0 || nxsched_deadline_eligible(&a))
      {
        printf("deadline time: FAIL T11 unlock 13\n");
        return 11;
      }

    deadline_replenish_apply(&a, 40 * ms);
    if (dla.remaining_ns != 7 * ms || dla.overrun_ns != 0 ||
        dla.abs_deadline != 80 * ms ||
        !nxsched_deadline_eligible(&a))
      {
        printf("deadline time: FAIL T11 grant 7\n");
        return 11;
      }

    deadline_switch_at(NULL, &a, 40 * ms);
    deadline_account(&a, 47 * ms);
    if (dla.remaining_ns != 0 || dla.overrun_ns != 0 ||
        dla.sched != DL_SCHED_THROTTLED || dla.abs_deadline != 80 * ms)
      {
        printf("deadline time: FAIL T11 spend 7\n");
        return 11;
      }

    time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
    a.lockcount = 1;
    deadline_account(&a, 26 * ms);
    a.lockcount = 0;
    nxsched_deadline_release_lock(&a);
    deadline_replenish_apply(&a, 40 * ms);
    if (dla.remaining_ns != 0 || dla.overrun_ns != 16 * ms - q ||
        dla.abs_deadline != 80 * ms || dla.sched != DL_SCHED_THROTTLED ||
        nxsched_deadline_eligible(&a))
      {
        printf("deadline time: FAIL T11 grant 26\n");
        return 11;
      }

    deadline_replenish_apply(&a, 40 * ms);
    if (dla.remaining_ns != 0 || dla.overrun_ns != 6 * ms ||
        dla.abs_deadline != 80 * ms || dla.replenish_count != 2)
      {
        printf("deadline time: FAIL T11 same stamp 26\n");
        return 11;
      }

    deadline_replenish_apply(&a, 80 * ms);
    if (dla.remaining_ns != 4 * ms || dla.overrun_ns != 0 ||
        dla.abs_deadline != 120 * ms ||
        !nxsched_deadline_eligible(&a))
      {
        printf("deadline time: FAIL T11 retire 26\n");
        return 11;
      }

    time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
    a.lockcount = 1;
    deadline_account(&a, 3 * ms);
    a.lockcount = 0;
    if (dla.remaining_ns != 7 * ms || dla.overrun_ns != 0 ||
        nxsched_deadline_release_lock(&a) ||
        dla.sched != DL_SCHED_ELIGIBLE)
      {
        printf("deadline time: FAIL T11 locked 3\n");
        return 11;
      }

    time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
    a.lockcount = 1;
    deadline_account(&a, q + 1);
    a.lockcount = 0;
    nxsched_deadline_release_lock(&a);
    deadline_replenish_apply(&a, 40 * ms);
    if (dla.overrun_ns != 0 || dla.remaining_ns != q - 1 ||
        dla.abs_deadline != 80 * ms)
      {
        printf("deadline time: FAIL T11 1ns\n");
        return 11;
      }

    memset(&dla, 0, sizeof(dla));
    dla.runtime_ns = q;
    dla.period_ns = 40 * ms;
    dla.deadline_ns = 40 * ms;
    dla.abs_deadline = 40 * ms;
    dla.overrun_ns = 3 * ms;
    dla.sched = DL_SCHED_THROTTLED;
    deadline_replenish_math(&dla, 125 * ms);
    if (dla.abs_deadline != 165 * ms || dla.remaining_ns != 7 * ms ||
        dla.overrun_ns != 0)
      {
        printf("deadline time: FAIL T11 late debt 3\n");
        return 11;
      }

    deadline_replenish_math(&dla, 125 * ms);
    if (dla.abs_deadline != 165 * ms || dla.remaining_ns != 7 * ms ||
        dla.overrun_ns != 0)
      {
        printf("deadline time: FAIL T11 late second\n");
        return 11;
      }

    memset(&dla, 0, sizeof(dla));
    dla.runtime_ns = q;
    dla.period_ns = 40 * ms;
    dla.abs_deadline = 0;
    dla.remaining_ns = 4 * ms;
    dla.overrun_ns = 5 * ms;
    deadline_replenish_math(&dla, UINT64_MAX - 10);
    if (dla.abs_deadline != 0 || dla.remaining_ns != 4 * ms ||
        dla.overrun_ns != 5 * ms)
      {
        printf("deadline time: FAIL T11 overflow keeps debt\n");
        return 11;
      }

    memset(&dla, 0, sizeof(dla));
    dla.runtime_ns = q;
    dla.period_ns = 40 * ms;
    dla.abs_deadline = 40 * ms;
    dla.overrun_ns = UINT64_MAX;
    dla.sched = DL_SCHED_THROTTLED;
    deadline_replenish_math(&dla, 40 * ms);
    if (dla.remaining_ns != 0 || dla.overrun_ns != UINT64_MAX - q ||
        dla.abs_deadline != 80 * ms)
      {
        printf("deadline time: FAIL T11 saturating debt\n");
        return 11;
      }

    deadline_replenish_math(&dla, 40 * ms);
    if (dla.overrun_ns != UINT64_MAX - q || dla.remaining_ns != 0)
      {
        printf("deadline time: FAIL T11 saturating second\n");
        return 11;
      }

    time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
    a.lockcount = 1;
    deadline_account(&a, 13 * ms);
    deadline_blocked_at(&a, 13 * ms);
    if (dla.overrun_ns != 3 * ms || dla.remaining_ns != 0 ||
        dla.sched != DL_SCHED_BLOCKED)
      {
        printf("deadline time: FAIL T11 block keeps debt\n");
        return 11;
      }

    deadline_woken_at(&a, 20 * ms);
    if (dla.overrun_ns != 3 * ms || dla.remaining_ns != 0 ||
        dla.sched != DL_SCHED_THROTTLED)
      {
        printf("deadline time: FAIL T11 wake keeps debt\n");
        return 11;
      }

    time_open(&a, &dla, q, 40 * ms, 40 * ms, 0);
    dla.overrun_ns = 9 * ms;
    nxsched_deadline_stop(&a);
    deadline_replenish_apply(&a, 80 * ms);
    if (dla.sched != DL_SCHED_STOPPED || dla.overrun_ns != 9 * ms ||
        dla.remaining_ns != q)
      {
        printf("deadline time: FAIL T11 stop\n");
        return 11;
      }

    memset(&fresh, 0, sizeof(fresh));
    fresh.pid = -1;
    fresh.task_state = TSTATE_TASK_INVALID;
    if (nxsched_deadline_reserve(&fresh, q, 40 * ms, 40 * ms) != OK)
      {
        printf("deadline time: FAIL T11 reserve\n");
        return 11;
      }

    fresh.deadline->overrun_ns = 9 * ms;
    nxsched_deadline_stop(&fresh);
    if (fresh.deadline != NULL ||
        nxsched_deadline_reserve(&fresh, q, 40 * ms, 40 * ms) != OK ||
        fresh.deadline->overrun_ns != 0)
      {
        nxsched_deadline_stop(&fresh);
        printf("deadline time: FAIL T11 restart\n");
        return 11;
      }

    nxsched_deadline_stop(&fresh);
    if (nxsched_deadline_admitted_ppm() != ppm_before)
      {
        printf("deadline time: FAIL T11 ppm\n");
        return 11;
      }

    printf("deadline time: ok T11 debt\n");
  }

  printf("deadline time: PASS T1-T8 synthetic; T9 switch only\n");
  return 0;
}

static int o_execbit(FAR const struct deadline_s *dl)
{
  return (dl->flags & DEADLINE_FLAG_EXECUTING) != 0;
}

static void o_line(int sid, int step, int head,
                   uint64_t r0, unsigned s0, int e0,
                   uint64_t r1, unsigned s1, int e1)
{
  printf("OBS sid=%d step=%d head=%d r0=%llu s0=%u e0=%d "
         "r1=%llu s1=%u e1=%d\n",
         sid, step, head,
         (unsigned long long)r0, s0, e0,
         (unsigned long long)r1, s1, e1);
}

static void o_make(FAR struct tcb_s *tcb, FAR struct deadline_s *dl,
                   int pid, uint64_t q, uint64_t d, uint64_t p,
                   uint64_t abs, uint64_t seq)
{
  memset(tcb, 0, sizeof(*tcb));
  memset(dl, 0, sizeof(*dl));
  tcb->deadline = dl;
  tcb->flags = TCB_FLAG_SCHED_DEADLINE;
  tcb->pid = pid;
  cbs_params(dl, q, d, p);
  dl->sched = DL_SCHED_ELIGIBLE;
  dl->flags = DEADLINE_FLAG_KEY_VALID;
  dl->remaining_ns = q;
  dl->abs_deadline = abs;
  dl->runnable_seq = seq;
}

static int o_head(FAR const dq_queue_t *q)
{
  FAR const struct tcb_s *tcb = (FAR const struct tcb_s *)q->head;

  return (tcb != NULL) ? (int)tcb->pid : -1;
}

static void o_drop(FAR dq_queue_t *q, FAR struct tcb_s *tcb)
{
  if (tcb->flink != NULL || tcb->blink != NULL ||
      q->head == (FAR dq_entry_t *)tcb)
    {
      dq_rem((FAR dq_entry_t *)tcb, q);
      tcb->flink = NULL;
      tcb->blink = NULL;
    }
}

/* Synthetic tasks are not SCHEDULABLE, so the production detach does
 * not receive this queue. The unlink is the same dq_rem the live
 * ready-list removal performs. The debit itself is deadline_switch_at.
 */

static void o_follow(FAR dq_queue_t *q, FAR struct tcb_s *prev,
                     uint64_t now)
{
  FAR struct tcb_s *head = (FAR struct tcb_s *)q->head;

  if (head != NULL && head != prev &&
      nxsched_deadline_eligible(head) &&
      (head->deadline->flags & DEADLINE_FLAG_EXECUTING) == 0)
    {
      deadline_switch_at(prev, head, now);
    }
}

int sched_deadline_oracle_test(void)
{
  struct tcb_s a;
  struct tcb_s b;
  struct tcb_s c;
  struct tcb_s d;
  struct deadline_s dla;
  struct deadline_s dlb;
  struct deadline_s dlc;
  dq_queue_t q;
  const uint64_t ms = 1000000ull;
  uint32_t ppm0;
  int rc;

  /* 1. Earlier absolute deadline is dispatched and preempts on wake. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  o_make(&b, &dlb, 2, 10 * ms, 50 * ms, 50 * ms, 50 * ms, 2);
  dq_init(&q);
  deadline_switch_at(NULL, &a, 0);
  nxsched_add_runnable(&a, &q);
  nxsched_add_runnable(&b, &q);
  o_line(1, 0, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));
  deadline_account(&a, 5 * ms);
  o_line(1, 1, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));
  deadline_blocked_at(&a, 5 * ms);
  o_drop(&q, &a);
  o_follow(&q, &a, 5 * ms);
  o_line(1, 2, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));
  deadline_woken_at(&a, 6 * ms);
  nxsched_add_runnable(&a, &q);
  deadline_switch_at(&b, &a, 6 * ms);
  o_line(1, 3, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));

  /* 2. Equal absolute deadlines keep the earlier runnable_seq. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  o_make(&b, &dlb, 2, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 2);
  dq_init(&q);
  nxsched_add_runnable(&a, &q);
  nxsched_add_runnable(&b, &q);
  o_line(2, 0, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));

  /* 3. Tick and switch at the same stamp debit once. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  o_make(&b, &dlb, 2, 10 * ms, 50 * ms, 50 * ms, 50 * ms, 2);
  deadline_switch_at(NULL, &a, 0);
  deadline_account(&a, 1000);
  deadline_switch_at(&a, &b, 1000);
  o_line(3, 0, 2, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));

  /* 4. Exhaustion leaves the task unselectable until one replenishment. */

  o_make(&a, &dla, 1, 1000, 1000, 1000, 1000, 1);
  deadline_switch_at(NULL, &a, 0);
  deadline_account(&a, 1000);
  dq_init(&q);
  rc = nxsched_add_runnable(&a, &q) ? 1 : 0;
  o_line(4, 0, rc == 0 ? -1 : o_head(&q), dla.remaining_ns, dla.sched,
         o_execbit(&dla), (uint64_t)dla.replenish_count, rc, 0);
  deadline_replenish_apply(&a, 1000);
  dq_init(&q);
  rc = nxsched_add_runnable(&a, &q) ? 1 : 0;
  o_line(4, 1, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.replenish_count, rc);
  deadline_replenish_apply(&a, 1000);
  o_line(4, 2, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.replenish_count, rc);

  /* 5. Wake while throttled stays throttled. Replenish of a user wait
   * does not insert. A later wake does not grant a second budget.
   */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  dla.sched = DL_SCHED_THROTTLED;
  dla.remaining_ns = 0;
  dla.flags = DEADLINE_FLAG_KEY_VALID;
  a.task_state = TSTATE_WAIT_SEM;
  deadline_woken_at(&a, 1000);
  o_line(5, 0, -1, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, 0, nxsched_deadline_eligible(&a));
  deadline_replenish_apply(&a, 40 * ms);
  dq_init(&q);
  o_line(5, 1, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.replenish_count,
         nxsched_deadline_eligible(&a));
  deadline_woken_at(&a, 40 * ms);
  nxsched_add_runnable(&a, &q);
  o_line(5, 2, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.replenish_count,
         nxsched_deadline_eligible(&a));

  /* 6. A late replenishment skips missed periods once. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  dla.sched = DL_SCHED_THROTTLED;
  dla.remaining_ns = 0;
  dla.flags = DEADLINE_FLAG_KEY_VALID;
  deadline_replenish_apply(&a, 125 * ms);
  o_line(6, 0, -1, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.replenish_count, 0);
  deadline_replenish_apply(&a, 125 * ms);
  o_line(6, 1, -1, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.replenish_count, 0);

  /* 7. sched_lock defers the throttle. Unlock enforces it once. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  deadline_switch_at(NULL, &a, 0);
  a.lockcount = 1;
  deadline_account(&a, 13 * ms);
  o_line(7, 0, 1, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u, 0);
  deadline_account(&a, 13 * ms);
  o_line(7, 1, 1, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u, 0);
  a.lockcount = 0;
  rc = nxsched_deadline_release_lock(&a) ? 1 : 0;
  o_line(7, 2, rc, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u,
         nxsched_deadline_eligible(&a));
  rc = nxsched_deadline_release_lock(&a) ? 1 : 0;
  o_line(7, 3, rc, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u,
         nxsched_deadline_eligible(&a));

  /* 8. Eligible deadline precedes FIFO and sporadic. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  memset(&b, 0, sizeof(b));
  b.pid = 8;
  b.flags = TCB_FLAG_SCHED_FIFO;
  b.sched_priority = 255;
  memset(&c, 0, sizeof(c));
  c.pid = 9;
  c.flags = TCB_FLAG_SCHED_SPORADIC;
  c.sched_priority = 255;
  o_make(&d, &dlc, 10, 10 * ms, 40 * ms, 40 * ms, 10 * ms, 4);
  dlc.sched = DL_SCHED_THROTTLED;
  dlc.remaining_ns = 0;
  dq_init(&q);
  nxsched_add_runnable(&b, &q);
  nxsched_add_runnable(&c, &q);
  nxsched_add_runnable(&a, &q);
  rc = nxsched_add_runnable(&d, &q) ? 1 : 0;
  o_line(8, 0, o_head(&q), dla.remaining_ns, dla.sched, rc,
         0, 0, nxsched_deadline_eligible(&a));

  /* 9. Admission at, below, and above 950000 ppm, released once. */

  ppm0 = nxsched_deadline_admitted_ppm();
  memset(&a, 0, sizeof(a));
  a.pid = -1;
  a.task_state = TSTATE_TASK_INVALID;
  rc = nxsched_deadline_reserve(&a, 950000, 1000000, 1000000);
  printf("OBS sid=9 step=0 rc=%d ppm=%lu\n", rc,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));
  nxsched_deadline_release(&a);
  printf("OBS sid=9 step=1 rc=%d ppm=%lu\n", 0,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));
  memset(&a, 0, sizeof(a));
  a.pid = -1;
  a.task_state = TSTATE_TASK_INVALID;
  rc = nxsched_deadline_reserve(&a, 950001, 1000000, 1000000);
  printf("OBS sid=9 step=2 rc=%d ppm=%lu\n", rc,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));
  memset(&b, 0, sizeof(b));
  b.pid = -1;
  b.task_state = TSTATE_TASK_INVALID;
  rc = nxsched_deadline_reserve(&b, 10 * ms, 40 * ms, 40 * ms);
  printf("OBS sid=9 step=3 rc=%d ppm=%lu\n", rc,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));
  nxsched_deadline_release(&b);
  nxsched_deadline_release(&b);
  printf("OBS sid=9 step=4 rc=%d ppm=%lu\n", 0,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));

  /* 10. Two preemptions and one replenishment. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  o_make(&b, &dlb, 2, 10 * ms, 60 * ms, 60 * ms, 60 * ms, 2);
  dq_init(&q);
  deadline_switch_at(NULL, &a, 0);
  nxsched_add_runnable(&a, &q);
  nxsched_add_runnable(&b, &q);
  o_line(10, 0, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));
  deadline_account(&a, 4 * ms);
  o_line(10, 1, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));
  deadline_blocked_at(&a, 4 * ms);
  o_drop(&q, &a);
  o_follow(&q, &a, 4 * ms);
  o_line(10, 2, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));
  deadline_woken_at(&a, 4 * ms);
  nxsched_add_runnable(&a, &q);
  deadline_switch_at(&b, &a, 4 * ms);
  o_line(10, 3, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));
  deadline_account(&a, 10 * ms);
  if (!nxsched_deadline_eligible(&a))
    {
      o_drop(&q, &a);
      o_follow(&q, &a, 10 * ms);
    }

  o_line(10, 4, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));
  deadline_replenish_apply(&a, 40 * ms);
  nxsched_add_runnable(&a, &q);
  o_line(10, 5, o_head(&q), dla.remaining_ns, dla.sched, o_execbit(&dla),
         dlb.remaining_ns, dlb.sched, o_execbit(&dlb));

  /* 11. Locked 13 ms. The next post repays 3 ms and leaves 7 ms. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  deadline_switch_at(NULL, &a, 0);
  a.lockcount = 1;
  deadline_account(&a, 13 * ms);
  o_line(11, 0, 1, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u,
         nxsched_deadline_eligible(&a));
  deadline_account(&a, 13 * ms);
  o_line(11, 1, 1, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u,
         nxsched_deadline_eligible(&a));
  a.lockcount = 0;
  rc = nxsched_deadline_release_lock(&a) ? 1 : 0;
  o_line(11, 2, rc, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u,
         nxsched_deadline_eligible(&a));
  deadline_replenish_apply(&a, 40 * ms);
  o_line(11, 3, nxsched_deadline_eligible(&a) ? 1 : 0,
         dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.overrun_ns, 0);
  deadline_switch_at(NULL, &a, 40 * ms);
  deadline_account(&a, 47 * ms);
  o_line(11, 4, nxsched_deadline_eligible(&a) ? 1 : 0,
         dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.overrun_ns, 0);

  /* 12. Locked 26 ms. One post cannot clear 16 ms of debt. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  deadline_switch_at(NULL, &a, 0);
  a.lockcount = 1;
  deadline_account(&a, 26 * ms);
  o_line(12, 0, 1, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u,
         nxsched_deadline_eligible(&a));
  a.lockcount = 0;
  rc = nxsched_deadline_release_lock(&a) ? 1 : 0;
  o_line(12, 1, rc, dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.overrun_ns,
         (dla.flags & DEADLINE_FLAG_THROTTLE_PENDING) ? 1u : 0u,
         nxsched_deadline_eligible(&a));
  deadline_replenish_apply(&a, 40 * ms);
  o_line(12, 2, nxsched_deadline_eligible(&a) ? 1 : 0,
         dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.overrun_ns, 0);
  deadline_replenish_apply(&a, 40 * ms);
  o_line(12, 3, (int)dla.replenish_count, dla.remaining_ns, dla.sched,
         o_execbit(&dla), dla.abs_deadline, (unsigned)dla.overrun_ns,
         nxsched_deadline_eligible(&a) ? 1 : 0);
  deadline_replenish_apply(&a, 80 * ms);
  o_line(12, 4, nxsched_deadline_eligible(&a) ? 1 : 0,
         dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.overrun_ns,
         nxsched_deadline_eligible(&a) ? 1 : 0);

  /* 13. D < P is refused. A rejected update keeps the ledger. */

  ppm0 = nxsched_deadline_admitted_ppm();
  memset(&a, 0, sizeof(a));
  a.pid = -1;
  a.task_state = TSTATE_TASK_INVALID;
  rc = nxsched_deadline_reserve(&a, 10 * ms, 20 * ms, 40 * ms);
  printf("OBS sid=13 step=0 rc=%d ppm=%lu\n", rc,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));
  rc = nxsched_deadline_reserve(&a, 10 * ms, 40 * ms, 40 * ms);
  printf("OBS sid=13 step=1 rc=%d ppm=%lu\n", rc,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));
  rc = nxsched_deadline_reserve(&a, 10 * ms, 20 * ms, 40 * ms);
  printf("OBS sid=13 step=2 rc=%d ppm=%lu\n", rc,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));
  printf("OBS sid=13 step=3 rc=%d ppm=%lu\n",
         (a.deadline != NULL && a.deadline->runtime_ns == 10 * ms &&
          a.deadline->deadline_ns == 40 * ms &&
          a.deadline->period_ns == 40 * ms) ? 0 : -1,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));
  nxsched_deadline_release(&a);
  printf("OBS sid=13 step=4 rc=%d ppm=%lu\n", 0,
         (unsigned long)(nxsched_deadline_admitted_ppm() - ppm0));

  /* 14. A late post still uses now+P, and it repays debt once. */

  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  dla.sched = DL_SCHED_THROTTLED;
  dla.remaining_ns = 0;
  dla.overrun_ns = 3 * ms;
  dla.flags = DEADLINE_FLAG_KEY_VALID;
  deadline_replenish_apply(&a, 125 * ms);
  o_line(14, 0, nxsched_deadline_eligible(&a) ? 1 : 0,
         dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.overrun_ns,
         (int)dla.replenish_count);
  deadline_replenish_apply(&a, 125 * ms);
  o_line(14, 1, nxsched_deadline_eligible(&a) ? 1 : 0,
         dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.overrun_ns,
         (int)dla.replenish_count);
  o_make(&a, &dla, 1, 10 * ms, 40 * ms, 40 * ms, 40 * ms, 1);
  dla.sched = DL_SCHED_THROTTLED;
  dla.remaining_ns = 0;
  dla.overrun_ns = 16 * ms;
  dla.flags = DEADLINE_FLAG_KEY_VALID;
  deadline_replenish_apply(&a, 125 * ms);
  o_line(14, 2, nxsched_deadline_eligible(&a) ? 1 : 0,
         dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.overrun_ns,
         (int)dla.replenish_count);
  deadline_replenish_apply(&a, 125 * ms);
  o_line(14, 3, nxsched_deadline_eligible(&a) ? 1 : 0,
         dla.remaining_ns, dla.sched, o_execbit(&dla),
         dla.abs_deadline, (unsigned)dla.overrun_ns,
         (int)dla.replenish_count);

  printf("ORACLE_DONE\n");
  return 0;
}

static void t9_wd_cb(wdparm_t arg)
{
  sem_post((FAR sem_t *)arg);
}

void sched_deadline_t9_expect(pid_t low, pid_t high)
{
  memset(&g_t9_wit, 0, sizeof(g_t9_wit));
  g_t9_low = low;
  g_t9_high = high;
  g_t9_mark = 0;
  g_t9_arm = 1;
}

int sched_deadline_t9_kick(FAR void *sem)
{
  return wd_start(&g_t9_wd, 1, t9_wd_cb, (wdparm_t)sem);
}

void sched_deadline_t9_read(FAR struct sched_deadline_t9_s *out)
{
  wd_cancel(&g_t9_wd);
  g_t9_arm = 0;
  *out = g_t9_wit;
}

#endif
