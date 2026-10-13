/****************************************************************************
 * sched/sched/sched_deadline_admit.c
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

#include <errno.h>
#include <stdint.h>
#include <string.h>

#ifdef CONFIG_TESTING_DEADLINE
#include <stdio.h>
#endif

#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Parts per million of one CPU. 950000 is exactly 95 percent. */

#define DL_BW_SCALE 1000000u
#define DL_BW_CAP   950000u

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_dl_admitted;

#ifdef CONFIG_TESTING_DEADLINE
static int g_admit_fail_before_commit;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* floor((hi << 64 | lo) / den). Caller guarantees hi < den and den != 0.
 * The remainder stays below den, so a shift needs one extra bit. When
 * that bit is set, unsigned wrap of rem - den is the mathematical
 * remainder. Checked against arbitrary-precision division for random
 * pairs with 0 < runtime <= period.
 */

static uint64_t deadline_div_u128(uint64_t hi, uint64_t lo, uint64_t den,
                                 FAR uint64_t *rem_out)
{
  uint64_t rem = hi;
  uint64_t quot = 0;
  int i;

  DEBUGASSERT(den != 0 && hi < den);

  for (i = 63; i >= 0; i--)
    {
      uint64_t carry = rem >> 63;

      rem = (rem << 1) | ((lo >> i) & 1ull);
      quot <<= 1;
      if (carry != 0 || rem >= den)
        {
          rem -= den;
          quot |= 1ull;
        }
    }

  if (rem_out != NULL)
    {
      *rem_out = rem;
    }

  return quot;
}

/* scale * value as a 128-bit product. scale fits in 32 bits. */

static void deadline_mul_u64_u32(uint64_t value, uint32_t scale,
                                FAR uint64_t *hi, FAR uint64_t *lo)
{
  uint64_t v0 = (uint32_t)value;
  uint64_t v1 = value >> 32;
  uint64_t p0 = v0 * scale;
  uint64_t p1 = v1 * scale;
  uint64_t mid = (p0 >> 32) + (uint32_t)p1;

  *lo = (p0 & 0xffffffffull) | (mid << 32);
  *hi = (p1 >> 32) + (mid >> 32);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_deadline_charge_ppm
 *
 * Description:
 *   ceil(1000000 * runtime / period) for 0 < runtime <= period.
 *   The result is in 1..1000000. It does not touch the ledger.
 *
 ****************************************************************************/

int nxsched_deadline_charge_ppm(uint64_t runtime_ns, uint64_t period_ns,
                               FAR uint32_t *charge)
{
  uint64_t hi;
  uint64_t lo;
  uint64_t rem;
  uint64_t quot;

  if (charge == NULL || runtime_ns == 0 || period_ns == 0 ||
      runtime_ns > period_ns)
    {
      return -EINVAL;
    }

  deadline_mul_u64_u32(runtime_ns, DL_BW_SCALE, &hi, &lo);
  if (hi >= period_ns)
    {
      return -EINVAL;
    }

  quot = deadline_div_u128(hi, lo, period_ns, &rem);
  if (rem != 0)
    {
      quot++;
    }

  if (quot == 0 || quot > DL_BW_SCALE)
    {
      return -EINVAL;
    }

  *charge = (uint32_t)quot;
  return OK;
}

/****************************************************************************
 * Name: nxsched_deadline_admitted_ppm
 *
 * Description:
 *   Current sum of reservation charges. Not a syscall.
 *
 ****************************************************************************/

uint32_t nxsched_deadline_admitted_ppm(void)
{
  irqstate_t flags = enter_critical_section();
  uint32_t total = g_dl_admitted;

  leave_critical_section(flags);
  return total;
}

#ifdef CONFIG_TESTING_DEADLINE
void nxsched_deadline_admit_fail_next(void)
{
  g_admit_fail_before_commit = 1;
}
#endif

/****************************************************************************
 * Name: nxsched_deadline_reserve
 *
 * Description:
 *   Charge a new reservation, or replace the charge of the reservation
 *   this TCB already owns. Does not set the deadline policy, does not
 *   set DEADLINE_FLAG_SCHEDULABLE, and does not arm a watchdog.
 *   Supported parameters are 0 < runtime <= deadline and
 *   deadline == period. A different deadline is -EINVAL and does not
 *   publish. -EINVAL also covers a bad argument or an object that is
 *   not owned by this TCB.
 *   -EBUSY   the rounded charge would pass 950000 ppm.
 *   -ENOMEM  the new object could not be allocated.
 *   -ESRCH   the TCB is exiting, or the reservation is already stopped.
 *   -EIO     test-only failure after allocation and before commit.
 *
 ****************************************************************************/

int nxsched_deadline_reserve(FAR struct tcb_s *tcb, uint64_t runtime_ns,
                            uint64_t deadline_ns, uint64_t period_ns)
{
  FAR struct deadline_s *fresh = NULL;
  irqstate_t flags;
  uint32_t charge;
  int ret;

  if (tcb == NULL || runtime_ns == 0 || deadline_ns == 0 ||
      period_ns == 0 || runtime_ns > deadline_ns ||
      deadline_ns != period_ns)
    {
      return -EINVAL;
    }

  ret = nxsched_deadline_charge_ppm(runtime_ns, period_ns, &charge);
  if (ret < 0)
    {
      return ret;
    }

  if ((tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      return -ESRCH;
    }

  if (tcb->deadline == NULL)
    {
      fresh = kmm_zalloc(sizeof(struct deadline_s));
      if (fresh == NULL)
        {
          return -ENOMEM;
        }

#ifdef CONFIG_TESTING_DEADLINE
      if (g_admit_fail_before_commit != 0)
        {
          g_admit_fail_before_commit = 0;
          kmm_free(fresh);
          return -EIO;
        }
#endif

      fresh->runtime_ns = runtime_ns;
      fresh->deadline_ns = deadline_ns;
      fresh->period_ns = period_ns;
      fresh->charge_ppm = charge;
      fresh->owner = tcb;
      fresh->flags = DEADLINE_FLAG_RESERVED;
      fresh->sched = DL_SCHED_INACTIVE;
    }
#ifdef CONFIG_TESTING_DEADLINE
  else if (g_admit_fail_before_commit != 0)
    {
      g_admit_fail_before_commit = 0;
      return -EIO;
    }
#endif

  flags = enter_critical_section();

  if ((tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      leave_critical_section(flags);
      if (fresh != NULL)
        {
          kmm_free(fresh);
        }

      return -ESRCH;
    }

  if (tcb->deadline == NULL)
    {
      if (fresh == NULL || g_dl_admitted > DL_BW_CAP ||
          charge > DL_BW_CAP - g_dl_admitted)
        {
          leave_critical_section(flags);
          if (fresh != NULL)
            {
              kmm_free(fresh);
            }

          return fresh == NULL ? -EINVAL : -EBUSY;
        }

      g_dl_admitted += charge;
      tcb->deadline = fresh;
      leave_critical_section(flags);
      return OK;
    }

  /* The TCB already has an object. Publish only a charge difference. */

  {
    FAR struct deadline_s *dl = tcb->deadline;
    uint32_t old;
    uint32_t kept;

    if (dl->owner != tcb ||
        (dl->flags & DEADLINE_FLAG_RESERVED) == 0 ||
        dl->sched == DL_SCHED_STOPPED)
      {
        int err = dl->sched == DL_SCHED_STOPPED ? -ESRCH : -EINVAL;

        leave_critical_section(flags);
        if (fresh != NULL)
          {
            kmm_free(fresh);
          }

        return err;
      }

    old = dl->charge_ppm;
    if (old > g_dl_admitted)
      {
        leave_critical_section(flags);
        if (fresh != NULL)
          {
            kmm_free(fresh);
          }

        return -EINVAL;
      }

    kept = g_dl_admitted - old;
    if (charge > DL_BW_CAP - kept)
      {
        leave_critical_section(flags);
        if (fresh != NULL)
          {
            kmm_free(fresh);
          }

        return -EBUSY;
      }

    g_dl_admitted = kept + charge;
    dl->charge_ppm = charge;
    dl->runtime_ns = runtime_ns;
    dl->deadline_ns = deadline_ns;
    dl->period_ns = period_ns;
    leave_critical_section(flags);
    if (fresh != NULL)
      {
        kmm_free(fresh);
      }

    return OK;
  }
}

/****************************************************************************
 * Name: nxsched_deadline_release
 *
 * Description:
 *   Drop this TCB's reservation once. A second call is success and does
 *   not change the ledger. A TCB that only shares the object pointer
 *   (a shallow copy) is rejected and does not free it. A CBS test fixture
 *   that was not reserved is left alone.
 *
 ****************************************************************************/

int nxsched_deadline_release(FAR struct tcb_s *tcb)
{
  FAR struct deadline_s *dl;
  irqstate_t flags;

  if (tcb == NULL)
    {
      return OK;
    }

  flags = enter_critical_section();
  dl = tcb->deadline;
  if (dl == NULL || (dl->flags & DEADLINE_FLAG_RESERVED) == 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  if (dl->owner != tcb || dl->charge_ppm > g_dl_admitted)
    {
      leave_critical_section(flags);
      return -EINVAL;
    }

  g_dl_admitted -= dl->charge_ppm;
  dl->charge_ppm = 0;
  dl->flags &= ~DEADLINE_FLAG_RESERVED;
  dl->generation++;
  dl->owner = NULL;
  wd_cancel(&dl->replenish);
  tcb->deadline = NULL;
  leave_critical_section(flags);
  kmm_free(dl);
  return OK;
}

#ifdef CONFIG_TESTING_DEADLINE

/****************************************************************************
 * Admission tests. Synthetic TCBs are not queued and are not deadline
 * policy tasks. Case 20 is the deadline-off link check, not this function.
 *
 ****************************************************************************/

static struct tcb_s g_admit_slot[3];
static struct tcb_s g_admit_copy;

static int admit_fail(int id)
{
  int i;

  printf("deadline admit: FAIL case %d\n", id);
  g_admit_fail_before_commit = 0;
  for (i = 0; i < 3; i++)
    {
      nxsched_deadline_release(&g_admit_slot[i]);
      memset(&g_admit_slot[i], 0, sizeof(g_admit_slot[i]));
    }

  return id;
}

static void admit_clear(int index)
{
  nxsched_deadline_release(&g_admit_slot[index]);
  memset(&g_admit_slot[index], 0, sizeof(g_admit_slot[index]));
  g_admit_slot[index].pid = -1;
  g_admit_slot[index].task_state = TSTATE_TASK_INVALID;
}

static int admit_ok_total(int id, uint32_t expect)
{
  if (nxsched_deadline_admitted_ppm() != expect)
    {
      return admit_fail(id);
    }

  return OK;
}

int sched_deadline_admit_test(void)
{
  FAR struct tcb_s *self = this_task();
  FAR struct deadline_s *saved;
  uint32_t charge;
  uint32_t total;
  uint64_t big_q = 17524406870024074020ull;
  uint64_t big_p = 18446744073709551600ull;
  int ret;
  int i;

  static const struct
  {
    uint64_t runtime;
    uint64_t period;
    uint32_t expect;
  } oracle[] =
  {
    { 950000ull, 1000000ull, 950000u },
    { 950001ull, 1000000ull, 950001u },
    { 475ull, 1000ull, 475000u },
    { 1ull, 3ull, 333334u },
    { 2ull, 3ull, 666667u },
    { 1ull, UINT64_MAX, 1u },
    { UINT64_MAX, UINT64_MAX, 1000000u },
    { UINT64_MAX / 2ull, UINT64_MAX, 500000u },
    { 18446744073709ull, UINT64_MAX, 1u },
    { UINT64_MAX - 100ull, UINT64_MAX, 1000000u },
    { 17524406870024074020ull, UINT64_MAX, 950000u },
    { 17524406870024074020ull, 18446744073709551600ull, 950000u },
  };

  for (i = 0; i < 3; i++)
    {
      admit_clear(i);
    }

  /* 1. Empty ledger. */

  if (nxsched_deadline_admitted_ppm() != 0 || self->deadline != NULL)
    {
      return admit_fail(1);
    }

  printf("deadline admit: ok case 1\n");

  /* 2. Exactly 95 percent. */

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 950000, 1000000,
                                1000000);
  if (ret != OK || g_admit_slot[0].deadline == NULL ||
      g_admit_slot[0].deadline->charge_ppm != 950000 ||
      (g_admit_slot[0].flags & TCB_FLAG_POLICY_MASK) != 0 ||
      (g_admit_slot[0].deadline->flags & DEADLINE_FLAG_SCHEDULABLE) != 0 ||
      g_admit_slot[0].deadline->sched != DL_SCHED_INACTIVE)
    {
      return admit_fail(2);
    }

  if (admit_ok_total(2, 950000) != OK)
    {
      return 2;
    }

  printf("deadline admit: ok case 2\n");
  admit_clear(0);

  /* 3. Strictly above 95 percent. */

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 950001, 1000000,
                                1000000);
  if (ret != -EBUSY || g_admit_slot[0].deadline != NULL ||
      nxsched_deadline_admitted_ppm() != 0)
    {
      return admit_fail(3);
    }

  printf("deadline admit: ok case 3\n");

  /* 4. Two reservations of 0.475. */

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 475, 1000, 1000);
  if (ret != OK)
    {
      return admit_fail(4);
    }

  ret = nxsched_deadline_reserve(&g_admit_slot[1], 475, 1000, 1000);
  if (ret != OK || nxsched_deadline_admitted_ppm() != 950000)
    {
      return admit_fail(4);
    }

  printf("deadline admit: ok case 4\n");

  /* 5. A third reservation at that total is rejected. */

  saved = g_admit_slot[0].deadline;
  ret = nxsched_deadline_reserve(&g_admit_slot[2], 475, 1000, 1000);
  if (ret != -EBUSY || g_admit_slot[2].deadline != NULL ||
      g_admit_slot[0].deadline != saved ||
      g_admit_slot[0].deadline->runtime_ns != 475 ||
      nxsched_deadline_admitted_ppm() != 950000)
    {
      return admit_fail(5);
    }

  printf("deadline admit: ok case 5\n");
  admit_clear(0);
  admit_clear(1);

  /* 6. Downward update releases only the difference. */

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 950000, 1000000,
                                1000000);
  saved = g_admit_slot[0].deadline;
  ret |= nxsched_deadline_reserve(&g_admit_slot[0], 100, 1000, 1000);
  if (ret != OK || g_admit_slot[0].deadline != saved ||
      saved->charge_ppm != 100000 || saved->runtime_ns != 100 ||
      nxsched_deadline_admitted_ppm() != 100000)
    {
      return admit_fail(6);
    }

  printf("deadline admit: ok case 6\n");

  /* 7. Upward update within the cap, still one charge. */

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 200, 1000, 1000);
  if (ret != OK || g_admit_slot[0].deadline != saved ||
      saved->charge_ppm != 200000 ||
      nxsched_deadline_admitted_ppm() != 200000)
    {
      return admit_fail(7);
    }

  printf("deadline admit: ok case 7\n");
  admit_clear(0);

  /* 8 and 16. Failed upward update keeps parameters and CBS fields. */

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 800000, 1000000,
                                1000000);
  saved = g_admit_slot[0].deadline;
  saved->remaining_ns = 42;
  saved->overrun_ns = 7;
  saved->abs_deadline = 99;
  saved->accounted_ns = 3;
  saved->runnable_seq = 11;
  saved->generation = 5;
  saved->replenish_count = 8;
  saved->sched = DL_SCHED_BLOCKED;
  ret = nxsched_deadline_reserve(&g_admit_slot[0], 960000, 1000000,
                                1000000);
  if (ret != -EBUSY || saved->runtime_ns != 800000 ||
      saved->deadline_ns != 1000000 || saved->period_ns != 1000000 ||
      saved->charge_ppm != 800000 || saved->remaining_ns != 42 ||
      saved->overrun_ns != 7 || saved->abs_deadline != 99 ||
      saved->accounted_ns != 3 || saved->runnable_seq != 11 ||
      saved->generation != 5 || saved->replenish_count != 8 ||
      saved->sched != DL_SCHED_BLOCKED ||
      nxsched_deadline_admitted_ppm() != 800000)
    {
      return admit_fail(8);
    }

  printf("deadline admit: ok case 8\n");
  printf("deadline admit: ok case 16\n");

  /* 9. Identical update. */

  total = nxsched_deadline_admitted_ppm();
  ret = nxsched_deadline_reserve(&g_admit_slot[0], 800000, 1000000,
                                1000000);
  if (ret != OK || g_admit_slot[0].deadline != saved ||
      saved->charge_ppm != 800000 || saved->remaining_ns != 42 ||
      nxsched_deadline_admitted_ppm() != total)
    {
      return admit_fail(9);
    }

  printf("deadline admit: ok case 9\n");

  /* 10. Release once, then again. A shallow copy is not an owner. */

  memcpy(&g_admit_copy, &g_admit_slot[0], sizeof(g_admit_copy));
  if (nxsched_deadline_release(&g_admit_copy) != -EINVAL ||
      nxsched_deadline_admitted_ppm() != 800000 ||
      g_admit_slot[0].deadline != saved)
    {
      return admit_fail(10);
    }

  if (nxsched_deadline_reserve(&g_admit_copy, 100, 1000, 1000) != -EINVAL ||
      saved->runtime_ns != 800000)
    {
      return admit_fail(10);
    }

  memset(&g_admit_copy, 0, sizeof(g_admit_copy));

  if (nxsched_deadline_release(&g_admit_slot[0]) != OK ||
      g_admit_slot[0].deadline != NULL ||
      nxsched_deadline_admitted_ppm() != 0)
    {
      return admit_fail(10);
    }

  if (nxsched_deadline_release(&g_admit_slot[0]) != OK ||
      nxsched_deadline_admitted_ppm() != 0)
    {
      return admit_fail(10);
    }

  printf("deadline admit: ok case 10\n");

  /* 11. Three reservations, released out of order. */

  if (nxsched_deadline_reserve(&g_admit_slot[0], 100, 1000, 1000) != OK ||
      nxsched_deadline_reserve(&g_admit_slot[1], 200, 1000, 1000) != OK ||
      nxsched_deadline_reserve(&g_admit_slot[2], 300, 1000, 1000) != OK ||
      nxsched_deadline_admitted_ppm() != 600000)
    {
      return admit_fail(11);
    }

  admit_clear(2);
  if (nxsched_deadline_admitted_ppm() != 300000)
    {
      return admit_fail(11);
    }

  admit_clear(0);
  if (nxsched_deadline_admitted_ppm() != 200000)
    {
      return admit_fail(11);
    }

  admit_clear(1);
  if (nxsched_deadline_admitted_ppm() != 0)
    {
      return admit_fail(11);
    }

  printf("deadline admit: ok case 11\n");

  /* 12. Malformed parameters. */

  if (nxsched_deadline_reserve(&g_admit_slot[0], 100, 1000, 1000) != OK)
    {
      return admit_fail(12);
    }

  total = nxsched_deadline_admitted_ppm();
  if (nxsched_deadline_reserve(&g_admit_slot[1], 0, 1, 1) != -EINVAL ||
      nxsched_deadline_reserve(&g_admit_slot[1], 1, 0, 1) != -EINVAL ||
      nxsched_deadline_reserve(&g_admit_slot[1], 1, 1, 0) != -EINVAL ||
      nxsched_deadline_reserve(&g_admit_slot[1], 2, 1, 2) != -EINVAL ||
      nxsched_deadline_reserve(&g_admit_slot[1], 1, 2, 1) != -EINVAL ||
      nxsched_deadline_reserve(&g_admit_slot[1], 10000000, 20000000,
                              40000000) != -EINVAL ||
      g_admit_slot[1].deadline != NULL ||
      g_admit_slot[0].deadline->runtime_ns != 100 ||
      g_admit_slot[0].deadline->deadline_ns != 1000 ||
      g_admit_slot[0].deadline->period_ns != 1000 ||
      nxsched_deadline_admitted_ppm() != total)
    {
      return admit_fail(12);
    }

  if (nxsched_deadline_reserve(&g_admit_slot[0], 10000000, 20000000,
                              40000000) != -EINVAL ||
      g_admit_slot[0].deadline->runtime_ns != 100 ||
      g_admit_slot[0].deadline->deadline_ns != 1000 ||
      g_admit_slot[0].deadline->period_ns != 1000 ||
      g_admit_slot[0].deadline->charge_ppm != 100000 ||
      nxsched_deadline_admitted_ppm() != total)
    {
      return admit_fail(12);
    }

  printf("deadline admit: ok case 12\n");
  admit_clear(0);

  /* 13. Large values against an independent integer oracle. */

  for (i = 0; i < (int)(sizeof(oracle) / sizeof(oracle[0])); i++)
    {
      ret = nxsched_deadline_charge_ppm(oracle[i].runtime,
                                       oracle[i].period, &charge);
      if (ret != OK || charge != oracle[i].expect)
        {
          printf("deadline admit: oracle slot %d got %u\n", i,
                 (unsigned int)charge);
          return admit_fail(13);
        }
    }

  if (nxsched_deadline_charge_ppm(0, 1, &charge) != -EINVAL ||
      nxsched_deadline_charge_ppm(2, 1, &charge) != -EINVAL)
    {
      return admit_fail(13);
    }

  ret = nxsched_deadline_reserve(&g_admit_slot[0], big_q, big_p, big_p);
  if (ret != OK || g_admit_slot[0].deadline->charge_ppm != 950000 ||
      nxsched_deadline_admitted_ppm() != 950000)
    {
      return admit_fail(13);
    }

  printf("deadline admit: ok case 13\n");
  admit_clear(0);

  /* 14. A tiny positive ratio rounds up to one part per million. */

  ret = nxsched_deadline_charge_ppm(1, UINT64_MAX, &charge);
  if (ret != OK || charge != 1)
    {
      return admit_fail(14);
    }

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 1, UINT64_MAX, UINT64_MAX);
  if (ret != OK || g_admit_slot[0].deadline->charge_ppm != 1 ||
      nxsched_deadline_admitted_ppm() != 1)
    {
      return admit_fail(14);
    }

  printf("deadline admit: ok case 14\n");
  admit_clear(0);

  /* 15. Failure after allocation and before the ledger commit. */

  nxsched_deadline_admit_fail_next();
  ret = nxsched_deadline_reserve(&g_admit_slot[0], 100, 1000, 1000);
  if (ret != -EIO || g_admit_slot[0].deadline != NULL ||
      nxsched_deadline_admitted_ppm() != 0)
    {
      return admit_fail(15);
    }

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 100, 1000, 1000);
  if (ret != OK || nxsched_deadline_admitted_ppm() != 100000)
    {
      return admit_fail(15);
    }

  printf("deadline admit: ok case 15\n");

  /* 17. Block, throttle, and a modeled replenishment keep the charge.
   * The static CBS replenish function is not called. Stop releases it.
   */

  saved = g_admit_slot[0].deadline;
  saved->sched = DL_SCHED_BLOCKED;
  if (saved->charge_ppm != 100000 ||
      nxsched_deadline_admitted_ppm() != 100000)
    {
      return admit_fail(17);
    }

  saved->sched = DL_SCHED_THROTTLED;
  saved->remaining_ns = 0;
  saved->overrun_ns = 5;
  if (nxsched_deadline_admitted_ppm() != 100000)
    {
      return admit_fail(17);
    }

  saved->abs_deadline += saved->period_ns;
  saved->remaining_ns += saved->runtime_ns;
  saved->replenish_count++;
  saved->sched = DL_SCHED_ELIGIBLE;
  if (saved->charge_ppm != 100000 ||
      nxsched_deadline_admitted_ppm() != 100000)
    {
      return admit_fail(17);
    }

  nxsched_deadline_stop(&g_admit_slot[0]);
  if (g_admit_slot[0].deadline != NULL ||
      nxsched_deadline_admitted_ppm() != 0)
    {
      return admit_fail(17);
    }

  printf("deadline admit: ok case 17\n");

  /* 18. stop, stop, release. nxtask_recover is not called: it also
   * runs semaphore and message-queue recovery, which a synthetic TCB
   * must not enter. recover's only deadline call is stop().
   */

  ret = nxsched_deadline_reserve(&g_admit_slot[0], 100, 1000, 1000);
  if (ret != OK)
    {
      return admit_fail(18);
    }

  nxsched_deadline_stop(&g_admit_slot[0]);
  nxsched_deadline_stop(&g_admit_slot[0]);
  if (nxsched_deadline_release(&g_admit_slot[0]) != OK ||
      g_admit_slot[0].deadline != NULL ||
      nxsched_deadline_admitted_ppm() != 0)
    {
      return admit_fail(18);
    }

  printf("deadline admit: ok case 18\n");

  /* 19. Mixed operations stay inside the cap and finish at zero. */

  if (nxsched_deadline_reserve(&g_admit_slot[0], 400000, 1000000,
                              1000000) != OK ||
      nxsched_deadline_admitted_ppm() > DL_BW_CAP)
    {
      return admit_fail(19);
    }

  if (nxsched_deadline_reserve(&g_admit_slot[1], 400000, 1000000,
                              1000000) != OK ||
      nxsched_deadline_admitted_ppm() != 800000)
    {
      return admit_fail(19);
    }

  if (nxsched_deadline_reserve(&g_admit_slot[1], 600000, 1000000,
                              1000000) != -EBUSY ||
      nxsched_deadline_admitted_ppm() != 800000)
    {
      return admit_fail(19);
    }

  if (nxsched_deadline_reserve(&g_admit_slot[0], 300000, 1000000,
                              1000000) != OK ||
      nxsched_deadline_admitted_ppm() != 700000)
    {
      return admit_fail(19);
    }

  if (nxsched_deadline_reserve(&g_admit_slot[2], 250000, 1000000,
                              1000000) != OK ||
      nxsched_deadline_admitted_ppm() != 950000)
    {
      return admit_fail(19);
    }

  if (nxsched_deadline_reserve(&g_admit_slot[2], 250000, 1000000,
                              1000000) != OK ||
      nxsched_deadline_admitted_ppm() != 950000)
    {
      return admit_fail(19);
    }

  admit_clear(1);
  admit_clear(2);
  admit_clear(0);
  if (nxsched_deadline_admitted_ppm() != 0 || self->deadline != NULL ||
      (self->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_DEADLINE)
    {
      return admit_fail(19);
    }

  printf("deadline admit: ok case 19\n");
  printf("deadline admit: PASS cases 1-19\n");
  printf("deadline admit: case 20 is the deadline-off link check\n");
  return 0;
}

#endif /* CONFIG_TESTING_DEADLINE */
