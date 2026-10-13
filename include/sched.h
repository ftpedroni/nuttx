/****************************************************************************
 * include/sched.h
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

#ifndef __INCLUDE_SCHED_H
#define __INCLUDE_SCHED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <strings.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Task Management Definitions **********************************************/

/* POSIX-like scheduling policies */

#define SCHED_NORMAL              0  /* Alias to SCHED_OTHER */
#define SCHED_OTHER               0  /* Map to SCHED_FIFO or SCHED_RR */
#define SCHED_FIFO                1  /* FIFO priority scheduling policy */
#define SCHED_RR                  2  /* Round robin scheduling policy */
#define SCHED_SPORADIC            3  /* Sporadic scheduling policy */
#define SCHED_BATCH               4  /* Batch scheduling policy */
#define SCHED_IDLE                5  /* Idle scheduling policy */
#define SCHED_DEADLINE            6  /* Deadline scheduling policy */

/* SCHED_DEADLINE is always visible so callers can name policy 6.
 * sched_setscheduler() rejects it. sched_setattr() can name the policy
 * only when CONFIG_SCHED_DEADLINE is set, and even a valid request is
 * refused until the EDF/CBS engine exists. No TCB is created with the
 * deadline encoding.
 */

/* Maximum number of SCHED_SPORADIC replenishments */

#define SS_REPL_MAX               CONFIG_SCHED_SPORADIC_MAXREPL

/* Cancellation definitions *************************************************/

/* Cancellation states used by task_setcancelstate() */

#define TASK_CANCEL_ENABLE        (0)
#define TASK_CANCEL_DISABLE       (1)

/* Cancellation types used by task_setcanceltype() */

#define TASK_CANCEL_DEFERRED      (0)
#define TASK_CANCEL_ASYNCHRONOUS  (1)

/* Pthread definitions ******************************************************/

#define PTHREAD_KEYS_MAX          CONFIG_TLS_NELEM

/* CPU affinity mask helpers ************************************************/

/* These are not standard but are defined for Linux compatibility */

/* Maximum number of CPUs */

#define CPU_SETSIZE CONFIG_SMP_NCPUS

/* void CPU_ZERO(FAR cpu_set_t *set); */

#define CPU_ZERO(s) do { *(s) = 0; } while (0)

/* void CPU_SET(int cpu, FAR cpu_set_t *set); */

#define CPU_SET(c,s) do { *(s) |= (1u << (c)); } while (0)

/* void CPU_CLR(int cpu, FAR cpu_set_t *set); */

#define CPU_CLR(c,s) do { *(s) &= ~(1u << (c)); } while (0)

/* int  CPU_ISSET(int cpu, FAR const cpu_set_t *set); */

#define CPU_ISSET(c,s) ((*(s) & (1u << (c))) != 0)

/* int CPU_COUNT(FAR const cpu_set_t *set); */

#define CPU_COUNT(s) popcountl(*s)

/* void CPU_AND(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#define CPU_AND(d,s1,s2) do { *(d) = *(s1) & *(s2); } while (0)

/* void CPU_OR(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *             FAR const cpu_set_t *srcset2);
 */

#define CPU_OR(d,s1,s2) do { *(d) = *(s1) | *(s2); } while (0)

/* void CPU_XOR(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#define CPU_XOR(d,s1,s2) do { *(d) = *(s1) ^ *(s2); } while (0)

/* int CPU_EQUAL(FAR const cpu_set_t *set1, FAR const cpu_set_t *set2); */

#define CPU_EQUAL(s1,s2) (*(s1) == *(s2))

/* REVISIT: Variably sized CPU sets are not supported */

/* FAR cpu_set_t *CPU_ALLOC(int num_cpus); */

#define CPU_ALLOC(n) (FAR cpu_set_t *)malloc(sizeof(cpu_set_t));

/* void CPU_FREE(cpu_set_t *set); */

#define CPU_FREE(s) free(s)

/* size_t CPU_ALLOC_SIZE(int num_cpus); */

#define CPU_ALLOC_SIZE(n) sizeof(cpu_set_t)

/* void CPU_ZERO_S(size_t setsize, FAR cpu_set_t *set); */

#define CPU_ZERO_S(n,s) CPU_ZERO_S(s)

/* void CPU_SET_S(int cpu, size_t setsize, FAR cpu_set_t *set); */

#define CPU_SET_S(c,n,s) CPU_SET(c,s)

/* void CPU_CLR_S(int cpu, size_t setsize, FAR cpu_set_t *set); */

#define CPU_CLR_S(c,n,s) CPU_CLR(c,s)

/* int CPU_ISSET_S(int cpu, size_t setsize, FAR const cpu_set_t *set); */

#define CPU_ISSET_S(c,n,s) CPU_ISSET(c,s)

/* int CPU_COUNT_S(size_t setsize, FAR const cpu_set_t *set); */

#define CPU_COUNT_S(n,s) CPU_COUNT(s)

/* void CPU_AND_S(size_t setsize, FAR cpu_set_t *destset,
 *                FAR const cpu_set_t *srcset1,
 *                FAR const cpu_set_t *srcset2);
 */

#define CPU_AND_S(n,d,s1,s2) CPU_AND(d,s1,s2)

/* void CPU_OR_S(size_t setsize, FAR cpu_set_t *destset,
 *              FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#define CPU_OR_S(n,d,s1,s2) CPU_OR(d,s1,s2)

/* void CPU_XOR_S(size_t setsize, FAR cpu_set_t *destset,
 *                FAR const cpu_set_t *srcset1,
 *                FAR const cpu_set_t *srcset2);
 */

#define CPU_XOR_S(n,d,s1,s2) CPU_XOR(d,s1,s2)

/* int CPU_EQUAL_S(size_t setsize, FAR const cpu_set_t *set1,
 *                 FAR const cpu_set_t *set2);
 */

#define CPU_EQUAL_S(n,s1,s2) CPU_EQUAL(s1,s2)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This is the POSIX-like scheduling parameter structure */

struct sched_param
{
  int sched_priority;                   /* Base thread priority */

#ifdef CONFIG_SCHED_SPORADIC
  int sched_ss_low_priority;            /* Low scheduling priority for sporadic
                                         * server */
  struct timespec sched_ss_repl_period; /* Replenishment period for sporadic
                                         * server. */
  struct timespec sched_ss_init_budget; /* Initial budget for sporadic server */
  int sched_ss_max_repl;                /* Maximum pending replenishments for
                                         * sporadic server. */
#endif
};

/* Linux-shaped scheduling attributes. This is not the Linux ABI.
 * The three time fields are nanoseconds. v0 accepts only a buffer
 * whose size is exactly SCHED_ATTR_SIZE_VER0; smaller or larger
 * sizes are rejected and are not read or written.
 * Visible only with CONFIG_SCHED_DEADLINE. v0 accepts a deadline
 * task only when runtime > 0, runtime <= deadline, and
 * deadline == period. A shorter deadline is -EINVAL and does not
 * change an existing reservation. sched_setattr() also enforces the
 * admission cap. Time spent past the budget while preemption is
 * locked is debt: later posts repay it. The scheduler cannot take
 * that interval back while the lock is held. This is not a statement
 * about physical isolation on a simulator or on hardware.
 */

#ifdef CONFIG_SCHED_DEADLINE
struct sched_attr
{
  uint32_t size;            /* Must be SCHED_ATTR_SIZE_VER0 */
  uint32_t sched_policy;    /* SCHED_* value */
  uint64_t sched_flags;     /* v0: must be 0 */
  int32_t  sched_nice;      /* v0: must be 0 */
  uint32_t sched_priority;  /* Deadline: must be 0. Others: NuttX priority */
  uint64_t sched_runtime;   /* Deadline runtime, nanoseconds */
  uint64_t sched_deadline;  /* Deadline relative deadline, nanoseconds */
  uint64_t sched_period;    /* Deadline period, nanoseconds */
};

#define SCHED_ATTR_SIZE_VER0 sizeof(struct sched_attr)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Task Control Interfaces (non-standard) */

#ifndef CONFIG_BUILD_KERNEL
int    task_create(FAR const char *name, int priority, int stack_size,
                   main_t entry, FAR char * const argv[]);
int    task_create_with_stack(FAR const char *name, int priority,
                              FAR void *stack_addr, int stack_size,
                              main_t entry, FAR char * const argv[]);
#endif
int    task_delete(pid_t pid);
int    task_restart(pid_t pid);

int    task_setcancelstate(int state, FAR int *oldstate);
int    task_setcanceltype(int type, FAR int *oldtype);
void   task_testcancel(void);

/* Task Scheduling Interfaces (based on POSIX APIs) */

int    sched_getcpu(void);
int    sched_setparam(pid_t pid, FAR const struct sched_param *param);
int    sched_getparam(pid_t pid, FAR struct sched_param *param);
int    sched_setscheduler(pid_t pid, int policy,
                          FAR const struct sched_param *param);
int    sched_getscheduler(pid_t pid);
int    sched_yield(void);
int    sched_get_priority_max(int policy);
int    sched_get_priority_min(int policy);
int    sched_rr_get_interval(pid_t pid, FAR struct timespec *interval);

#ifdef CONFIG_SCHED_DEADLINE
int    sched_setattr(pid_t pid, FAR const struct sched_attr *attr,
                     unsigned int flags);
int    sched_getattr(pid_t pid, FAR struct sched_attr *attr,
                     unsigned int size, unsigned int flags);

/* Exercised by apps/testing/sched/deadline. The order, CBS, and
 * admission tests use synthetic TCBs. The view below is how the live
 * task tests read a reservation without changing it.
 */

#  ifdef CONFIG_TESTING_DEADLINE
struct sched_deadline_view_s
{
  uint64_t runtime_ns;
  uint64_t deadline_ns;
  uint64_t period_ns;
  uint64_t remaining_ns;
  uint64_t abs_deadline;
  uint64_t overrun_ns;
  uint64_t runnable_seq;
  uint32_t admitted_ppm;
  uint8_t  sched;
  uint8_t  flags;
};

int      sched_deadline_order_test(void);
int      sched_deadline_cbs_test(void);
int      sched_deadline_time_test(void);
int      sched_deadline_admit_test(void);
int      sched_deadline_oracle_test(void);

struct sched_deadline_t9_s
{
  int      tick_irq;
  int      tick_running;
  int      tick_head;
  uint64_t tick_now;
  uint64_t tick_rem;
  int      sw_irq;
  int      sw_from;
  int      sw_to;
  int      sw_running;
  int      sw_head;
  uint64_t sw_now;
  uint64_t sw_from_before;
  uint64_t sw_from_after;
  uint64_t sw_to_before;
  uint64_t sw_to_after;
};

void     sched_deadline_t9_expect(pid_t low, pid_t high);
int      sched_deadline_t9_kick(FAR void *sem);
void     sched_deadline_t9_read(FAR struct sched_deadline_t9_s *out);
int      sched_deadline_view(pid_t pid,
                             FAR struct sched_deadline_view_s *view);
uint32_t sched_deadline_admitted(void);
#    ifdef CONFIG_SCHED_DEADLINE_TRACE
void     sched_deadline_trace_reset(void);
void     sched_deadline_trace_freeze(void);
void     sched_deadline_trace_report(uint64_t t0, uint64_t t1);
#    endif
#  endif
#endif

#ifdef CONFIG_SMP
/* Task affinity */

int    sched_setaffinity(pid_t pid, size_t cpusetsize,
                         FAR const cpu_set_t *mask);
int    sched_getaffinity(pid_t pid, size_t cpusetsize, FAR cpu_set_t *mask);
int    sched_cpucount(FAR const cpu_set_t *set);
#else
#  define sched_setaffinity(p, c, m) 0
#  define sched_getaffinity(p, c, m) (*(m) |= (1 << 0), 0)
#  define sched_cpucount(s) 1
#endif /* CONFIG_SMP */

/* Task Switching Interfaces (non-standard) */

void   sched_lock(void);
void   sched_unlock(void);
int    sched_lockcount(void);

/* Queries */

bool   sched_idletask(void);

/* Task Backtrace */

#ifdef CONFIG_SCHED_BACKTRACE
int    sched_backtrace(pid_t tid, FAR void **buffer, int size, int skip);
void   sched_dumpstack(pid_t tid);
#else
#  define sched_backtrace(tid, buffer, size, skip) 0
#  define sched_dumpstack(tid)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __INCLUDE_SCHED_H */
