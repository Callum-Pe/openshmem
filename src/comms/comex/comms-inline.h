/*
 *
 * Copyright (c) 2016
 *   Stony Brook University
 * Copyright (c) 2015 - 2016
 *   Los Alamos National Security, LLC.
 * Copyright (c) 2011 - 2016
 *   University of Houston System and UT-Battelle, LLC.
 * Copyright (c) 2009 - 2016
 *   Silicon Graphics International Corp.  SHMEM is copyrighted
 *   by Silicon Graphics International Corp. (SGI) The OpenSHMEM API
 *   (shmem) is released by Open Source Software Solutions, Inc., under an
 *   agreement with Silicon Graphics International Corp. (SGI).
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * o Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimers.
 *
 * o Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * o Neither the name of the University of Houston System,
 *   UT-Battelle, LLC. nor the names of its contributors may be used to
 *   endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * o Neither the name of Los Alamos National Security, LLC, Los Alamos
 *   National Laboratory, LANL, the U.S. Government, nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/**
 * This file provides the layer on top of GASNet, ARMCI or whatever.
 * API should be formalized at some point, but basically everything
 * non-static that starts with "shmemi_comms_"
 */

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>
#include <limits.h>

#include "uthash.h"

#include "state.h"
#include "memalloc.h"
#include "atomic.h"
#include "ping.h"
#include "exe.h"
#include "globalvar.h"
#include "clock.h"

#include "barrier.h"
#include "barrier-all.h"
#include "broadcast.h"
#include "collect.h"
#include "fcollect.h"

#include "trace.h"
#include "utils.h"
#include "unitparse.h"

#include "shmemx.h"

#include "comms-shared.h"

#include <assert.h>
/**
 * --------------- real work starts here ---------------------
 *
 */

static inline void comms_bailout (char *fmt, ...);
static inline void shmemi_comms_exit (int status);
static inline void *shmemi_symmetric_addr_lookup (void *dest, int pe);
/**
 * trap comex errors gracefully
 *
 */
#define COMEX_SAFE(fncall)                                                 \
    do {                                                                   \
           const int _retval = fncall ;                                    \
           if (_retval != COMEX_SUCCESS) {                                 \
               comms_bailout ("error calling: %s at %s:%i, %s (%s)\n",     \
                              #fncall, __FILE__, __LINE__,                 \
                              __FILE__,                                    \
                              __LINE__                                     \
                              );                                           \
           }                                                               \
       } while(0)                                                          

#define BLOCKUNTIL(cond)                                                   \
        while(!(cond)){comex_wait_all(shmemgroup);}
/* bail.c */

#define MSG_BUF_SIZE 256

/**
 * Handle error messages while initializing the comms layer.  We don't
 * have access to the trace sub-system yet, since it depends on comms
 * being up to get PE and other informational output
 */

static inline void
comms_bailout (char *fmt, ...)
{
    char tmp1[MSG_BUF_SIZE];
    char tmp2[MSG_BUF_SIZE];    /* incoming args */
    va_list ap;

    strncpy (tmp1, "COMMS ERROR: ", MSG_BUF_SIZE);

    va_start (ap, fmt);
    vsnprintf (tmp2, MSG_BUF_SIZE, fmt, ap);
    va_end (ap);

    strncat (tmp1, tmp2, strlen (tmp2));
    strncat (tmp1, "\n", 1);

    fputs (tmp1, stderr);
    fflush (stderr);

    shmemi_comms_exit (1);
}

/* end: bail.c */

/* locality query */

static bool thread_starter = false;

static inline bool
shmemi_thread_starter (void)
{
    const int me = GET_STATE (mype);
    const int *where = GET_STATE (locp);

    /* PE 0 always starts a thread */
    if (me == 0) {
        return true;
    }

    /* only start thread if I am first PE on a host */
    if (where[me - 1] < where[me]) {
        return true;
    }

    return false;
}

/**
 * get some hopefully-interesting locality information.
 *
 */

/* service.c */

/**
 * Do network service.  When code is not engaged in shmem calls,
 * something needs to provide communication access so that operations
 * where "this" PE is a passive target can continue
 */

/**
 * choose thread implementation
 */

/**
 * for hi-res timer
 */

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309
#endif /* _POSIX_C_SOURCE */
#include <time.h>

/**
 * polling sentinel
 */

static volatile bool done = false;

/**
 * assume initially we need to manage progress ourselves
 */
static bool use_conduit_thread = false;

/**
 * stop the servicer
 */

static inline void
shmemi_service_finalize (void)
{
    if (!use_conduit_thread) {
        done = true;

        if (thread_starter) {
        }
    }
}

/* end: service.c */


/**
 * which node (PE) am I?
 */
static inline int
shmemi_comms_mynode (void)
{
    int rank;
    assert(comex_initialized());
    comex_group_rank(shmemgroup, &rank);
    return rank;
}

/**
 * how many nodes (PEs) take part in this program?
 */
static inline int
shmemi_comms_nodes (void)
{
    int size;
    assert(comex_initialized());
    comex_group_size(shmemgroup, &size);
    return size;
}


/**
 * ---------------------------------------------------------------------------
 *
 * global barrier done through comex
 *
 */

static inline void
shmemi_comms_barrier_all (void)
{
    comex_barrier( shmemgroup);
    barcount += 1;
}

/**
 * ---------------------------------------------------------------------------
 *
 * lookup where another PE stores things
 *
 */

/**
 * where the symmetric memory lives on the given PE
 */
#define SHMEM_SYMMETRIC_HEAP_BASE(p) (heapptr[p])
#define SHMEM_SYMMETRIC_HEAP_SIZE(p) (DEFAULT_HEAP_SIZE)
/**
 * translate my "dest" to corresponding address on PE "pe"
 */
static inline void *
shmemi_symmetric_addr_lookup (void *dest, int pe)
{
    /* globals are in same place everywhere */
    if (shmemi_symmetric_is_globalvar (dest)) {
        return dest;
    }
    /* symmetric if inside of heap */
    {
        int me = GET_STATE (mype);
        size_t al = (size_t) SHMEM_SYMMETRIC_HEAP_BASE (me); /* lower bound */
        size_t aao = (size_t) dest; /* my addr as offset */
        long offset = aao - al;

        /* trap addresses outside the heap */
        if (offset < 0) {
            return NULL;
        }
        if (offset > SHMEM_SYMMETRIC_HEAP_SIZE (me)) {
            return NULL;
        }
        /* and where it is in the remote heap */
        return SHMEM_SYMMETRIC_HEAP_BASE (pe) + offset;
    }
}

/**
 * can't just call getenv, it might not pass through environment
 * info to other nodes from launch.
 */
static inline char *
shmemi_comms_getenv (const char *name)
{
    return getenv(name);
}

/**
 * work out how big the symmetric segment areas should be.
 *
 * Either from environment setting, or default value from
 * implementation
 */
static inline size_t
shmemi_comms_get_segment_size (void)
{

    char *mlss_str = shmemi_comms_getenv ("SHMEM_SYMMETRIC_HEAP_SIZE");
    size_t retval;
    int ok;
    if (EXPR_LIKELY (mlss_str == (char *) NULL)) 
      return DEFAULT_HEAP_SIZE;
    shmemi_parse_size (mlss_str, &retval, &ok);
    return retval;
    comms_bailout ("Unusable symmetric heap size \"%s\"", mlss_str);
    /* NOT REACHED */
    return 0;
}

/**
 * initialize the symmetric memory, taking into account the different
 */
static inline void
shmemi_symmetric_memory_init (void)
{
    const int me = GET_STATE (mype);
    shmemi_mem_init (heapptr[me],GET_STATE(heapsize));
}

/**
 * shut down the memory allocation handler
 */
static inline void
shmemi_symmetric_memory_finalize (void)
{
    shmemi_mem_finalize ();
}

/**
 * -- atomics handlers ---------------------------------------------------------
 */

/*
 * to wait on remote updates
 */
#define VOLATILIZE(Type, Var) (* ( volatile Type *) (Var))

#define COMMS_WAIT_TYPE(Name, Type, OpName, Op)                         \
    static inline void                                                  \
    shmemi_comms_wait_##Name##_##OpName (volatile Type *var, Type cmp_value) \
    {                                                                   \
        BLOCKUNTIL ( VOLATILIZE (Type, var) Op cmp_value );      \
    }
COMMS_WAIT_TYPE (short, short, eq, ==);
COMMS_WAIT_TYPE (int, int, eq, ==);
COMMS_WAIT_TYPE (long, long, eq, ==);
COMMS_WAIT_TYPE (longlong, long long, eq, ==);

COMMS_WAIT_TYPE (short, short, ne, !=);
COMMS_WAIT_TYPE (int, int, ne, !=);
COMMS_WAIT_TYPE (long, long, ne, !=);
COMMS_WAIT_TYPE (longlong, long long, ne, !=);

COMMS_WAIT_TYPE (short, short, gt, >);
COMMS_WAIT_TYPE (int, int, gt, >);
COMMS_WAIT_TYPE (long, long, gt, >);
COMMS_WAIT_TYPE (longlong, long long, gt, >);

COMMS_WAIT_TYPE (short, short, le, <=);
COMMS_WAIT_TYPE (int, int, le, <=);
COMMS_WAIT_TYPE (long, long, le, <=);
COMMS_WAIT_TYPE (longlong, long long, le, <=);

COMMS_WAIT_TYPE (short, short, lt, <);
COMMS_WAIT_TYPE (int, int, lt, <);
COMMS_WAIT_TYPE (long, long, lt, <);
COMMS_WAIT_TYPE (longlong, long long, lt, <);

COMMS_WAIT_TYPE (short, short, ge, >=);
COMMS_WAIT_TYPE (int, int, ge, >=);
COMMS_WAIT_TYPE (long, long, ge, >=);
COMMS_WAIT_TYPE (longlong, long long, ge, >=);

#define WAIT_ON_COMPLETION(Cond)   BLOCKUNTIL (Cond)


/**
 * ---------------------------------------------------------------------------
 */

/**
 * perform the ping
 *
 * TODO: JUST RETURN TRUE FOR NOW IF GOOD PE, NEED TO WORK ON PROGRESS LOGIC
 *
 */
static inline int
shmemi_comms_ping_request (int pe)
{
    if ( (pe >= 0) && (pe < GET_STATE(numpes)) ) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * ---------------------------------------------------------------------------
 */
static volatile unsigned long put_counter = 0L;
static inline void
atomic_wait_put_zero (void)
{
    WAIT_ON_COMPLETION (put_counter == 0L);
}


/**
 * called by mainline to fence off outstanding requests
 *
 * chances here for fence/quiet differentiation and optimization, but
 * we'll just fence <=> quiet
 */
static inline void
do_fencequiet (void)
{
    atomic_wait_put_zero ();
    comex_wait_all(shmemgroup);
    LOAD_STORE_FENCE ();
    return;
}

static inline void
shmemi_comms_quiet_request (void)
{
    do_fencequiet ();
}

static inline void
shmemi_comms_fence_request (void)
{
    do_fencequiet ();
}

/**
 * fence and quiet tests just call fence/quiet and then report success
 *
 */

static inline int
shmemi_fence_test (void)
{
    shmemi_comms_fence_request ();
    return 1;
}

static inline int
shmemi_quiet_test (void)
{
    shmemi_comms_quiet_request ();
    return 1;
}
/**
 * called by initiator PE of global_exit
 *
 * TODO: tree-based setup would be more scalable.
 */
static void
shmemi_comms_globalexit_request (int status)
{

}
/* end: global exit */


/**
 * ---------------------------------------------------------------------------
 *
 * start of handlers
 */

/**
 * First parse out the process' command-line.  This is important for
 * the UDP conduit, in which the number of PEs comes from the
 * command-line rather than a launcher program.
 */

static int argc = 0;
static char **argv = NULL;

static const char *cmdline = "/proc/self/cmdline";
static const char *cmdline_fmt = "/proc/%ld/cmdline";

static inline void
parse_cmdline (void)
{
    FILE *fp;
    char an_arg[1024];          /* TODO: arbitrary size */
    char *p = an_arg;
    int i = 0;
    int c;

    /*
     * try to find this process' command-line:
     * either from short-cut, or from pid
     */
    fp = fopen (cmdline, "r");
    if (EXPR_UNLIKELY (fp == NULL)) {
        char pidname[MAXPATHLEN];
        snprintf (pidname, MAXPATHLEN, cmdline_fmt, getpid ());
        fp = fopen (pidname, "r");
        if (EXPR_UNLIKELY (fp == NULL)) {
            comms_bailout ("could not discover process' command-line (%s)",
                           strerror (errno)
                           );
            /* NOT REACHED */
        }
    }

    /* first count the number of nuls in cmdline to see how many args */
    while ((c = fgetc (fp)) != EOF) {
        if (c == '\0') {
            argc += 1;
        }
    }
    rewind (fp);

    argv = (char **) malloc ((argc + 1) * sizeof (*argv));
    if (EXPR_UNLIKELY (argv == (char **) NULL)) {
        comms_bailout
            ("internal error: unable to allocate memory for"
             " faked command-line arguments");
        /* NOT REACHED */
    }

    while (1) {
        int c = fgetc (fp);
        switch (c) {
        case EOF:              /* end of args */
            argv[i] = NULL;
            goto end;
            break;
        case '\0':             /* end of this arg */
            *p = c;
            argv[i++] = strdup (an_arg);    /* unchecked return */
            p = an_arg;
            break;
        default:               /* copy out char in this arg */
            *p++ = c;
            break;
        }
    }
  end:
    fclose (fp);
}

static inline void
release_cmdline (void)
{
    if (argv != NULL) {
        int i;
        for (i = 0; i < argc; i += 1) {
            if (argv[i] != NULL) {
                free (argv[i]);
            }
        }
    }
}

/**
 * GASNet does this timeout thing if its collective routines
 * (e.g. barrier) go idle, so make this as long as possible
 */
static inline void
maximize_gasnet_timeout (void)
{
    char buf[32];
    snprintf (buf, 32, "%d", INT_MAX - 1);
    setenv ("GASNET_ EXITTIMEOUT", buf, 1);
}

/**
 * -----------------------------------------------------------------------
 */

/**
 * bail out of run-time with STATUS error code
 */
static inline void
shmemi_comms_exit (int status)
{
    /*
     * calling multiple times is undefined, I'm just going to do nothing
     */
    if (EXPR_UNLIKELY (GET_STATE (pe_status) == PE_SHUTDOWN)) {
        return;
    }

    /* ok, no more pending I/O ... */
    shmemi_comms_barrier_all ();

    release_cmdline ();
    comex_destroy_mutexes();
    comex_finalize();
    MPI_Finalize();
    /* clean up atomics and memory */
    shmemi_atomic_finalize ();
    shmemi_symmetric_memory_finalize ();
    shmemi_symmetric_globalvar_table_finalize ();

    /* clean up plugin modules */
    /* shmemi_modules_finalize (); */

    /* tidy up binary inspector */
    shmemi_executable_finalize ();

    /* stop run time clock */
    shmemi_elapsed_clock_finalize ();

    /* update our state */
    SET_STATE (pe_status, PE_SHUTDOWN);

    shmemi_trace (SHMEM_LOG_FINALIZE,
                  "finalizing shutdown, handing off to communications layer");

    shmemi_tracers_fini ();

    /*
     * TODO, tc: need to be better at cleanup for 1.2, since finalize
     * doesn't imply follow-on exit, merely end of OpenSHMEM portion.
     *
     */

    /* shmemi_comms_barrier_all (); */
}

/**
 * finalize can now happen in 2 ways: (1) program finishes via
 * atexit(), or (2) user explicitly calls shmem_finalize().  Need to
 * detect explicit call and not terminate program until exit.
 *
 */
static void
shmemi_comms_finalize (void)
{
    shmemi_comms_exit (EXIT_SUCCESS);
}

/**
 * This is where the communications layer gets set up and torn down
 */
static inline void
shmemi_comms_init (void)
{
    /*
     * prepare environment for GASNet
     */
    parse_cmdline ();
    COMEX_SAFE(comex_init_args(&argc,&argv));
    /* now we can ask about the node count & heap */
    SET_STATE (mype, shmemi_comms_mynode ());
    SET_STATE (numpes, shmemi_comms_nodes ());
    SET_STATE (heapsize, shmemi_comms_get_segment_size ());
        heapptr = (void **) calloc (GET_STATE (numpes), sizeof (void*));
    comex_malloc( heapptr, GET_STATE (heapsize), shmemgroup); 
    COMEX_SAFE(comex_create_mutexes(1));
    
    /* enable messages */
    shmemi_elapsed_clock_init ();
    shmemi_tracers_init ();

    /* who am I? */
    shmemi_executable_init ();

    /* find global symbols */
    shmemi_symmetric_globalvar_table_init ();

    /* handle the heap */
    shmemi_symmetric_memory_init ();

    /* which message/trace levels are active */
    shmemi_maybe_tracers_show_info ();
    shmemi_tracers_show ();

    /* set up the atomic ops handling */
    shmemi_atomic_init ();

    /* initialize collective algs */
    shmemi_barrier_dispatch_init ();
    shmemi_barrier_all_dispatch_init ();
    shmemi_broadcast_dispatch_init ();
    shmemi_collect_dispatch_init ();
    shmemi_fcollect_dispatch_init ();

    /* register shutdown handler */
    if (EXPR_UNLIKELY (atexit (shmemi_comms_finalize) != 0)) {
        shmemi_trace (SHMEM_LOG_FATAL,
                      "internal error: cannot register"
                      " OpenSHMEM finalize handler");
        /* NOT REACHED */
    }

    SET_STATE (pe_status, PE_RUNNING);

    /* Up and running! */
}

/* mcs-lock.c */

/*
 * ------------------------------------------------------------------
 *
 * Low-level lock routines
 *
 */

/*
 *    Copyright (c) 1996-2002 by Quadrics Supercomputers World Ltd.
 *    Copyright (c) 2003-2005 by Quadrics Ltd.
 *
 *    For licensing information please see the supplied COPYING file
 *
 */

/**
 * Implement the CRAY SHMEM locking API using MCS locks
 *
 * Mellor-Crummey & Scott, Algorithms for scalable synchronisation on
 * shared-memory multiprocessors ACM Trans. Computer Systems, 1991
 *
 * With CRAY SHMEM locks we are given an 8-byte global symmetric
 * object. This memory is pre-initialised to zero in all processes.
 *
 * We split this lock memory into two 32-bit halves where each half
 * then represents a SHMEM_LOCK.  The SHMEM_LOCK struct consists of a
 * 16-bit boolean flag (locked) and a 16-bit vp (next)
 *
 * One vp is chosen to the global lock owner process and here the 1st
 * SHMEM_LOCK acts as the 'tail' of a globally distributed linked
 * list.  In all processes the 2nd SHMEM_LOCK is used to hold and
 * manage the distributed linked list state.
 */
typedef struct
{
    union
    {
        struct
        {
            volatile uint16_t locked;   /* boolean to indicate current state
                                           of lock */
            volatile int16_t next;  /* vp of next requestor */
        } s;
        volatile uint32_t word;
    } u;
#define l_locked        u.s.locked
#define l_next          u.s.next
#define l_word          u.word
} SHMEM_LOCK;

enum
{
    SHMEM_LOCK_FREE = -1,
    SHMEM_LOCK_RESET,
    SHMEM_LOCK_SET
};

/* Macro to map lock virtual address to owning process vp */
#define LOCK_OWNER(LOCK) ( ((uintptr_t)(LOCK) >> 3) % (GET_STATE (numpes)) )

static inline void
shmemi_comms_lock_acquire (SHMEM_LOCK * node, SHMEM_LOCK * lock, int this_pe)
{
    SHMEM_LOCK tmp;
    long locked;
    int prev_pe;

    node->l_next = SHMEM_LOCK_FREE;

    /* Form our lock request (integer) */
    tmp.l_locked = 1;
    tmp.l_next = this_pe;

    LOAD_STORE_FENCE ();

    /*
     * Swap this_pe into the global lock owner, returning previous
     * value, atomically
     */
    tmp.l_word =
        shmem_int_swap ((int *) &lock->l_word, tmp.l_word, LOCK_OWNER (lock));

    /* Translate old (broken) default lock state */
    if (tmp.l_word == SHMEM_LOCK_FREE) {
        tmp.l_word = SHMEM_LOCK_RESET;
    }

    /* Extract the global lock (tail) state */
    prev_pe = tmp.l_next;
    locked = tmp.l_locked;

    /* Is the lock held by someone else ? */
    if (locked) {
        /*
         * This flag gets cleared (remotely) once the lock is dropped
         */
        node->l_locked = 1;

        LOAD_STORE_FENCE ();

        /*
         * I'm now next in global linked list, update l_next in the
         * prev_pe process with our vp
         */
        shmem_short_p ((short *) &node->l_next, this_pe, prev_pe);

        /* Wait for flag to be released */
        while (!(node->l_locked));
        
    }
}

static inline void
shmemi_comms_lock_release (SHMEM_LOCK * node, SHMEM_LOCK * lock, int this_pe)
{
    /* Is there someone on the linked list ? */
    if (node->l_next == SHMEM_LOCK_FREE) {
        SHMEM_LOCK tmp;

        /* Form the remote atomic compare value (int) */
        tmp.l_locked = 1;
        tmp.l_next = this_pe;

        /*
         * If global lock owner value still equals this_pe, load RESET
         * into it & return prev value
         */
        tmp.l_word = shmem_int_cswap ((int *) &lock->l_word,
                                      tmp.l_word,
                                      SHMEM_LOCK_RESET, LOCK_OWNER (lock));

        if (tmp.l_next == this_pe) {
            /* We were still the only requestor, all done */
            return;
        }

        /*
         * Somebody is about to chain themself off us, wait for them to do it.
         *
         * Quadrics: we have seen l_next being written as two individual
         * bytes here when when the usercopy device is active, poll for
         * it being valid as well as it being set to ensure both bytes
         * are written before we try to use its value below.
         *
         */
                BLOCKUNTIL (!
                           ((node->l_next == SHMEM_LOCK_FREE) ||
                            (node->l_next < 0))
                           );
    }

    /* Be more strict about the test above, this memory consistency problem is
       a tricky one */
    BLOCKUNTIL (!(node->l_next < 0));

    /*
     * Release any waiters on the linked list
     */

    shmem_short_p ((short *) &node->l_locked, 0, node->l_next);
}


/*
 * I am not sure this is strictly correct. The Cray man pages suggest
 * that this routine should not block. With this implementation we could
 * race with another PE doing the same and then block in the acquire
 * Perhaps a conditional swap at the beginning would fix it ??
 *
 * (addy 12.10.05)
 */
static inline int
shmemi_comms_lock_test (SHMEM_LOCK * node, SHMEM_LOCK * lock, int this_pe)
{
    SHMEM_LOCK tmp;
    int retval;

    /* Read the remote global lock value */
    tmp.l_word = shmem_int_g ((int *) &lock->l_word, LOCK_OWNER (lock));

    /* Translate old (broken) default lock state */
    if (tmp.l_word == SHMEM_LOCK_FREE)
        tmp.l_word = SHMEM_LOCK_RESET;

    /* If lock already set then return 1, otherwise grab the lock & return 0 */
    if (tmp.l_word == SHMEM_LOCK_RESET) {
        shmemi_comms_lock_acquire (node, lock, this_pe);
        retval = 0;
    }
    else {
        retval = 1;
    }

    return retval;
}
/* end: mcs-lock.c */
