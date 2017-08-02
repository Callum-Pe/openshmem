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
 * all the non-inlined bits that need to be shared per-PE
 *
 */
#include <comex.h>
#include <stdint.h>
/**
 * This file provides the layer on top of GASNet, ARMCI or whatever.
 * API should be formalized at some point.
 */

#include "comms-shared.h"

/**
 * set up segment/symmetric handling
 *
 */

void **heapptr;  
comex_group_t shmemgroup = COMEX_GROUP_WORLD;
uintptr_t heapsize;

static comex_request_t nexthdl = 0;
struct hdl_lst_elem head = {NULL ,NULL};
hdl_lst_elem* tail = &head;
comex_request_t* newhdl()
{
  tail->next = (hdl_lst_elem*) malloc(sizeof(hdl_lst_elem));
  tail = tail->next;
  tail->next = NULL;
  tail->request = (comex_request_t*) malloc(sizeof(hdl_lst_elem));
  *(tail->request) = nexthdl++;
  return tail->request;
}  
void clearhdls()
{
  comex_wait_all(shmemgroup);	 
  hdl_lst_elem* temp = head.next;
  hdl_lst_elem* temp2;
  while(temp != NULL)
  {
    free(temp->request);
    temp2 = temp->next;
    free(temp);
    temp = temp2;
  } 
  tail = &head;
  nexthdl =0;
}

/**
 * global barrier counters
 */

long barcount = 0;
int barflag = 0;
