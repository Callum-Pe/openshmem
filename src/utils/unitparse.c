/*
 *
 * Copyright (c) 2011 - 2013
 *   University of Houston System and Oak Ridge National Laboratory.
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * o Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * 
 * o Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 
 * o Neither the name of the University of Houston System, Oak Ridge
 *   National Laboratory nor the names of its contributors may be used to
 *   endorse or promote products derived from this software without specific
 *   prior written permission.
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


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/types.h>

/**
 * define accepted size units in ascending order, which are
 * S.I. compliant
 *
 * http://en.wikipedia.org/wiki/SI_Unit_Prefixes
 *
 */

static char *units_string = "kmgtpezy";
static const size_t multiplier = 1000;

/**
 * Take a scaling unit and work out its numeric value.
 *
 * Return scale value if known, otherwise -1
 *
 */

static size_t
parse_unit (char u)
{
  int foundit = 0;
  char *usp = units_string;
  size_t bytes = 1;
 
  u = tolower (u);
  while (*usp != '\0')
    {
      bytes *= multiplier;
      if (*usp == u)
	{
	  foundit = 1;
	  break;
	  /* NOT REACHED */
	}
      usp += 1;
    }
 
  return foundit ? bytes : -1;
}


/**
 * segment size can be expressed with scaling units.  Parse those.
 *
 * Return segemnt size, scaled where necessary by unit, or -1
 * if we couldn't parse it.
 */

size_t
__shmem_parse_size (char *size_str)
{
  char unit = '\0';
  size_t ret = 0;
  char *p;

  p = size_str;
  while (*p != '\0')
    {
      if (! isdigit (*p))
	{
	  unit = *p;
	  break;
	  /* NOT REACHED */
	}

      ret = ret * 10 + (*p - '0'); /* works for ASCII/EBCDIC */
      p += 1;
    }

  /* if no unit, we already have value.  Otherwise, do scaling */
  if (unit == '\0')
    {
      return ret;
    }
  else
    {
      const size_t bytes = parse_unit (unit);
      return (bytes != -1) ? bytes * ret : -1;
    }
}