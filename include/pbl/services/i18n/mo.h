/*-
 * Copyright (c) 2000, 2001 Citrus Project,
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#pragma once

#include "pbl/util/attributes.h"

/* ==============
 * MO File Format            *
 * ==============
 *
          byte
               +------------------------------------------+
            0  | magic number = 0x950412de                |
               |                                          |
            4  | file format revision = 0                 |
               |                                          |
            8  | number of strings                        |  == N
               |                                          |
           12  | offset of table with original strings    |  == O
               |                                          |
           16  | offset of table with translation strings |  == T
               |                                          |
           20  | size of hashing table                    |  == S
               |                                          |
           24  | offset of hashing table                  |  == H
               |                                          |
               .                                          .
               .    (possibly more entries later)         .
               .                                          .
               |                                          |
            O  | length & offset 0th string  ----------------.
        O + 8  | length & offset 1st string  ------------------.
                ...                                    ...   | |
  O + ((N-1)*8)| length & offset (N-1)th string           |  | |
               |                                          |  | |
            T  | length & offset 0th translation  ---------------.
        T + 8  | length & offset 1st translation  -----------------.
                ...                                    ...   | | | |
  T + ((N-1)*8)| length & offset (N-1)th translation      |  | | | |
               |                                          |  | | | |
            H  | start hash table                         |  | | | |
                ...                                    ...   | | | |
    H + S * 4  | end hash table                           |  | | | |
               |                                          |  | | | |
               | NUL terminated 0th string  <----------------' | | |
               |                                          |    | | |
               | NUL terminated 1st string  <------------------' | |
               |                                          |      | |
                ...                                    ...       | |
               |                                          |      | |
               | NUL terminated 0th translation  <---------------' |
               |                                          |        |
               | NUL terminated 1st translation  <-----------------'
               |                                          |
                ...                                    ...
               |                                          |
               +------------------------------------------+
 *
 */

#define MO_MAGIC    0x950412de
#define MO_GET_REV_MAJOR(r)  (((r) >> 16) & 0xFFFF)
#define MO_GET_REV_MINOR(r)  ((r) & 0xFFFF)
#define MO_MAKE_REV(maj, min)  (((maj) << 16) | (min))

#define LANG_PROP_NAME "Language: "

/* *.mo file format */
typedef struct PACKED {
  uint32_t mo_magic;  /* determines endian */
  uint32_t mo_revision;  /* file format revision: 0 */
  uint32_t mo_nstring;  /* N: number of strings */
  uint32_t mo_otable;  /* O: original text table offset */
  uint32_t mo_ttable;  /* T: translated text table offset */
  uint32_t mo_hsize;  /* S: size of hashing table */
  uint32_t mo_hoffset;  /* H: offset of hashing table */
} MoHeader;

typedef struct PACKED {
  uint32_t len;    /* strlen(str), so region will be len + 1 */
  uint32_t off;    /* offset of \0-terminated string */
} MoEntry;

typedef struct {
  MoHeader hdr;
  char *mo_lang;
  uint32_t *mo_htable;  /* H: hash table */
} Mo;

typedef struct {
  size_t len;
  Mo mo;    /* endian-flipped mo file header */
} MoHandle;

