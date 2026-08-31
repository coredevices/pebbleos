/*
 * tinflate  -  tiny inflate
 *
 * Copyright (c) 2003 by Joergen Ibsen / Jibz
 * All Rights Reserved
 * http://www.ibsensoftware.com/
 *
 * Copyright (c) 2014 by Paul Sokolovsky
 *
 * Copyright (c) 2015 by Pebble Inc.
 *
 * This software is provided 'as-is', without any express
 * or implied warranty.  In no event will the authors be
 * held liable for any damages arising from the use of
 * this software.
 *
 * Permission is granted to anyone to use this software
 * for any purpose, including commercial applications,
 * and to alter it and redistribute it freely, subject to
 * the following restrictions:
 *
 * 1. The origin of this software must not be
 *    misrepresented; you must not claim that you
 *    wrote the original software. If you use this
 *    software in a product, an acknowledgment in
 *    the product documentation would be appreciated
 *    but is not required.
 *
 * 2. Altered source versions must be plainly marked
 *    as such, and must not be misrepresented as
 *    being the original software.
 *
 * 3. This notice may not be removed or altered from
 *    any source distribution.
 */

/* Version history:
   1.0  14 Nov 2003  Public release
   1.1  21 Oct 2014  Added pre-computed huffman values, destination grow callback
   1.2  14 Dec 2015  Moved TINF_DATA to heap to avoid overflowing small embedded stack
                     Removed runtime value generation (now only pre-computed values)
                     Removed destination grow callback
 */

#ifndef TINFLATE_H_INCLUDED
#define TINFLATE_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#define TINF_OK              0
#define TINF_MEMORY_ERROR  (-1)
#define TINF_DATA_ERROR    (-3)
#define TINF_DEST_OVERFLOW (-4)

int tinflate_uncompress(void *dest, unsigned int *destLen,
                        const void *source, unsigned int sourceLen);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TINF_H_INCLUDED */
