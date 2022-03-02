#ifndef _CRC32_H
#define _CRC32_H
/*******************************************************************************
 * (c) Copyright 2012-2019 Norbit Subsea. All rights reserved.                 
 *******************************************************************************/
/* This file has been prepared for Doxygen automatic documentation generation. */
/****************************************************************************//**
* @file crc32.h
*
* @brief 32 bit crc implementation
* @author Oct 15, 2000 Matt Domsch <Matt_Domsch@dell.com>
* Nicer crc32 functions/docs submitted by linux@horizon.com.  Thanks!
* Code was from the public domain, copyright abandoned.  Code was
* subsequently included in the kernel, thus was re-licensed under the
* GNU GPL v2.
*
* Oct 12, 2000 Matt Domsch <Matt_Domsch@dell.com>
* Same crc32 function was used in 5 other places in the kernel.
* I made one version, and deleted the others.
* There are various incantations of crc32().  Some use a seed of 0 or ~0.
* Some xor at the end with ~0.  The generic crc32() function takes
* seed as an argument, and doesn't xor at the end.  Then individual
* users can do whatever they need.
*   drivers/net/smc9194.c uses seed ~0, doesn't xor with ~0.
*   fs/jffs2 uses seed 0, doesn't xor with ~0.
*   fs/partitions/efi.c uses seed ~0, xor's with ~0.
*
* This source code is licensed under the GNU General Public License,
* Version 2.  See the file COPYING for more details.
*
*/
/** @cond */
#include <stdint.h>
#ifdef __unix__
#include <linux/types.h>
#endif
/** @endcond */


uint32_t  crc32_le(uint32_t crc, unsigned char const *p, uint32_t len);

/** Wrapper pointing crc32 to crc32_le */
#define crc32(seed, data, length)  crc32_le(seed, (unsigned char const *)data, length)

#endif 
