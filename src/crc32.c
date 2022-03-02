/*******************************************************************************
 * (c) Copyright 2012-2019 Norbit Subsea. All rights reserved.                 
 *******************************************************************************/
/* This file has been prepared for Doxygen automatic documentation generation. */
/****************************************************************************//**
* @file crc32.c
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
#include <linux/kernel.h>
#endif
/** @endcond */

#include "crc32.h"
#include "crc32table.h"

//MODULE_AUTHOR("Matt Domsch <Matt_Domsch@dell.com>");
//MODULE_DESCRIPTION("Ethernet CRC32 calculations");
//MODULE_LICENSE("GPL");

#define unlikely(x) x

static inline uint32_t
crc32_body(uint32_t crc, unsigned char const *buf, uint32_t len, const uint32_t *tab)
{
#  define DO_CRC(x) crc = tab[(crc ^ (x)) & 255 ] ^ (crc >> 8)
	const uint32_t *b;
	uint32_t    rem_len;

	/* Align it */
	if (unlikely((long)buf & 3 && len)) {
		do {
			DO_CRC(*buf++);
		} while ((--len) && ((long)buf)&3);
	}
	rem_len = len & 3;
	/* load data 32 bits wide, xor data 32 bits wide. */
	len = len >> 2;
	b = (const uint32_t *)buf;
	for (--b; len; --len) {
		crc ^= *++b; /* use pre increment for speed */
		DO_CRC(0);
		DO_CRC(0);
		DO_CRC(0);
		DO_CRC(0);
	}
	len = rem_len;
	/* And the last few bytes */
	if (len) {
		uint8_t *p = (uint8_t *)(b + 1) - 1;
		do {
			DO_CRC(*++p); /* use pre increment for speed */
		} while (--len);
	}
	return crc;
#undef DO_CRC
}


/***************************************************************************//**
* @brief Little endian CRC32 implementation
*
* @param[in]    crc     CRC seed
* @param[in]    p       Data, pointer to char array
* @param[in]    len     Length, in bytes of char array p
* @return               Calculated CRC value
******************************************************************************/
uint32_t  crc32_le(uint32_t crc, unsigned char const *p, uint32_t len)
{
	const uint32_t      *tab = crc32table_le;

	//crc = __cpu_to_le32(crc);
	crc = crc ^ 0xffffffffL;
	crc = crc32_body(crc, p, len, tab);
	//return __le32_to_cpu(crc);
	return crc ^ 0xffffffffL;
}



