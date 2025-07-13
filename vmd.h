/*
 * vmd.h
 *
 *  Created on: May 5, 2025
 *      Author: coder0908
 */

#ifndef __VMD_H__
#define __VMD_H__

#include <stdbool.h>


void vmd_fail_handler(void);


#define VMD_USE_FULL_ASSERT 1

#if VMD_USE_FULL_ASSERT
#define VMD_ASSERT_PARAM(expression)	((expression)?(void*)0:vmd_fail_handler())
#else
#define VMD_ASSERT_PARAM(expression)	((void*)0)
#endif

#endif /* __VMD_H__ */

