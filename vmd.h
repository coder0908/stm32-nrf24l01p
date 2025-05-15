/*
 * vmd.h
 *
 *  Created on: May 5, 2025
 *      Author: coder0908
 */

#ifndef __VMD_H__
#define __VMD_H__

#include <stdbool.h>

enum vmd_ret {
	VMD_SUCC = 0,	//success
	VMD_FAIL,	//fail
	VMD_ERR	//communication failed
};

void vmd_fail_handler(void);

#define VMD_IS_ERROR(vmd_return_value)	(vmd_return_value == VMD_ERR:true:false)

#define VMD_USE_FULL_ASSERT 1

#if VMD_USE_FULL_ASSERT
#define VMD_ASSERT_PARAM(expression)	((expression)?(void*)0:vmd_fail_handler())
#else
#define VMD_ASSERT_PARAM(expression)	((void*)0)
#endif


#ifndef IS_NULL
#define IS_NULL(expression)	(expression == NULL?true:false)
#endif

#ifndef NOT_NULL
#define NOT_NULL(expression)	(expression != NULL?true:false)
#endif

#endif /* __VMD_H__ */
