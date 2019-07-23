/*
 * dbgu.h
 *
 *  Created on: Jul 23, 2019
 *      Author: michael
 */

#ifndef DRIVERS_DBGU_H_
#define DRIVERS_DBGU_H_

#include <stdint.h>
#include "state.h"

#if (DBGU)
void _init_usart_dbg();
#endif

#endif /* DRIVERS_DBGU_H_ */
