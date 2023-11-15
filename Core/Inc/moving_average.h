/*
 * moving_average.h
 *
 *  Created on: Oct 12, 2023
 *      Author: ADMIN
 */

#ifndef INC_MOVING_AVERAGE_H_
#define INC_MOVING_AVERAGE_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Definitions ---------------------------------------------------------------*/
#define WindowLength 10

/* TypeDefs ------------------------------------------------------------------*/
typedef struct{
	uint32_t History[WindowLength]; /*Array to store values of filter window*/
	uint32_t Sum;	/* Sum of filter window's elements*/
	uint32_t WindowPointer; /* Pointer to the first element of window*/
}FilterTypeDef;

/* Function prototypes -------------------------------------------------------*/
void Moving_Average_Init(FilterTypeDef* filter_struct);
uint32_t Moving_Average_Compute(uint32_t raw_data, FilterTypeDef* filter_struct);

#endif /* INC_MOVING_AVERAGE_H_ */
