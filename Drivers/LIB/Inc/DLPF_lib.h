/*
 * DLPF_lib.h
 *
 *  Created on: Dec 19, 2023
 *      Author: munir
 */

#ifndef LIB_INC_DLPF_LIB_H_
#define LIB_INC_DLPF_LIB_H_


typedef struct
{
  double result, alpha;
}DLPF_HandleTypeDef;

void dlpf_set_alpha (DLPF_HandleTypeDef *hdlpf, double alpha_val);
double dlpf_get_result (DLPF_HandleTypeDef *hdlpf, double raw_data);

#endif /* LIB_INC_DLPF_LIB_H_ */
