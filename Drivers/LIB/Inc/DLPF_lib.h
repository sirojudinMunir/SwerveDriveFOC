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
  float result, alpha;
}DLPF_HandleTypeDef;

void dlpf_set_alpha (DLPF_HandleTypeDef *hdlpf, float alpha_val);
float dlpf_get_result (DLPF_HandleTypeDef *hdlpf, float raw_data);

#endif /* LIB_INC_DLPF_LIB_H_ */
