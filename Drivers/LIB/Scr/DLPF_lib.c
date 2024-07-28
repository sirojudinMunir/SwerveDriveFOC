/*
 * DLPF_lib.c
 *
 *  Created on: Dec 19, 2023
 *      Author: munir
 */

#include "DLPF_lib.h"

//============================================================================================

void dlpf_set_alpha (DLPF_HandleTypeDef *hdlpf, double alpha_val)
{
	hdlpf->alpha = alpha_val;
}

//============================================================================================

double dlpf_get_result (DLPF_HandleTypeDef *hdlpf, double raw_data)
{
	hdlpf->result = hdlpf->result * (1.0 - hdlpf->alpha) + raw_data * hdlpf->alpha;
	return hdlpf->result;
}

//============================================================================================
