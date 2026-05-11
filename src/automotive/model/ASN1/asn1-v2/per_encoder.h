/* ============================================================================
 * Research Project: Data communication in the environment of
intelligent cars
 * Author: Kirill Komarov
 * Date: 2026
 * 
 * Description:
 * This file contains source code developed (or modified) as part of the 
 * research for the paper: "Data communication in the environment of
intelligent cars".
 * 
 * DISCLAIMER & ACKNOWLEDGEMENT:
 * Please note that this file contains or may contain code fragments, 
 * algorithms, or architectural solutions that were previously implemented 
 * in the "VaN3Twin" project https://github.com/DriveX-devs/VaN3Twin.git.
 * 
 * The borrowed code has been adapted and is used strictly for academic 
 * and research purposes. All rights to the original code segments belong 
 * to their respective original authors.
 * ============================================================================ */
/*-
 * Copyright (c) 2006-2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	_PER_ENCODER_H_
#define	_PER_ENCODER_H_

#include "asn_application.h"
#include "per_support.h"

#ifdef __cplusplus
extern "C" {
#endif

struct asn_TYPE_descriptor_s;	/* Forward declaration */

/*
 * Type of the generic PER encoder function.
 */
typedef asn_enc_rval_t(per_type_encoder_f)(
    const struct asn_TYPE_descriptor_s *type_descriptor,
    const asn_per_constraints_t *constraints, const void *struct_ptr,
    asn_per_outp_t *per_output);

int ignore_output(const void *data, size_t size, void *app_key);

typedef struct enc_dyn_arg {
    void *buffer;
    size_t length;
    size_t allocated;
} enc_dyn_arg;
int encode_dyn_cb(const void *buffer, size_t size, void *key);

#ifdef __cplusplus
}
#endif

#endif	/* _PER_ENCODER_H_ */
