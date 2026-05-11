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
/*
 * Copyright (c) 2007-2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	_PER_OPENTYPE_H_
#define	_PER_OPENTYPE_H_

#include "asn_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

asn_dec_rval_t uper_sot_suck(
        const asn_codec_ctx_t *,
        const asn_TYPE_descriptor_t *td,
        const asn_per_constraints_t *constraints,
        void **sptr, asn_per_data_t *pd);

#ifdef __cplusplus
}
#endif

#endif	/* _PER_OPENTYPE_H_ */
