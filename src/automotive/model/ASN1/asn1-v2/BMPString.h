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
 * Copyright (c) 2003-2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	_BMPString_H_
#define	_BMPString_H_

#include "OCTET_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef OCTET_STRING_t BMPString_t;  /* Implemented via OCTET STRING */

extern asn_TYPE_descriptor_t asn_DEF_BMPString;
extern asn_TYPE_operation_t asn_OP_BMPString;
extern asn_OCTET_STRING_specifics_t asn_SPC_BMPString_specs;

#if !defined(ASN_DISABLE_PRINT_SUPPORT) || !defined(ASN_DISABLE_XER_SUPPORT)
ssize_t BMPString__dump(const BMPString_t *st, asn_app_consume_bytes_f *cb, void *app_key);
#endif  /* !defined(ASN_DISABLE_PRINT_SUPPORT) || !defined(ASN_DISABLE_XER_SUPPORT) */

#define BMPString_free OCTET_STRING_free

#if !defined(ASN_DISABLE_PRINT_SUPPORT)
asn_struct_print_f BMPString_print;	/* Human-readable output */
#endif  /* !defined(ASN_DISABLE_PRINT_SUPPORT) */

#define BMPString_compare OCTET_STRING_compare

asn_constr_check_f BMPString_constraint;

#if !defined(ASN_DISABLE_BER_SUPPORT)
#define BMPString_decode_ber OCTET_STRING_decode_ber
#define BMPString_encode_der OCTET_STRING_encode_der
#endif  /* !defined(ASN_DISABLE_BER_SUPPORT) */

#if !defined(ASN_DISABLE_XER_SUPPORT)
xer_type_decoder_f BMPString_decode_xer;
xer_type_encoder_f BMPString_encode_xer;
#endif  /* !defined(ASN_DISABLE_XER_SUPPORT) */

#if !defined(ASN_DISABLE_JER_SUPPORT)
jer_type_encoder_f BMPString_encode_jer;
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */

#if !defined(ASN_DISABLE_UPER_SUPPORT)
#define BMPString_decode_uper OCTET_STRING_decode_uper
#define BMPString_encode_uper OCTET_STRING_encode_uper
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) */
#if !defined(ASN_DISABLE_APER_SUPPORT)
#define BMPString_decode_aper OCTET_STRING_decode_aper
#define BMPString_encode_aper OCTET_STRING_encode_aper
#endif  /* !defined(ASN_DISABLE_APER_SUPPORT) */

#if !defined(ASN_DISABLE_RFILL_SUPPORT)
#define BMPString_random_fill OCTET_STRING_random_fill
#endif  /* !defined(ASN_DISABLE_RFILL_SUPPORT) */

#ifdef __cplusplus
}
#endif

#endif	/* _BMPString_H_ */
