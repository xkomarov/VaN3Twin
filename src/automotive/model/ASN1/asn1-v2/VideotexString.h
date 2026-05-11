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
#ifndef	_VideotexString_H_
#define	_VideotexString_H_

#include "OCTET_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef OCTET_STRING_t VideotexString_t;  /* Implemented via OCTET STRING */

extern asn_TYPE_descriptor_t asn_DEF_VideotexString;
extern asn_TYPE_operation_t asn_OP_VideotexString;

#define VideotexString_free OCTET_STRING_free

#if !defined(ASN_DISABLE_PRINT_SUPPORT)
#define VideotexString_print OCTET_STRING_print
#endif  /* !defined(ASN_DISABLE_PRINT_SUPPORT) */

#define VideotexString_compare OCTET_STRING_compare

#define VideotexString_constraint asn_generic_unknown_constraint

#if !defined(ASN_DISABLE_BER_SUPPORT)
#define VideotexString_decode_ber OCTET_STRING_decode_ber
#define VideotexString_encode_der OCTET_STRING_encode_der
#endif  /* !defined(ASN_DISABLE_BER_SUPPORT) */

#if !defined(ASN_DISABLE_XER_SUPPORT)
#define VideotexString_decode_xer OCTET_STRING_decode_xer_hex
#define VideotexString_encode_xer OCTET_STRING_encode_xer
#endif  /* !defined(ASN_DISABLE_XER_SUPPORT) */

#if !defined(ASN_DISABLE_JER_SUPPORT)
#define VideotexString_encode_jer OCTET_STRING_encode_jer
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */

#if !defined(ASN_DISABLE_UPER_SUPPORT)
#define VideotexString_decode_uper OCTET_STRING_decode_uper
#define VideotexString_encode_uper OCTET_STRING_encode_uper
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) */
#if !defined(ASN_DISABLE_APER_SUPPORT)
#define VideotexString_decode_aper OCTET_STRING_decode_aper
#define VideotexString_encode_aper OCTET_STRING_encode_aper
#endif  /* !defined(ASN_DISABLE_APER_SUPPORT) */

#ifdef __cplusplus
}
#endif

#endif	/* _VideotexString_H_ */
