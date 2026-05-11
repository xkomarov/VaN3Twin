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
#ifndef	_BER_TLV_LENGTH_H_
#define	_BER_TLV_LENGTH_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef ssize_t ber_tlv_len_t;

/*
 * This function tries to fetch the length of the BER TLV value and place it
 * in *len_r.
 * RETURN VALUES:
 *	 0:	More data expected than bufptr contains.
 *	-1:	Fatal error deciphering length.
 *	>0:	Number of bytes used from bufptr.
 * On return with >0, len_r is constrained as -1..MAX, where -1 mean
 * that the value is of indefinite length.
 */
ssize_t ber_fetch_length(int _is_constructed, const void *bufptr, size_t size,
	ber_tlv_len_t *len_r);

/*
 * This function expects bufptr to be positioned over L in TLV.
 * It returns number of bytes occupied by L and V together, suitable
 * for skipping. The function properly handles indefinite length.
 * RETURN VALUES:
 * 	Standard {-1,0,>0} convention.
 */
ssize_t ber_skip_length(
	const struct asn_codec_ctx_s *opt_codec_ctx,	/* optional context */
	int _is_constructed, const void *bufptr, size_t size);

/*
 * This function serializes the length (L from TLV) in DER format.
 * It always returns number of bytes necessary to represent the length,
 * it is a caller's responsibility to check the return value
 * against the supplied buffer's size.
 */
size_t der_tlv_length_serialize(ber_tlv_len_t len, void *bufptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif	/* _BER_TLV_LENGTH_H_ */
