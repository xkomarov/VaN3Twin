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
 * Run-time support for Information Object Classes.
 * Copyright (c) 2017 Lev Walkin <vlm@lionet.info>. All rights reserved.
 * Redistribution and modifications are permitted subject to BSD license.
 */
#ifndef	ASN_IOC_H
#define	ASN_IOC_H

#include "asn_system.h"		/* Platform-specific types */

#ifdef __cplusplus
extern "C" {
#endif

struct asn_TYPE_descriptor_s;
struct asn_ioc_cell_s;

/*
 * X.681, #13
 */
typedef struct asn_ioc_set_s {
    size_t rows_count;
    size_t columns_count;
    const struct asn_ioc_cell_s *rows;
} asn_ioc_set_t;


typedef struct asn_ioc_cell_s {
    const char *field_name; /* Is equal to corresponding column_name */
    enum {
        aioc__undefined = 0,
        aioc__value,
        aioc__type,
        aioc__open_type,
    } cell_kind;
    struct asn_TYPE_descriptor_s *type_descriptor;
    const void *value_sptr;
    struct {
        size_t types_count;
        struct {
            unsigned choice_position;
        } *types;
    } open_type;
} asn_ioc_cell_t;


#ifdef __cplusplus
}
#endif

#endif	/* ASN_IOC_H */
