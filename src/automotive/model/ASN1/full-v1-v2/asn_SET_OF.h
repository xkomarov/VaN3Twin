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
#ifndef	ASN_SET_OF_H
#define	ASN_SET_OF_H

#ifdef __cplusplus
#define A_SET_OF(type)                   \
    struct {                             \
        type **array;                    \
        int count; /* Meaningful size */ \
        int size;  /* Allocated size */  \
        void (*free)(decltype(*array));  \
    }
#else   /* C */
#define A_SET_OF(type)                   \
    struct {                             \
        type **array;                    \
        int count; /* Meaningful size */ \
        int size;  /* Allocated size */  \
        void (*free)(type *);    \
    }
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define	ASN_SET_ADD(headptr, ptr)		\
	asn_set_add((headptr), (ptr))

/*******************************************
 * Implementation of the SET OF structure.
 */

/*
 * Add another structure into the set by its pointer.
 * RETURN VALUES:
 * 0 for success and -1/errno for failure.
 */
int  asn_set_add(void *asn_set_of_x, void *ptr);

/*
 * Delete the element from the set by its number (base 0).
 * This is a constant-time operation. The order of elements before the
 * deleted ones is guaranteed, the order of elements after the deleted
 * one is NOT guaranteed.
 * If _do_free is given AND the (*free) is initialized, the element
 * will be freed using the custom (*free) function as well.
 */
void asn_set_del(void *asn_set_of_x, int number, int _do_free);

/*
 * Empty the contents of the set. Will free the elements, if (*free) is given.
 * Will NOT free the set itself.
 */
void asn_set_empty(void *asn_set_of_x);

/*
 * Cope with different conversions requirements to/from void in C and C++.
 * This is mostly useful for support library.
 */
typedef A_SET_OF(void) asn_anonymous_set_;
#define _A_SET_FROM_VOID(ptr)		((asn_anonymous_set_ *)(ptr))
#define _A_CSET_FROM_VOID(ptr)		((const asn_anonymous_set_ *)(ptr))

#ifdef __cplusplus
}
#endif

#endif	/* ASN_SET_OF_H */
