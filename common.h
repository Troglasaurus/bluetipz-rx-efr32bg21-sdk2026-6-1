/***************************************************************************//**
 * @file    common.h
 * @brief   Common content
 *******************************************************************************
 * <b>Copyright 2022 i3 Product Development</b>
 *******************************************************************************
 *
 ******************************************************************************/
#ifndef _COMMON_H_
#define _COMMON_H_

/**************************************************************************//**
 * #include
 *****************************************************************************/
#include <stdio.h>
#include <stdint.h>

#include "app_assert.h"

/**************************************************************************//**
 * #define
 *****************************************************************************/
#define DBPRINT                 printf

/**************************************************************************//**
 * #define macros
 *****************************************************************************/
/*
 * DEBUG RESET LOG - TEMPORARY, REMOVE (comment out) BEFORE A CLIENT RELEASE.
 * Records reset causes, watchdog-warning PC/LR and the last failed assert in
 * NVM, and shows them in the xgatt_name/owner/email/phone characteristics.
 * See "Debug reset log" in app.c.
 */
#define BT_DEBUG_RESET_LOG

#ifdef BT_DEBUG_RESET_LOG
void debug_record_assert(uint32_t line, uint32_t status);
#define DEBUG_RECORD_ASSERT(line, status)   debug_record_assert((line), (uint32_t)(status))
#warning "BT_DEBUG_RESET_LOG is enabled - disable before client release"
#else
#define DEBUG_RECORD_ASSERT(line, status)
#endif

#define APP_ASSERT(condition)   if( !(condition) )                               \
                                {                                                \
                                    DBPRINT("Config assert %s (line %d)\r\n",    \
                                            __FILE__, __LINE__);                 \
                                    DEBUG_RECORD_ASSERT(__LINE__, 0xFFFF);       \
                                }                                                \
                                app_assert( (condition) )

#define APP_ASSERT_STATUS(sc)   if( (sc) )                                       \
                                {                                                \
                                    DBPRINT("ASSERT  %s %d sc = %d\r\n",         \
                                            __FILE__, __LINE__, (uint16_t)sc);   \
                                    DEBUG_RECORD_ASSERT(__LINE__, (sc));         \
                                }                                                \
                                app_assert_status( (sc) )
                                
#endif  // ndef _VERSION_H_                                