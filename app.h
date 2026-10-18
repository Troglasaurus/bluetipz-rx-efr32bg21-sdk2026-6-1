/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include <stdbool.h>

/*
 * SiSDK 2026.6.1 port (BlueTipz Rx):
 * The five functions below come from the Simplicity Studio 6 SoC-Empty
 * template. They are implemented in app_bm.c (component app_os_helper);
 * app_init_bt() is called by the generated sl_event_handler.c during
 * sl_main_init(). The BlueTipz application does not use them.
 *
 * app_init() and app_process_action() are implemented in app.c and must NOT
 * be SL_WEAK: sl_main_init.c already provides weak empty versions.
 */
void app_proceed(void);
bool app_is_process_required(void);
bool app_mutex_acquire(void);
void app_mutex_release(void);
void app_init_bt(void);

/*
 * sl_main / Power Manager hooks implemented in app.c to keep the 4.3.1 Rx
 * main() behavior (see app.c).
 */
void app_init_pre_clock(void);
bool app_is_ok_to_sleep(void);

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

#endif // APP_H
