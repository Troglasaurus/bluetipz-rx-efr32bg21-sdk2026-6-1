/***************************************************************************//**
 * @file    version.h
 * @brief   Application version information
 *******************************************************************************
 * <b>Copyright 2022 i3 Product Development</b>
 *******************************************************************************
 *
 ******************************************************************************/
#ifndef _VERSION_H_
#define _VERSION_H_

/**************************************************************************//**
 * #include
 *****************************************************************************/
#include "common.h"

/**************************************************************************//**
 * #define
 *****************************************************************************/
#define BUILD_DATE                      "11/03/2022"

#define APP_VERSION                     "03.00.04"
#define FW_VERSION                      "2212"
#define DELIM                           "========="


/**************************************************************************//**
 * #typedef struct
 *****************************************************************************/
typedef struct
{
    char            prefix[10];
    char            app_version[10];
    char            fw_version[10];
    char            macro_date[15];
    char            suffix[10];
} version_into_t;


/**************************************************************************//**
 * Local function prototypes
 *****************************************************************************/

// Enables a firmware binary search to locate the information surrounded by
// the delimiters.
static const version_into_t version_info = {
    .prefix         = DELIM,
    .app_version    = APP_VERSION,
    .fw_version     = FW_VERSION,
    .macro_date     = BUILD_DATE,
    .suffix         = DELIM,
};


/**************************************************************************//**
 * Simple functions
 *****************************************************************************/
const char * get_app_version(void) { return version_info.app_version; }
const char * get_fw_version(void) { return version_info.fw_version; }

void check_version_info(void)
{
    // Make sure text loaded into the structure fits properly.
    APP_ASSERT (strlen(version_info.prefix)     <= sizeof(version_info.prefix)      );
    APP_ASSERT (strlen(version_info.app_version)<= sizeof(version_info.app_version) );
    APP_ASSERT (strlen(version_info.fw_version) <= sizeof(version_info.fw_version)  );
    APP_ASSERT (strlen(version_info.macro_date) <= sizeof(version_info.macro_date)  );
    APP_ASSERT (strlen(version_info.suffix)     <= sizeof(version_info.suffix)      );

}
void print_version_info(void)
{
    DBPRINT("\n%s%s\n%s %s\n%s\n%s%s\n\n",
            version_info.prefix,        version_info.prefix,
            version_info.app_version,   version_info.fw_version,
            version_info.macro_date,
            version_info.suffix,        version_info.suffix);
};


#endif      // ndef _VERSION_H_
