/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
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

/*
 * SiSDK 2026.6.1 port notes (BlueTipz Rx):
 *  - The Simple Timer API (sl_simple_timer.h, deprecated) is replaced by
 *    app_timer. Timer callbacks now use the app_timer_callback_t signature.
 *  - app_init()/app_process_action() are no longer SL_WEAK: sl_main already
 *    provides weak defaults, and two weak definitions would let the linker
 *    pick the empty one.
 *  - main.c is the unchanged Studio 6 SoC-Empty main(). The two ways the
 *    4.3.1 Rx main() differed from the stock one are kept with sl_main and
 *    Power Manager hooks defined right after app_process_action():
 *      app_init_pre_clock()  - EMU_UnlatchPinRetention() before any other
 *                              initialization (was the first line of main()).
 *      app_is_ok_to_sleep()  - returns false, so sl_power_manager_sleep() in
 *                              the main loop returns at once and the Rx never
 *                              sleeps (4.3.1 had that call commented out).
 *  - OTA-control write handling and the reset into the AppLoader are done
 *    solely by the in_place_ota_dfu component (the old app duplicated it and
 *    used sl_bt_system_reset(2), which no longer exists).
 *  - sl_bt_evt_scanner_scan_report (removed) is no longer handled; the legacy
 *    scanner always delivered sl_bt_evt_scanner_legacy_advertisement_report.
 *  - sl_bt_system_get_identity_address() -> sl_bt_gap_get_identity_address().
 *  - Board pins come from bluetipz_pins.h (was the Pin Tool pin_config.h).
 * See ../../rx_port_to_2026-6-1_report.md for the full list.
 */

/**************************************************************************//**
 * #include
 *****************************************************************************/
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "ustimer.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_iadc.h"
#include "em_rmu.h"
#include "em_wdog.h"
#include "efr32bg21_emu.h"
#include "gatt_db.h"
#include "gpiointerrupt.h"
#include "bluetipz_pins.h"      // port note: board pins (was pin_config.h)
#include "sl_bluetooth.h"
#include "sl_bt_api.h"
#include "app_timer.h"          // port note: replaces sl_simple_timer.h
#include "sl_power_manager.h"
#include "sl_sleeptimer.h"
#include "app.h"

#include "common.h"
#include "version.h"


/**************************************************************************//**
 * #define
 *****************************************************************************/
// Define this to get EM state transition information
//#define OUTPUT_EM_INFO

//#define CODEDPHY                        1

#define ADC_BATTERY                     0
#define ADC_TEMP                        1
#define BT_SYSTEM_HALT                  1
#define BT_SYSTEM_RESUME                0
#define CLK_ADC_FREQ                    10000000  // CLK_ADC - 10 MHz (normal mode)
#define CLK_SRC_ADC_FREQ                20000000  // CLK_SRC_ADC
#define DBPRINT                         printf
#define GAP_TYPE_CUSTOM                 0xFF
#define GAP_TYPE_SERVICE                0x07
#define GAP_TYPE_STATUS                 0xFF
#define MAX_NVM_SIZE                    32
#define MAX_NVM_VALUE_SIZE              56
#define MAX_PS_ITEM_SIZE                10
#define MAX_TX_TO_STORE                 50
#define NO_CALLBACK_DATA                (void*)NULL
#define NO_ERROR                        0
#define POWER_OFF_BLINKS                3
#define PULLDOWN                        0
#define PULLUP                          1
#define SCAN_BY_INTERVAL_AND_WINDOW     0x00
#define SCAN_BY_STOP_AND_DELAY          0x01
#define SUPPRESS_BUZZER                 0x01
#define SUPPRESS_LED                    0x02
#define SUPPRESS_STATUS                 0x04
#define SUPPRESS_VIBRATE                0x08
#define TX_POWER_MAX                    100
#define TX_POWER_MIN                    (-30)

//
// EM4 Related
//
//  - POWER button is on PC05.  Alternate function is EM4UEN pin 3, which
//		becomes bitmask 19 within the referenced registers
//  - References:
//      EFR32BG21 Datasheet PDF Table 6.2. GPIO Alternate Function Table
//      EFR32BG21 Reference Manual PDF
//          Table 32.2. EM4WU Register Bit Index to EM4WU Pin Mapping
//          Section 32.5.20 GPIO_EM4WUEN - EM4 Wake Up Enable Register
//
#define USE_EM4_ON_POWER_OFF            // controls using EM4 or pwr mgr sleep

#define EM4WU_EM4WU_X                   (7)
#define EM4WU_EM4WUEN_MASK              (1 << EM4WU_EM4WU_X)
#define EM4WUEN_BITMASK                 (EM4WU_EM4WUEN_MASK << _GPIO_EM4WUEN_EM4WUEN_SHIFT)

//
// Timing Related
//
#define SYS_TICK_MS                     (150)

#define BEACON_RELAY_CLEAR_DELAY_MS     10000
#define POWER_BTN_PRESS_DURATION_MS     1000
#define SECONDS_IN_POWER_BLINK_INTERVAL (.25)
#define SECONDS_IN_TAZ_BLINK_INTERVAL   (.5)
#define STATUS_DELAY_MS                 (30*1000)
#define STATUS_INTERVAL_MS              (15*60*1000)
#define STATUS_TRANSMIT_MS              (12000)         // Status advertising
#define TAZ_RELAY_CLEAR_DELAY_MS        25000
#define POWERON_STATUS_TRANSMIT_MS      (25000)         // PowerOn advertising
#define WATCHDOG_PET_MS                 2000            // Pet the watchdog every 2 seconds

#define SYS_SLEEP_MS                    ((SYS_TICK_MS*3)/5)
#define TICKS_IN_POWER_BLINK_INTERVAL   ((SECONDS_IN_POWER_BLINK_INTERVAL * 1000) / SYS_TICK_MS)
#define TICKS_IN_TAZ_BLINK_INTERVAL     ((SECONDS_IN_TAZ_BLINK_INTERVAL * 1000) / SYS_TICK_MS)


// IDLE SCAN TIME INPUTS
//
//       +---------+              +---------------------------------------+
//       |  SCAN   |    SCAN      |:::::::::::::::::::::::                |
//       |   ON    |     OFF      |::::: SCAN Window :::::                |
//       |         |              |:::::::::::::::::::::::                |
//   ----+         +--------------+:::::::::::::::::::::::                +----
//
//       |<-- LEGACY ------------>|<-- LEGACY + CODED PHY --------------->|
//       |<-- (SCAN ON/OFF) ----->|<-- (SCAN INTERVAL)------------------->|
//       |<-- One Scan Cycle -------------------------------------------->|
//

// Inputs ======================================================================
#define SCAN_CYCLE_MSEC                 200

#define SCAN_ON_PCT                     25
#define SCAN_OFF_PCT                    50
#define SCAN_WINDOW_PCT                 25

// CALCULATIONS ================================================================
#define SCAN_LEGACY_PCT                 (SCAN_ON_PCT + SCAN_OFF_PCT)
#define SCAN_INTERVAL_PCT               (100 - SCAN_LEGACY_PCT)

#define SCAN_ON_MSEC                    (SCAN_CYCLE_MSEC * SCAN_ON_PCT       / 100)
#define SCAN_OFF_MSEC                   (SCAN_CYCLE_MSEC * SCAN_OFF_PCT      / 100)
#define LEGACY_MSEC                     (SCAN_CYCLE_MSEC * SCAN_LEGACY_PCT   / 100)

#define SCAN_WINDOW_MSEC                (SCAN_CYCLE_MSEC * SCAN_WINDOW_PCT   / 100)
#define SCAN_INTERVAL_MSEC              (SCAN_CYCLE_MSEC * SCAN_INTERVAL_PCT / 100)

#define SCAN_WINDOW_VALUE               ( 8 * SCAN_WINDOW_MSEC   / 5)
#define SCAN_INTERVAL_VALUE             ( 8 * SCAN_INTERVAL_MSEC / 5)

//
// Multi-line macros
//
#define EM_EVENT_MASK_ALL      (  SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM0 \
                                | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM0  \
                                | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM1 \
                                | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM1  \
                                | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM2 \
                                | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM2)
                                // port note: the EM3 transition events no longer exist

// Code macros



/**************************************************************************//**
 * #typedef enum
 *****************************************************************************/
typedef enum
{
    PS_STORED_ADDR_FIRST    = 0x4000,
    PS_STORED_ADDR_LAST     = (PS_STORED_ADDR_FIRST + MAX_TX_TO_STORE),
    PS_STORED_SUPPRESS,
    PS_STORED_REPEATER,
    PS_STORED_LAST_STATE,
} ps_keys_t;

typedef enum
{
    //TIMER_BOOT,
    TIMER_TICK,
    TIMER_TAZ_CLEAR,
    TIMER_BEACON_CLEAR,
    TIMER_START_POWER_ON,
    //TIMER_WAKEUP,
    TIMER_STATUS_DELAY,
    TIMER_STATUS_INTERVAL,
    TIMER_WATCHDOG,
    TIMER_MAX
} timer_types_t;

typedef enum
{
    SM_NONE = 0,
    SM_IDLE,                    /** Listening for beacons */
    SM_ACTIVE,                  /** Relaying TAZ beacon */
    SM_POWERING_ON,             /** Powering on */
    SM_POWERING_OFF,            /** Powering off */
    SM_BEACON_RELAY,            /** Relaying non-taz beacon */
    SM_STATES_SIZE
} SM_States_t;

typedef enum
{
    EVT_BOOT,
    EVT_TICK,
    EVT_MIDDLE_BTN,
    EVT_TOP_BTN,
    EVT_TAZ_FOUND,
    EVT_TAZ_CLEAR,
    EVT_BEACON_FOUND,
    EVT_BEACON_CLEAR,
    EVT_STATUS,
} SM_Events_t;

typedef enum
{
    PKT_UNKNOWN,
    PKT_PRE_2020,               /** All pre 2020 packets are TAZ */
    PKT_2020
} packet_versions_t;


/**************************************************************************//**
 * #typedef function pointer
 *****************************************************************************/
// port note: timer callbacks use the app_timer_callback_t signature
typedef app_timer_callback_t funcPtr;
typedef void (*runState)(SM_Events_t);


/**************************************************************************//**
 * #typedef struct
 *****************************************************************************/
typedef struct
{
    uint8_t           len_flags;
    uint8_t           type_flags;
    uint8_t           val_flags;

    uint8_t           len_addr;
    uint8_t           type_addr;

    uint8_t           addr[6];

    // high bit set, heartbeats are enabled and sent out.
    uint8_t           version;
    uint8_t           packetType;

    // length of the name AD element is variable,
    // adding it last to keep things simple
    uint8_t len_id;
    uint8_t type_id;
    uint8_t id[16];
} non_taz_pkt_t;

typedef struct {
    uint16_t        mask;
    char            *reset_text;
} reset_cause_t;

typedef struct
{
    // This enum value must match the array index for the simple_timer_def_t table
    timer_types_t     timer_type;

    app_timer_t       timer;            // port note: was sl_simple_timer_t
    funcPtr           pfunc;
    uint32_t          timeout_ms;
    bool              is_periodic;
} simple_timer_def_t;

typedef struct
{
    uint8_t           len_flags;
    uint8_t           type_flags;
    uint8_t           val_flags;

    uint8_t           len_addr;
    uint8_t           type_addr;

    uint8_t           addr[6];

    // length of the name AD element is variable,
    // adding it last to keep things simple
    uint8_t           len_id;
    uint8_t           type_id;
    uint8_t           id[16];

    // actual length of advertising data
    uint8_t data_size;
} taz_pkt_t;


/**************************************************************************//**
 * Local function prototypes
 *****************************************************************************/
void                    adc_init(void);
int16_t                 adc_read(int channel);

void                    buzzer_off();
void                    buzzer_on();

static int16_t          convert_adc_count_to_celsius(int32_t adc_count);
SM_States_t             detect_buttons();
void                    displayResetCause(uint32_t resetCause);

#ifdef OUTPUT_EM_INFO
void                    em_transition_cb (sl_power_manager_em_t from, sl_power_manager_em_t to);
#endif

uint16_t                fill_addrs();
void                    fill_non_taz_beacon(bd_addr addr, uint8_t type, uint8_t version);
void                    fill_taz_beacon(bd_addr addr);
packet_versions_t       get_packet_version(uint8_t *adv_packet);

#ifndef USE_EM4_ON_POWER_OFF
static SM_States_t      get_latest_state( void );
#endif

void                    int_handler(uint8_t pin);
void                    init_ps();
void                    init_states();
void                    init_watchdog();
static void             kill_all_timers();

void                    leds_off();
void                    leds_on(SM_States_t state);

sl_status_t             ps_load_object(uint16_t key, void *pValue, uint8_t size);
sl_status_t             ps_save_object(uint16_t key, void *pValue, uint8_t size);

void                    run_active(SM_Events_t evt);
void                    run_beacon_relay(SM_Events_t evt);
void                    run_idle(SM_Events_t evt);
void                    run_machine(SM_Events_t evt);
void                    run_powering_off(SM_Events_t evt);
void                    run_powering_on(SM_Events_t evt);

void                    setup_ext_interrupts(void);
static int              set_latest_state( SM_States_t newState );
bool                    should_trigger_off_addr(bd_addr *addr );
void                    start_advertising_beacon();
void                    start_advertising_taz();
void                    start_scanning();
void                    stop_scanning();

void                    temperature_sensor_goto_low_power();
void                    temperature_sensor_read_disable();
void                    temperature_sensor_read_enable();

void                    timer_cb_beacon_clear(app_timer_t *timer, void *data);
//void                  timer_cb_boot();
void                    timer_cb_start_power_on(app_timer_t *timer, void *data);
void                    timer_cb_status(app_timer_t *timer, void *data);
void                    timer_cb_taz_clear(app_timer_t *timer, void *data);
void                    timer_cb_tick(app_timer_t *timer, void *data);
//void                  timer_cb_wakeup();
void                    timer_cb_watchdog(app_timer_t *timer, void *data);

static int              user_read_request_handler(sl_bt_msg_t *evt );
static int              user_write_request_handler(sl_bt_msg_t *evt );

void                    vibrate_off();
void                    vibrate_on();

/**************************************************************************//**
 * Local globals
 *****************************************************************************/
static volatile uint32_t      adc_sample;
static volatile bool          adc_done;
static volatile IADC_Result_t iadcResult;

static IADC_Init_t            init                      = IADC_INIT_DEFAULT;
static IADC_AllConfigs_t      initAllConfigs            = IADC_ALLCONFIGS_DEFAULT;


static uint32_t               beacon_time;

static uint8_t                adv_handle_1M             = 0xff;

#ifdef CODEDPHY
static uint8_t                adv_handle_codedPHY       = 0xfe;
#endif

static uint8_t                beacon_type;
static uint8_t                param_buffer[149];
static uint8_t                repeater_addr[6];
static uint8_t                suppress_filter = 0;
static uint8_t                toggle_scan_method = 0;

static bool                   allow_to_power_off        = false;
static bool                   buzzer_state              = false;
static bool                   is_powered_off            = false;
static bool                   led_state                 = false;
static bool                   supression_active         = false;
static bool                   vibrator_state            = false;

static bd_addr                addrs[MAX_TX_TO_STORE];
static bd_addr                last_device_seen;
static bd_addr                last_taz_seen;

static non_taz_pkt_t          non_taz_pkt;
static runState               states[SM_STATES_SIZE];
static taz_pkt_t              tazpkt;

static SM_States_t            previousState             = SM_NONE;
static SM_States_t            nextState                 = SM_POWERING_ON;
static SM_States_t            leds_on_state             = SM_NONE;

#ifdef OUTPUT_EM_INFO
static sl_power_manager_em_transition_event_handle_t event_handle;
static sl_power_manager_em_transition_event_info_t   event_info =
          { .event_mask = EM_EVENT_MASK_ALL,
            .on_event = em_transition_cb,
          };
#endif

/**************************************************************************//**
 * Local global arrays
 *****************************************************************************/

// updated (2020) uuid format
static const uint8_t  new_uuid[] =        {0x58, 0xd8, 0x3c, 0x8a, 0x2e, 0xa1, 0x7f, 0xbc,
                                           0xff, 0x4a, 0xb6, 0xb4, 0x5A, 0x48, 0xb5, 0xbe};
// Old uuid value
static const uint8_t  old_uuid[] =        {0x58, 0xd8, 0x3c, 0x8a, 0x2e, 0xa1, 0x7f, 0xbc,
                                           0xff, 0x4a, 0xb6, 0xb4, 0x59, 0x48, 0xb5, 0xbe};

// updated (2020) uuid format  Used for non-taz packets
static const uint8_t  rx_new_uuid[] =     {0x58, 0xd8, 0x3c, 0x8a, 0x2e, 0xa1, 0x7f, 0xbc,
                                           0xff, 0x4a, 0xb6, 0xb4, 0x5A, 0x48, 0xb5, 0xbe};

// used for taz packet to maintain backwards compatibility
static const uint8_t  rx_old_uuid[] =     {0x59, 0xd8, 0x3c, 0x8a, 0x2e, 0xa1, 0x7f, 0xbc,
                                           0xff, 0x4a, 0xb6, 0xb4, 0x59, 0x48, 0xb5, 0xbe};

static const uint8_t  rx_status_uuid[] =  {0x58, 0xd8, 0x3c, 0x8a, 0x2e, 0xa1, 0x7f, 0xbc,
                                           0xff, 0x4a, 0xb6, 0xb4, 0x5B, 0x48, 0xb5, 0xbe};

static simple_timer_def_t simple_timer_def[TIMER_MAX] =
{
//  { .timer_type   = TIMER_BOOT,                 // NOT USED
//    .pfunc        = timer_cb_boot,
//    .timeout_ms   = 0,
//    .is_periodic  = false,
//  },
    { .timer_type   = TIMER_TICK,
      .pfunc        = timer_cb_tick,
      .timeout_ms   = SYS_TICK_MS,
      .is_periodic  = true,
    },
    { .timer_type   = TIMER_TAZ_CLEAR,
      .pfunc        = timer_cb_taz_clear,
      .timeout_ms   = TAZ_RELAY_CLEAR_DELAY_MS,
      .is_periodic  = false,
    },
    { .timer_type   = TIMER_BEACON_CLEAR,
      .pfunc        = timer_cb_beacon_clear,
      .timeout_ms   = BEACON_RELAY_CLEAR_DELAY_MS,
      .is_periodic  = false,
    },
    { .timer_type   = TIMER_START_POWER_ON,
      .pfunc        = timer_cb_start_power_on,
      .timeout_ms   = POWER_BTN_PRESS_DURATION_MS,
      .is_periodic  = false,
    },
//  { .timer_type   = TIMER_WAKEUP,                 // NOT USED
//    .pfunc        = timer_cb_wakeup,
//    .timeout_ms   = 0,
//    .is_periodic  = false,
//  },
    { .timer_type   = TIMER_STATUS_DELAY,
      .pfunc        = timer_cb_status,              // Shared callback
      .timeout_ms   = STATUS_DELAY_MS,
      .is_periodic  = false,
    },
    { .timer_type   = TIMER_STATUS_INTERVAL,
      .pfunc        = timer_cb_status,              // Shared callback
      .timeout_ms   = STATUS_INTERVAL_MS,
      .is_periodic  = false,
    },
    { .timer_type   = TIMER_WATCHDOG,
      .pfunc        = timer_cb_watchdog,
      .timeout_ms   = WATCHDOG_PET_MS,
      .is_periodic  = true,
    },
};


/**************************************************************************//**
 * Single line functions
 *****************************************************************************/

bool are_leds_on()  { return led_state; }
bool is_buzzer_on() { return buzzer_state; }
bool is_vibrator_on() { return vibrator_state;}

#ifdef OUTPUT_EM_INFO
void em_transition_cb (sl_power_manager_em_t from, sl_power_manager_em_t to)
{
    DBPRINT("From EM%d to EM%d\r\n", from, to);
}
#endif

/**************************************************************************//**
 * adc_init -   Initialize common IADC settings shared between reading the
 *              battery voltage (internal) and temperature (external pin)
 *****************************************************************************/
void adc_init (void)
{
    // Enable ADC0 clock
    CMU_ClockEnable (cmuClock_IADC0, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

    // Shutdown between conversions to reduce current
    init.warmup = iadcWarmupNormal;

    initAllConfigs.configs[0].osrHighSpeed   = iadcCfgOsrHighSpeed2x;
    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                       CLK_ADC_FREQ,
                                                                       0,
                                                                       iadcCfgModeNormal,
                                                                       init.srcClkPrescale);

    // Allocate the analog bus for ADC0 inputs:
    //    - Battery (PD01)    -- PD01 not referenced.  Vddx/4 is used
    //    - Temp    (PD03)
    GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDODD0_ADC0;

    IADC_init (IADC0, &init, &initAllConfigs);     // apply global settings to ADC
}


/**************************************************************************//**
 * IADC_IRQHandler
 *****************************************************************************/
void IADC_IRQHandler (void)
{
    iadcResult = IADC_pullSingleFifoResult(IADC0);

    // Read a result from the FIFO
    adc_sample = iadcResult.data;
    IADC_clearInt (IADC0, IADC_IF_SINGLEDONE); // Reading from FIFO doesn't clear automatically

    adc_done = 1;
}

/**************************************************************************//**
 * adc_read -   Read the IADC for the specified channel.
 *****************************************************************************/
int16_t adc_read (int channel)
{
    //DBPRINT("ADC starting read for channel %d\r\n", channel);
    unsigned int raw = 0;

    IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t singleInput = IADC_SINGLEINPUT_DEFAULT;

    // Configure the IADC read based on the channel
    switch (channel)
    {
        /*
         * Resolution is not configurable directly but is based on the
         * selected oversampling ratio (osrHighSpeed), which defaults to
         * 2x and generates 12-bit results.
         */
        case ADC_BATTERY:
        {
            singleInput.posInput                    = iadcPosInputDvdd;
            singleInput.negInput                    = iadcNegInputGnd;
            initAllConfigs.configs[0].analogGain    = iadcCfgAnalogGain1x;
            initAllConfigs.configs[0].reference     = iadcCfgReferenceInt1V2;

            // changing this value does not seem to impact the ADC measure
            // But, it does document the reference voltage used for the IADC
            initAllConfigs.configs[0].vRef          = 1210;

            break;
        }

        case ADC_TEMP:
        {
            singleInput.posInput                    = iadcPosInputPortCPin1;
            singleInput.negInput                    = iadcNegInputGnd;
            initAllConfigs.configs[0].analogGain    = iadcCfgAnalogGain1x;
            initAllConfigs.configs[0].reference     = iadcCfgReferenceVddx;

            // changing this value does not seem to impact the ADC measure
            // But, it does document the reference voltage used for the IADC
            initAllConfigs.configs[0].vRef          = 3000;

            break;
        }

        // No other channels are supported
        default:
        {
            break;
        }
    }

    // Update with new initAllConfigs information
    IADC_init (IADC0, &init, &initAllConfigs);     // apply global settings to ADC

    // Initialize a single-channel conversion
    IADC_initSingle (IADC0, &initSingle, &singleInput);

    // Clear any previous interrupt flags
    IADC_clearInt (IADC0, _IADC_IF_MASK);

    // Enable single-channel done interrupts
    IADC_enableInt (IADC0, IADC_IEN_SINGLEDONE);

    // Enable IADC interrupts
    NVIC_ClearPendingIRQ (IADC_IRQn);
    NVIC_EnableIRQ (IADC_IRQn);

    adc_done = 0;

    // Start ADC conversion
    IADC_command (IADC0, iadcCmdStartSingle);

    // Wait for conversion to complete.  This is set by IADC_IRQHandler(),
    // which is called when the IADC conversion is finished
    while (adc_done)
    {
        ; // Wait
    }

    raw = adc_sample;

    //  IADC_disableInt(IADC0, IADC_IEN_SINGLEDONE);

    sl_sleeptimer_delay_millisecond(10);

    //DBPRINT("ADC finished read count = %ld\r\n", adc_sample);

    return raw;
}


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)       // port note: no longer SL_WEAK
{
    /////////////////////////////////////////////////////////////////////////////
    // Put your additional application init code here!                         //
    // This is called once during start-up.                                    //
    /////////////////////////////////////////////////////////////////////////////

    // Store the cause of the last reset, and clear the reset cause register
    uint32_t resetCause = RMU_ResetCauseGet();
    // Clear Reset causes so we know which reset matters the next time
    RMU_ResetCauseClear();

    // Make sure the timer table indexes match
    APP_ASSERT (/*(simple_timer_def[TIMER_BOOT].timer_type         == TIMER_BOOT) && */
                (simple_timer_def[TIMER_TICK].timer_type            == TIMER_TICK) &&
                (simple_timer_def[TIMER_TAZ_CLEAR].timer_type       == TIMER_TAZ_CLEAR) &&
                (simple_timer_def[TIMER_BEACON_CLEAR].timer_type    == TIMER_BEACON_CLEAR) &&
                (simple_timer_def[TIMER_START_POWER_ON].timer_type  == TIMER_START_POWER_ON) &&
                /*(simple_timer_def[TIMER_WAKEUP].timer_type        == TIMER_WAKEUP) && */
                (simple_timer_def[TIMER_STATUS_DELAY].timer_type    == TIMER_STATUS_DELAY) &&
                (simple_timer_def[TIMER_STATUS_INTERVAL].timer_type == TIMER_STATUS_INTERVAL) &&
                (simple_timer_def[TIMER_WATCHDOG].timer_type        == TIMER_WATCHDOG) );

    APP_ASSERT ( (SCAN_ON_PCT + SCAN_OFF_PCT + SCAN_WINDOW_PCT) <= 100);

    check_version_info();
    print_version_info();
    displayResetCause(resetCause);

#ifdef OUTPUT_EM_INFO
    // Note: Printing EM states causes the device to freeze at EM2
    sl_power_manager_subscribe_em_transition_event(&event_handle, &event_info);
#endif

    init_states();

    // Set up each of the GPIO pins
    GPIO_PinModeSet(LED1_EN_PORT,       LED1_EN_PIN,        gpioModePushPull, 			PULLDOWN);
    GPIO_PinModeSet(LED2_EN_PORT,       LED2_EN_PIN,        gpioModePushPull, 			PULLDOWN);
    GPIO_PinModeSet(BUZZ_EN_PORT,       BUZZ_EN_PIN,        gpioModePushPull, 			PULLDOWN);
    GPIO_PinModeSet(VIB_EN_PORT,        VIB_EN_PIN,         gpioModePushPull, 			PULLDOWN);
    GPIO_PinModeSet(NTC_SENSOR_EN_PORT, NTC_SENSOR_EN_PIN,  gpioModePushPull, 			PULLDOWN);
    GPIO_PinModeSet(PWR_BTN_PORT,       PWR_BTN_PIN,        gpioModeInputPullFilter,	PULLDOWN);
	
	// Set the POWER pin for EM4 wakeup.  See #defines for information
    GPIO_EM4EnablePinWakeup(EM4WUEN_BITMASK, EM4WUEN_BITMASK);

    // Default setup for using EM4
    EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
    em4Init.pinRetentionMode = emuPinRetentionEm4Exit;
    EMU_EM4Init(&em4Init);

    // Used for debugging EM4.  Ensure there is time to reprogram a device
    // during this time, if a device is otherwise in low power EM4 mode, which
    // would block the device from being reprogrammed.
    //GPIO_PinOutSet(LED1_EN_PORT, LED1_EN_PIN);
    //GPIO_PinOutSet(LED2_EN_PORT, LED2_EN_PIN);
    //sl_sleeptimer_delay_millisecond(10000);          // need a sizeable delay to allow programmerto reprogram subsequent times.

#ifndef USE_EM4_ON_POWER_OFF
    if (get_latest_state() == SM_POWERING_OFF )
    {
        leds_off();
        setup_ext_interrupts();
        while( 1 )
        {
            int longDuration = 100;
            sl_power_manager_sleep();
            while( --longDuration )
            {
                if ( GPIO_PinInGet(POWER_PORT, POWER_PIN) == 0 ) break;
                sl_sleeptimer_delay_millisecond(10);
            }

            if ( longDuration == 0 ) break;
            longDuration = 100;
        }
    }
#endif      // ifndef USE_EM4_ON_POWER_OFF

    // Initialize and enable the watchdog, start the timer that feeds the watchdog
    init_watchdog();

    adc_init();
}


/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void)       // port note: no longer SL_WEAK
{
    /////////////////////////////////////////////////////////////////////////////
    // Put your additional application code here!                              //
    // This is called infinitely.                                              //
    // Do not call blocking functions from here!                               //
    /////////////////////////////////////////////////////////////////////////////
}


/**************************************************************************//**
 * Port note (SiSDK 2026.6.1): sl_main hook, called at the very start of
 * sl_main_init(), before clocks, interrupts and the Power Manager are
 * initialized. Replaces the EMU_UnlatchPinRetention() call that was the first
 * line of the 4.3.1 project's main(): it releases the GPIO pins latched in EM4
 * after the power button wakes the Rx.
 * Overrides the SL_WEAK default in sl_main_init.c.
 *****************************************************************************/
void app_init_pre_clock(void)
{
    EMU_UnlatchPinRetention();
}


/**************************************************************************//**
 * Port note (SiSDK 2026.6.1): Power Manager hook. The Studio 6 main() calls
 * sl_power_manager_sleep() in every loop pass; returning false here makes that
 * call return immediately, so the Rx keeps running in EM0 exactly as the 4.3.1
 * build did (its main() had sl_power_manager_sleep() commented out).
 * Return true instead to let the Rx sleep between events (changes timing and
 * current draw - test before enabling).
 * Overrides the SL_WEAK default in the generated sl_power_manager_handler.c.
 *****************************************************************************/
bool app_is_ok_to_sleep(void)
{
    return false;
}


/**************************************************************************//**
 * buzzer_off
 *****************************************************************************/
void buzzer_off()
{
    buzzer_state = false;
    GPIO_PinOutClear(BUZZ_EN_PORT, BUZZ_EN_PIN);
}


/**************************************************************************//**
 * buzzer_on
 *****************************************************************************/
void buzzer_on()
{
    if ( !supression_active )
    {
        buzzer_state = true;
        GPIO_PinOutSet(BUZZ_EN_PORT, BUZZ_EN_PIN);
    }
}


/**************************************************************************//**
 * convert_adc_count_to_celsius
 *
 * Convert the 12 bit (4096) ADC count to a temperature based on the
 * ERT-J0EG103JA thermistor.
 *
 * Reference:  https://industrial.panasonic.com/cdbs/www-data/pdf/AUA0000/AUA0000C8.pdf
 *****************************************************************************/
static int16_t convert_adc_count_to_celsius(int32_t adc_count)
{
    float steinhart, average;

    // Algorithm ported from previous h/w & f/w release.
    average = (4095.0/(float)adc_count) - 1;
    average = 20000.0/average;
    steinhart = average/(10000.0);           // (R/Ro)
    steinhart = log(steinhart);                   // ln(R/Ro)
    steinhart /= 3435.0;                          // 1/B * ln(R/Ro)
    steinhart += 1.0 / (25.0 + 273.15);           // + (1/To)
    steinhart = 1.0 / steinhart;                  // Invert
    steinhart -= 273.15;                          // convert absolute temp to C
    steinhart += ((.071 * steinhart) - 1.9);      // Calibration

    return ( (int16_t)(steinhart * 100) );
}


/**************************************************************************//**
 * detect_buttons - detect whether the power on/off button has been pressed
 *                  for the required number of one millisecond ticks, when
 *                  powering off is allowed.
 *****************************************************************************/
SM_States_t detect_buttons()
{
    SM_States_t state_switch_found = SM_NONE;

    // if powering off is allowed (only time it is not is when it is first
    // powering on.
    if (allow_to_power_off)
    {
        // set values that are retained for subsequent calls to this function
        static uint32_t consecutive_power_btn_presses = 0;
        static bool is_btn_pressed_previous_tick = false;

        // if the button is pressed and was also pressed the previous time
        // this function was called.
        bool is_btn_pressed_now = GPIO_PinInGet(PWR_BTN_PORT, PWR_BTN_PIN);
        if (is_btn_pressed_now && is_btn_pressed_previous_tick)
        {
            // increment the number presses. Since this is checked every
            // millisecond, then check to see that the button has been pressed
            // for the duration required for powering off
            consecutive_power_btn_presses++;
            if (consecutive_power_btn_presses * SYS_TICK_MS > POWER_BTN_PRESS_DURATION_MS)
            {
                // indicate the unit should be powering off and reset the counter for next time
                state_switch_found = SM_POWERING_OFF;
                consecutive_power_btn_presses = 0;
            }
        }

        // Button is not pressed.  Reset the counter
        else
        {
            consecutive_power_btn_presses = 0;
        }

        // Store the button state for the next call to this function
        is_btn_pressed_previous_tick = is_btn_pressed_now;
    }

    return state_switch_found;
}


/**************************************************************************//**
 * Display reason for reset
 *****************************************************************************/
void displayResetCause(uint32_t resetCause)
{
    // if there is a reset cause
    if (resetCause)
    {
        DBPRINT("RESET CAUSE(S):  (0x%04lx)\n",resetCause);   // Output header

        // Reset causes' mask and corresponding text
        // Ordered by main cause.  Reference AN0018.2, Section 2.2
        reset_cause_t reset_causes[] =
        {
            { EMU_RSTCAUSE_POR,       "Poweron Reset (POR)"},                   // Bit 0
            { EMU_RSTCAUSE_PIN,       "External pin reset"},                    // Bit 1
            { EMU_RSTCAUSE_EM4,       "EM4 Wakeup reset"},                      // Bit 2
            { EMU_RSTCAUSE_WDOG0,     "Watchdog 1 reset"},                      // Bit 3
            { EMU_RSTCAUSE_WDOG1,     "Watchdog 2 reset"},                      // Bit 4
            { EMU_RSTCAUSE_LOCKUP,    "Core lockup reset"},                     // Bit 5
            { EMU_RSTCAUSE_SYSREQ,    "Core system reset"},                     // Bit 6
            { EMU_RSTCAUSE_DVDDBOD,   "DVDD HV Brown-out detection reset"},     // Bit 7
            { EMU_RSTCAUSE_DVDDLEBOD, "DVDD LE Brown-out detection reset"},     // Bit 8
            { EMU_RSTCAUSE_DECBOD,    "Decouple LV Brown-out detection reset"}, // Bit 9
            { EMU_RSTCAUSE_AVDDBOD,   "AVDD LE Brown-out detection reset"},     // Bit 10
            { EMU_RSTCAUSE_IOVDD0BOD, "VDDIO 0 Brown-out detection reset"},     // Bit 11
            { EMU_RSTCAUSE_SETAMPER,  "Secure Element tamper event reset"},     // Bit 13
            { EMU_RSTCAUSE_SESYSREQ,  "Secure Element core system reset"},      // Bit 14
            { EMU_RSTCAUSE_SELOCKUP,  "Secure Element Lockup reset"},           // Bit 15
        };

        uint16_t ndx;
        uint16_t size = sizeof(reset_causes) / sizeof(reset_cause_t);
        for(ndx=0; ndx<size; ndx++)
        {
            if (resetCause & reset_causes[ndx].mask)
            {
                DBPRINT("   0x%04x %s\n", reset_causes[ndx].mask,
                                          reset_causes[ndx].reset_text);
            }
        }

        DBPRINT("\n");
    }
}


/**************************************************************************//**
 * fill_addrs
 *****************************************************************************/
uint16_t fill_addrs()
{
    sl_status_t sc;

    // for each value to store in flash
    for (int i = 0; i < MAX_TX_TO_STORE; i++)
    {
        // Store the information in flash
        sc = ps_load_object(PS_STORED_ADDR_FIRST + i, &addrs[i], sizeof(addrs[i]) );

        // if the value's key does not exist or was unspecified, initialize and break out
        if ( (sc == SL_STATUS_BT_PS_KEY_NOT_FOUND ) || (sc == SL_STATUS_BT_UNSPECIFIED ))
        {
            init_ps();
            i--;
            continue;
        }

        //DBPRINT("idx: %d\r\n", i);
        if (sc != SL_STATUS_OK)
        {
            return sc;
        }
    }

    // Store suppression  flags to flash
    sc = ps_load_object(PS_STORED_SUPPRESS, &suppress_filter, 1);
    if ( (sc == SL_STATUS_BT_PS_KEY_NOT_FOUND ) || (sc == SL_STATUS_BT_UNSPECIFIED ))
    {
        suppress_filter = SUPPRESS_STATUS;
        ps_save_object( PS_STORED_SUPPRESS, &suppress_filter, 1 );
    }

    // Store repeater address to flash
    sc = ps_load_object(PS_STORED_REPEATER, repeater_addr, 6);
    if ( (sc == SL_STATUS_BT_PS_KEY_NOT_FOUND ) || (sc == SL_STATUS_BT_UNSPECIFIED ))
    {
        memset( repeater_addr, 0, 6 );
        ps_save_object( PS_STORED_REPEATER, repeater_addr, 6 );
    }

    return sc;
}


/**************************************************************************//**
 * fill_non_taz_beacon - store non-TAZ BLE address and UUID information
 *****************************************************************************/
void fill_non_taz_beacon(bd_addr addr, uint8_t type, uint8_t version)
{
    non_taz_pkt.len_flags   = 2;
    non_taz_pkt.type_flags  = 1;
    non_taz_pkt.val_flags   = 6;
    non_taz_pkt.len_addr    = 9;
    non_taz_pkt.type_addr   = 0xff;
    memcpy(&non_taz_pkt.addr, &addr, sizeof(non_taz_pkt.addr));

    non_taz_pkt.version     = version;
    non_taz_pkt.packetType  = type;
    non_taz_pkt.type_id     = 0x07;
    non_taz_pkt.len_id      = 17;
    memcpy(&non_taz_pkt.id, rx_new_uuid, sizeof(non_taz_pkt.id));
}


/**************************************************************************//**
 * fill_taz_beacon- store TAZ BLE address and UUID information
 *****************************************************************************/
void fill_taz_beacon(bd_addr addr)
{
    sl_status_t sc;

    tazpkt.len_flags        = 2;
    tazpkt.type_flags       = 1;
    tazpkt.val_flags        = 6;

    if (beacon_type == GAP_TYPE_SERVICE)
    {
        tazpkt.len_addr     = 7;
        tazpkt.type_addr    = GAP_TYPE_CUSTOM;
        memcpy(tazpkt.addr, &addr, sizeof(tazpkt.addr));

        tazpkt.type_id      = beacon_type;
        tazpkt.len_id       = 17;
        memcpy(tazpkt.id, rx_old_uuid, sizeof(tazpkt.id));

    }

    // Status
    else
    {
        int16_t data_16;

        tazpkt.len_addr     = 7;
        tazpkt.type_addr    = beacon_type;
        sc = sl_bt_gatt_server_read_attribute_value( gattdb_temperature,
                                                     0,
                                                     sizeof(data_16),
                                                     0,
                                                     (uint8_t *) &data_16);
        APP_ASSERT_STATUS(sc);
        memset( tazpkt.addr, 0, 6);
        memcpy(&tazpkt.addr[2], &data_16, 2 );

        sc = sl_bt_gatt_server_read_attribute_value( gattdb_battery_voltage,
                                                     0,
                                                     sizeof(data_16),
                                                     0,
                                                     (uint8_t *) &data_16);
        APP_ASSERT_STATUS(sc);
        memcpy(&tazpkt.addr[4], &data_16, 2 );

        tazpkt.type_id      = GAP_TYPE_SERVICE;
        tazpkt.len_id       = 17;
        memcpy(tazpkt.id, rx_status_uuid, sizeof(tazpkt.id));
    }
}


/**************************************************************************//**
 * get_packet_version - get BLE packet information
 *****************************************************************************/
packet_versions_t get_packet_version(uint8_t *adv_packet)
{
    // make sure mfr id is where we expect it to be
    // adv_packet[4] == 0xff;
    packet_versions_t results = PKT_UNKNOWN;

    // if length of our mfr data, which we're assuming is in this spot,
    // is 7,  we'll assume for now that this  an old format packet,
    // else we assume that it is a new one.
    bool is_old_format = (adv_packet[3] == 7);
    int  cmp_results = 1;
    if (is_old_format)
    {
        results = PKT_PRE_2020;
        // check uuid matches what we expect.
        cmp_results = memcmp(&adv_packet[13], old_uuid, sizeof(old_uuid));
        if ( cmp_results != 0 )
        {
            // Check to see if it's coming from an rx
            cmp_results = memcmp(&adv_packet[13], rx_old_uuid, sizeof(old_uuid));
        }
    }
    else
    {
        results = PKT_2020;
        cmp_results = memcmp(&adv_packet[15], new_uuid, sizeof(new_uuid));
    }

    // we can now be pretty sure we got a good packet as both the length of mfr
    // data as well as the expected uuids match up
    if (cmp_results == 0)
    {
        // we matched the expected uuid
        return results;
    }

    return PKT_UNKNOWN;
}


#ifndef USE_EM4_ON_POWER_OFF
/**************************************************************************//**
 * get_latest_state - get most recent state stored to flash
 *****************************************************************************/
static SM_States_t get_latest_state( void )
{
    SM_States_t storedState;

    sl_status_t sc = ps_load_object(PS_STORED_LAST_STATE,
                                    &storedState,
                                    sizeof(SM_States_t ));
    if ( (sc == SL_STATUS_BT_PS_KEY_NOT_FOUND ) || (sc == SL_STATUS_BT_UNSPECIFIED ))
    {
        set_latest_state( SM_POWERING_ON );
        storedState = SM_POWERING_ON;
    }

    return( storedState );
}
#endif      // #ifndef USE_EM4_ON_POWER_OFF

/**************************************************************************//**
 * init_ps - initialize for flash storage
 *****************************************************************************/
void init_ps()
{
    uint8_t buf[MAX_PS_ITEM_SIZE] = {0};
    uint8_t suppressDefault = SUPPRESS_STATUS;

    // set stored addrs to zero
    sl_bt_nvm_erase_all();
    for (int i = 0; i < MAX_TX_TO_STORE; i++)
    {
        ps_save_object( PS_STORED_ADDR_FIRST + i, buf, sizeof(addrs[0]));
        memset(&addrs[i], 0, sizeof(addrs[i]));
    }

    ps_save_object( PS_STORED_SUPPRESS, &suppressDefault, 1 );
    memset( repeater_addr, 0, 6 );
    ps_save_object( PS_STORED_REPEATER, repeater_addr, 6 );

    DBPRINT("Erasing PS\r\n");
}


/**************************************************************************//**
 * init_states - set up state machine array
 *****************************************************************************/
void init_states()
{
    states[SM_IDLE]         = run_idle;
    states[SM_ACTIVE]       = run_active;
    states[SM_POWERING_ON]  = run_powering_on;
    states[SM_POWERING_OFF] = run_powering_off;
    states[SM_BEACON_RELAY] = run_beacon_relay;
}


/**************************************************************************//**
 * init_watchdog - set up the watchdog
 *****************************************************************************/
void init_watchdog(void)
{
    //DBPRINT("Watchdog started\n");

    // Start timer that feeds the watchdog
    app_timer_start(&(simple_timer_def[TIMER_WATCHDOG].timer),
                    simple_timer_def[TIMER_WATCHDOG].timeout_ms,
                    simple_timer_def[TIMER_WATCHDOG].pfunc,
                    NO_CALLBACK_DATA,
                    simple_timer_def[TIMER_WATCHDOG].is_periodic);

    // Enable clock for the WDOG module; has no effect on xG21
    CMU_ClockEnable(cmuClock_WDOG0, true);

    // Watchdog Initialize settings
    WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
    CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_ULFRCO); /* ULFRCO as clock source */
    //wdogInit.clrSrc           = cmuSelect_ULFRCO; // Select 1kHZ WDOG oscillator
    wdogInit.debugRun           = false;// true     // WDOG not counting during debug halt
    wdogInit.em2Run             = true;             // WDOG counting when in EM2
    wdogInit.em3Run             = true;             // WDOG counting when in EM3
    wdogInit.em4Block           = false;            // EM4 can be entered
    wdogInit.enable             = true;             // Start watchdog when init done
    wdogInit.lock               = false;            // Do not lock WDOG configuration (if locked, reset needed to unlock)
    wdogInit.perSel             = wdogPeriod_8k;    // Set the watchdog period to 8193 clock periods (ie ~8 seconds)
    //wdogInit.perSel           = wdogPeriod_2k;    // Set the watchdog period to 2049 clock periods (ie ~2 seconds)
    //wdogInit.perSel           = wdogPeriod_16k;   // Set the watchdog period to 16385 clock periods (ie ~16 seconds)
    //wdogInit.prs0MissRstEn    =
    //wdogInit.prs1MissRstEn    =
    //wdogInit.resetDisable     =
    //wdogInit.warnSel          =
    //wdogInit.winSel           =

    // Initializing watchdog with chosen settings
    WDOGn_Init(DEFAULT_WDOG, &wdogInit);
}


/**************************************************************************//**
 * int_handler - external interrupt callback function
 *****************************************************************************/
void int_handler(uint8_t pin)
{
    // if the pin pressed was the POWER pin
    // theoretically, this would handle an external interrupt from different
    // pins, but for this product, there is only the POWER pin
    if (pin == PWR_BTN_PIN)
    {
        // read the pin again
        if (GPIO_PinInGet(PWR_BTN_PORT, PWR_BTN_PIN))
        {
            // button is down, send event
            sl_bt_external_signal(EVT_MIDDLE_BTN);
            // allow_to_power_off = true;
        }
    }

    // Clear the interrupt
    GPIO_IntClear(0xffff);
}


/**************************************************************************//**
 * kill_all_timers - stop each of the timers
 *****************************************************************************/
static void kill_all_timers()
{
    for( int i = 0; i < TIMER_MAX; i++ )
    {
        app_timer_stop( &(simple_timer_def[i].timer) );
    }
}


/**************************************************************************//**
 * leds_off - turn off all the LEDs
 *****************************************************************************/
void leds_off()
{
    led_state = false;
    GPIO_PinOutClear(LED1_EN_PORT, LED1_EN_PIN);
    GPIO_PinOutClear(LED2_EN_PORT, LED2_EN_PIN);
}


/**************************************************************************//**
 * leds_on - turn on all the LEDs
 *****************************************************************************/
void leds_on(SM_States_t state)
{
    leds_on_state = state;

    if ( !supression_active )
    {
        led_state = true;
        GPIO_PinOutSet(LED1_EN_PORT, LED1_EN_PIN);
        GPIO_PinOutSet(LED2_EN_PORT, LED2_EN_PIN);
    }
}


/**************************************************************************//**
 * ps_load_object - fetches a key's value from flash
 *****************************************************************************/
sl_status_t ps_load_object(uint16_t key, void *pValue, uint8_t size)
{
    sl_status_t sc;
    uint8_t tmpArray[MAX_NVM_SIZE];
    size_t returned_data_len;

    sc = sl_bt_nvm_load(key, MAX_NVM_SIZE, &returned_data_len, tmpArray);
    if (sc == SL_STATUS_OK)
    {
        // if expected and actual size match
        if (returned_data_len == size)
        {
            memcpy(pValue, tmpArray, returned_data_len);
        }
        else
        {
            sc = SL_STATUS_BT_UNSPECIFIED;
        }
    }

    return sc;
}


/**************************************************************************//**
 * ps_save_object
 *
 * Store a value into the specified NVM key. Allowed NVM keys are in range from
 * 0x4000 to 0x407F. At most, 56 bytes user data can be stored in one NVM key.
 * The error code 0x018a (command_too_long) is returned if the value data is
 * more than 56 bytes.
 *
 * A corresponding sl_status_t enum could not be found per
 * https://docs.silabs.com/mcu/latest/efr32bg1/group-sl-status.
 *
 * To eliminate the ambuguity, check the received size against the maximum
 * allowed.  If the length is exceeded, return   SL_STATUS_BT_UNSPECIFIED
 *****************************************************************************/
sl_status_t ps_save_object(uint16_t key, void *pValue, uint8_t size)
{
    sl_status_t sc;

    if (size <= MAX_NVM_VALUE_SIZE)
    {
        sc = sl_bt_nvm_save(key, size, pValue);
    }
    else
    {
        sc = SL_STATUS_BT_UNSPECIFIED;
    }

    return sc;
}


/**************************************************************************//**
 * run_active - logic for SM_ACTIVE run state
 *****************************************************************************/
void run_active(SM_Events_t evt)
{
    DBPRINT("run_active %d\r\n", evt);
	
    // if the states have changed
    if (previousState != nextState)
    {
        DBPRINT("TAZ Found\r\n");
        fill_taz_beacon(last_taz_seen);
        start_advertising_taz();
        // do entry things
        app_timer_stop( &(simple_timer_def[TIMER_TAZ_CLEAR].timer) );

        app_timer_start(&(simple_timer_def[TIMER_TAZ_CLEAR].timer),
                        beacon_time,
                        simple_timer_def[TIMER_TAZ_CLEAR].pfunc,
                        NO_CALLBACK_DATA,
                        simple_timer_def[TIMER_TAZ_CLEAR].is_periodic);
    }

    // Process the event
    switch (evt)
    {
        // button press
        case EVT_MIDDLE_BTN:
        {
            allow_to_power_off = true;

            break;
        }

        // clock tick
        case EVT_TICK:
        {
            static uint32_t blinkTicks = 0;
            SM_States_t     e = detect_buttons();
            if (e != SM_NONE)
            {
                nextState = e;
                sl_bt_advertiser_stop(adv_handle_1M);
#ifdef CODEDPHY
                sl_bt_advertiser_stop(adv_handle_codedPHY);
#endif
            }

            // if we're staying in active, lets blink
            if (nextState == SM_ACTIVE)
            {
                // if enough time has elapsed, toggle the LEDs, audio, and tactile
                if (blinkTicks++ > TICKS_IN_TAZ_BLINK_INTERVAL)
                {
                    // if on
                    if ( are_leds_on() || is_buzzer_on() || (beacon_type != GAP_TYPE_SERVICE) )
                    {
                        // turn leds off and decrement blinkTimes
                        leds_off();
                        buzzer_off();

                        // Only turn on the vibrator when the LEDs were turned on
                        // because of a tip-up
                        if (leds_on_state == SM_ACTIVE)
                        {
                            if ( !(suppress_filter & SUPPRESS_VIBRATE) )
                            {
                                 vibrate_on();
                            }
                        }
                    }

                    // else currently off
                    else
                    {
                        if ( !(suppress_filter & SUPPRESS_LED) )
                        {
                            leds_on(SM_ACTIVE);
                            vibrate_off();
                        }

                        if ( !(suppress_filter & SUPPRESS_BUZZER) )
                        {
                            buzzer_on();
                        }
                    }

                    blinkTicks = 0;
                }
            }

            break;
        }

        // Incoming BLE message
        case EVT_TAZ_FOUND:
        {
            // got an alert
            // rebroadcast the TAZ
            if (should_trigger_off_addr(&last_taz_seen))
            {
                if ( beacon_type == GAP_TYPE_STATUS )
                {
                    beacon_time = TAZ_RELAY_CLEAR_DELAY_MS;
                    beacon_type = GAP_TYPE_SERVICE;
                    // each timer we're in here, let's kick a timer to get out a few
                    // seconds down the road.
                    sl_bt_advertiser_stop(adv_handle_1M);
#ifdef CODEDPHY
                    sl_bt_advertiser_stop(adv_handle_codedPHY);
#endif
                    fill_taz_beacon(last_taz_seen);
                    start_advertising_taz();
                }

                app_timer_stop( &(simple_timer_def[TIMER_TAZ_CLEAR].timer) );

                app_timer_start(&(simple_timer_def[TIMER_TAZ_CLEAR].timer),
                                simple_timer_def[TIMER_TAZ_CLEAR].timeout_ms,
                                simple_timer_def[TIMER_TAZ_CLEAR].pfunc,
                                NO_CALLBACK_DATA,
                                simple_timer_def[TIMER_TAZ_CLEAR].is_periodic);
            }

            break;
        }

        case EVT_STATUS:
        {
            if ( suppress_filter & SUPPRESS_STATUS )
            {
                break;
            }

            app_timer_start(&(simple_timer_def[TIMER_STATUS_DELAY].timer),
                            simple_timer_def[TIMER_STATUS_DELAY].timeout_ms,
                            simple_timer_def[TIMER_STATUS_DELAY].pfunc,
                            NO_CALLBACK_DATA,
                            simple_timer_def[TIMER_STATUS_DELAY].is_periodic);

            break;
        }

        case EVT_TAZ_CLEAR:
        {
            DBPRINT("Clear TAZ\r\n");

            // got an alert
            app_timer_stop( &(simple_timer_def[TIMER_TAZ_CLEAR].timer) );
            nextState = SM_IDLE;
            sl_bt_advertiser_stop(adv_handle_1M);
#ifdef CODEDPHY
            sl_bt_advertiser_stop(adv_handle_codedPHY);
#endif
            break;
        }

        case EVT_BEACON_FOUND:
        {
            DBPRINT("Event Beacon Found\r\n");

            break;
        }

        default:
        {
            DBPRINT("event in ACTIVE not handled: %d\r\n", evt);

            break;
        }
    }
}


/**************************************************************************//**
 * run_beacon_relay - logic for SM_BEACON_RELAY run state
 *****************************************************************************/
void run_beacon_relay(SM_Events_t evt)
{
    DBPRINT("run_beacon_relay %d\r\n", evt);
	
    // if the states have changed
    if (previousState != nextState)
    {
        // do entry things
        DBPRINT("Relaying beacon\r\n");
        start_advertising_beacon();
        // setup data for advertising beacon info

        // each time we're in here, let's kick a timer to get out a few
        // seconds down the road.
        app_timer_stop( &(simple_timer_def[TIMER_BEACON_CLEAR].timer) );

        app_timer_start(&(simple_timer_def[TIMER_BEACON_CLEAR].timer),
                        simple_timer_def[TIMER_BEACON_CLEAR].timeout_ms,
                        simple_timer_def[TIMER_BEACON_CLEAR].pfunc,
                        NO_CALLBACK_DATA,
                        simple_timer_def[TIMER_BEACON_CLEAR].is_periodic);
    }

    // process the event
    switch (evt)
    {
        case EVT_MIDDLE_BTN:
        {
            allow_to_power_off = true;

            break;
        }

        case EVT_TICK:
        {
            SM_States_t e = detect_buttons();
            if (e != SM_NONE)
            {
                nextState = e;
            }

            break;
        }

        case EVT_TAZ_FOUND:
        {
            // Got a TAZ, we want that prioritized
            if (should_trigger_off_addr(&last_taz_seen))
            {
                nextState = SM_ACTIVE;
                beacon_type = GAP_TYPE_SERVICE;
                beacon_time = TAZ_RELAY_CLEAR_DELAY_MS;
                sl_bt_advertiser_stop(adv_handle_1M);
#ifdef CODEDPHY
                sl_bt_advertiser_stop(adv_handle_codedPHY);
#endif
            }

            break;
        }

        case EVT_STATUS:
        {
            if ( suppress_filter & SUPPRESS_STATUS )
                break;

            app_timer_start(&(simple_timer_def[TIMER_STATUS_DELAY].timer),
                            simple_timer_def[TIMER_STATUS_DELAY].timeout_ms,
                            simple_timer_def[TIMER_STATUS_DELAY].pfunc,
                            NO_CALLBACK_DATA,
                            simple_timer_def[TIMER_STATUS_DELAY].is_periodic);

            break;
        }

        case EVT_BEACON_FOUND:
        {
            // got an alert
            // rebroadcast the beacon

            // each timer we're in here, let's kick a timer to get out a few
            // seconds down the road.
            app_timer_stop( &(simple_timer_def[TIMER_BEACON_CLEAR].timer) );

            app_timer_start(&(simple_timer_def[TIMER_BEACON_CLEAR].timer),
                            simple_timer_def[TIMER_BEACON_CLEAR].timeout_ms,
                            simple_timer_def[TIMER_BEACON_CLEAR].pfunc,
                            NO_CALLBACK_DATA,
                            simple_timer_def[TIMER_BEACON_CLEAR].is_periodic);

            break;
        }

        case EVT_BEACON_CLEAR:
        {
            DBPRINT("Clear BEACON\r\n");
            // got an alert
            nextState = SM_IDLE;
            sl_bt_advertiser_stop(adv_handle_1M);
#ifdef CODEDPHY
            sl_bt_advertiser_stop(adv_handle_codedPHY);
#endif
            break;
        }

        default:
        {
            DBPRINT("event in BEACON not handled: %d\r\n", evt);

            break;
        }
    }
}


/**************************************************************************//**
 * run_idle - logic for SM_IDLE run state
 *****************************************************************************/
void run_idle(SM_Events_t evt)
{
    DBPRINT("run_idle %d\r\n", evt);
	
    // if the states have changed
    if (previousState != nextState)
    {
        // do entry things
        leds_off();
        buzzer_off();
        vibrate_off();

        DBPRINT("Entering idle\r\n");
    }

    // process the event
    switch (evt)
    {
        case EVT_MIDDLE_BTN:
        {
            allow_to_power_off = true;

            break;
        }

        case EVT_TICK:
        {
#if 0
            if (toggle_scan_method & SCAN_BY_STOP_AND_DELAY)
            {
                stop_scanning();
                sl_sleeptimer_delay_millisecond(SCAN_OFF_MSEC);
            }
            else    // SCAN_BY_INTERVAL_AND_WINDOW
            {
                ; // Do nothing - handled by sl_bt_scanner_set_timing()
            }

            start_scanning();
#endif
            SM_States_t e = detect_buttons();
            if (e != SM_NONE)
            {
                nextState = e;
            }

            break;
        }

        case EVT_TAZ_FOUND:
        {
            if (should_trigger_off_addr(&last_taz_seen) )
            {
              beacon_time = TAZ_RELAY_CLEAR_DELAY_MS;
              beacon_type = GAP_TYPE_SERVICE;
                nextState = SM_ACTIVE;
            }

            break;
        }

        case EVT_STATUS:
        {
            if ( suppress_filter & SUPPRESS_STATUS )
                break;

            beacon_time = STATUS_TRANSMIT_MS;
            beacon_type = GAP_TYPE_STATUS;
            nextState = SM_ACTIVE;
            app_timer_start(&(simple_timer_def[TIMER_STATUS_INTERVAL].timer),
                            simple_timer_def[TIMER_STATUS_INTERVAL].timeout_ms,
                            simple_timer_def[TIMER_STATUS_INTERVAL].pfunc,
                            NO_CALLBACK_DATA,
                            simple_timer_def[TIMER_STATUS_INTERVAL].is_periodic);

            break;
        }

        case EVT_BEACON_FOUND:
        {
            // got an alert
            if (should_trigger_off_addr(&last_taz_seen) )
            {
                nextState = SM_BEACON_RELAY;
            }

            break;
        }

        default:
        {
            DBPRINT("event in IDLE  not handled: %d\r\n", evt);

            break;
        }
    }
}


/**************************************************************************//**
 * run_machine - logic for calling the run state's logic
 *****************************************************************************/
void run_machine(SM_Events_t evt)
{
    DBPRINT("run_machine %d\r\n", evt);
	
    if (states[nextState] != NULL)
    {
        SM_States_t runningState = nextState;
        states[nextState](evt);             // Call the state's handler
        previousState = runningState;
    }
    else
    {
        ; // something happened, start over?
    }
}


/**************************************************************************//**
 * run_powering_off - logic for SM_POWERING_OFF run state
 *****************************************************************************/
void run_powering_off(SM_Events_t evt)
{
    static int blinks = POWER_OFF_BLINKS;
    static int blinkTicks = 0;

    DBPRINT("run_powering_off %d\r\n", evt);
	
    // if the states have changed
    if (previousState != nextState)
    {
        // do entry things
        set_latest_state( SM_POWERING_OFF );
        DBPRINT("Powering off\r\n");
    }

    // process the event
    switch (evt)
    {
        case EVT_TICK:
        {
            // we blink for POWER_OFF_BLINKS ticks and than shut down
            if (blinkTicks++ > TICKS_IN_POWER_BLINK_INTERVAL)
            {
                if (blinks > 0)
                {
                    if (are_leds_on())
                    {
                        leds_off();
                        buzzer_off();

                        blinks--;

                        if (blinks == 0)
                        {
                            blinks = POWER_OFF_BLINKS;
                            nextState = SM_POWERING_ON;

                            WDOGn_Enable (DEFAULT_WDOG, false);
                            kill_all_timers();
                            stop_scanning();
                            sl_bt_advertiser_stop(adv_handle_1M);
#ifdef CODEDPHY
                            sl_bt_advertiser_stop(adv_handle_codedPHY);
#endif
                            is_powered_off = true;

#ifdef USE_EM4_ON_POWER_OFF
                            // Clear the interrupt from the POWER button press,
                            // in order to avoid an immediate wakeup from EM4.
                            //
                            // Reference: EFR32BG21 Reference manual - Section 24.3.8
                            GPIO->IF_CLR = EM4WUEN_BITMASK;

                            EMU_EnterEM4();
#else
                            sl_power_manager_sleep();
#endif
                        }
                    }
                    else
                    {
                        leds_on(SM_POWERING_OFF);
                        buzzer_on();
                    }
                    blinkTicks = 0;
                }
            }

            break;
        }

        default:
        {
            break;
        }
    }
}


/**************************************************************************//**
 * run_powering_on - logic for SM_POWERING_ON run state
 *****************************************************************************/
void run_powering_on(SM_Events_t evt)
{
    static uint32_t blinkTicks = 0;
    static uint8_t  blinkTimes = POWER_OFF_BLINKS;

    DBPRINT("run_powering_on %d\r\n", evt);
	
    // if the states have changed
    if (previousState != nextState)
    {
        // do entry things
        DBPRINT("Powering on\r\n");
        is_powered_off = false;
        start_scanning();
        setup_ext_interrupts();
        leds_on(SM_POWERING_ON);

        app_timer_start(&(simple_timer_def[TIMER_TICK].timer),
                        simple_timer_def[TIMER_TICK].timeout_ms,
                        simple_timer_def[TIMER_TICK].pfunc,
                        NO_CALLBACK_DATA,
                        simple_timer_def[TIMER_TICK].is_periodic);
        blinkTicks = 0;
        allow_to_power_off = false;
        if ( !(suppress_filter & SUPPRESS_STATUS) )
        {
            app_timer_start(&(simple_timer_def[TIMER_STATUS_INTERVAL].timer),
                            simple_timer_def[TIMER_STATUS_INTERVAL].timeout_ms,
                            simple_timer_def[TIMER_STATUS_INTERVAL].pfunc,
                            NO_CALLBACK_DATA,
                            simple_timer_def[TIMER_STATUS_INTERVAL].is_periodic);
        }

        set_latest_state( SM_POWERING_ON );
    }

    // state has not changed
    else
    {
        // blink lights POWER_ON_BLINKS times
        if (blinkTicks++ > TICKS_IN_POWER_BLINK_INTERVAL)
        {
            if (are_leds_on())
            {
                // turn leds off and decrement blinkTimes
                leds_off();
                buzzer_off();
            }
            else
            {
                leds_on(SM_POWERING_ON);
                buzzer_on();
            }
            blinkTimes--;
            blinkTicks = 0;
        }
        if (blinkTimes == 0)
        {
            nextState = SM_ACTIVE;
            beacon_time = POWERON_STATUS_TRANSMIT_MS;
            beacon_type = GAP_TYPE_STATUS;
            blinkTimes = POWER_OFF_BLINKS;
            blinkTicks = 0;
        }
    }

    // Do nothing to avoid compiler warning
    switch(evt)
    {
        default: break;
    }
}



/**************************************************************************//**
 * set_latest_state
 *****************************************************************************/
static int set_latest_state( SM_States_t newState )
{
    int err;
    err = ps_save_object(PS_STORED_LAST_STATE, &newState, sizeof(SM_States_t) );
    return err;
}


/**************************************************************************//**
 * setup_ext_interrupts
 *****************************************************************************/
void setup_ext_interrupts(void)
{
    GPIOINT_Init();

    GPIO_ExtIntConfig(PWR_BTN_PORT, PWR_BTN_PIN, PWR_BTN_PIN,
                      true,         // rising edge trigger
                      true,         // falling edge trigger
                      true);

    GPIOINT_CallbackRegister( PWR_BTN_PIN, int_handler );
    GPIO_IntEnable(1 << PWR_BTN_PIN);
}


/**************************************************************************//**
 * should_trigger_off_addr
 *****************************************************************************/
bool should_trigger_off_addr(bd_addr *addr )
{
    int disabled_slots = 0;

    // we have some stored addresses, we'll only trigger off of them
    for (int i = 0; i < MAX_TX_TO_STORE; i++)
    {
        if ( (addrs[i].addr[0] + addrs[i].addr[1] + addrs[i].addr[2] +
                addrs[i].addr[3] + addrs[i].addr[4] + addrs[i].addr[5] ) == 0 )
        {
            disabled_slots++;
        }
        else if (memcmp(&addrs[i], addr, sizeof(*addr)) == 0)
        {
            // found a match
            return true;
        }
    }

    if ( disabled_slots == MAX_TX_TO_STORE )
        return(true);

    return false;
}


/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
    sl_status_t sc;
    bd_addr address;
    uint8_t address_type;
    uint8_t system_id[8];
    uint8_t *data;


    //DBPRINT("sl_bt_on_event(%ld) \r\n", SL_BT_MSG_ID(evt->header) );

    switch (SL_BT_MSG_ID(evt->header))
    {
        // -------------------------------
        // This event indicates the device has started and the radio is ready.
        // Do not call any stack command before receiving this boot event!
        case sl_bt_evt_system_boot_id:
        {
            //DBPRINT("sl_bt_evt_system_boot_id...START\r\n");

            sc = sl_bt_gatt_server_write_attribute_value (gattdb_sw_rev, 0,
                                                          strlen(get_app_version() ),
                                                          (uint8_t*) get_app_version() );
            APP_ASSERT_STATUS(sc);
            sc = sl_bt_gatt_server_write_attribute_value (gattdb_firmware_revision_string, 0,
                                                          strlen(get_fw_version() ),
                                                          (uint8_t*) get_fw_version() );
            APP_ASSERT_STATUS(sc);

            fill_addrs();

            // we're booted up, start the timers
            sc = app_timer_start(&(simple_timer_def[TIMER_TICK].timer),
                                 simple_timer_def[TIMER_TICK].timeout_ms,
                                 simple_timer_def[TIMER_TICK].pfunc,
                                 NO_CALLBACK_DATA,
                                 simple_timer_def[TIMER_TICK].is_periodic);

            // Extract unique ID from BT Address.
            sc = sl_bt_gap_get_identity_address(&address, &address_type);  // port note: was sl_bt_system_get_identity_address()
            APP_ASSERT_STATUS(sc);

            // Pad and reverse unique ID to get System ID.
            system_id[0] = address.addr[5];
            system_id[1] = address.addr[4];
            system_id[2] = address.addr[3];
            system_id[3] = 0xFF;                  // magic numbers from SSv4
            system_id[4] = 0xFE;                  // magic numbers from SSv4
            system_id[5] = address.addr[2];
            system_id[6] = address.addr[1];
            system_id[7] = address.addr[0];

            sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                         0,
                                                         sizeof(system_id),
                                                         system_id);
            APP_ASSERT_STATUS(sc);

            supression_active = false;

            // Create an advertising set.
            sc = sl_bt_advertiser_create_set(&adv_handle_1M);
            APP_ASSERT_STATUS(sc);

            // Set advertising interval to 100ms.
            sc = sl_bt_advertiser_set_timing(adv_handle_1M,
                                             32, // min. adv. interval (milliseconds * 1.6)
                                             40, // max. adv. interval (milliseconds * 1.6)
                                             0,   // adv. duration
                                             0);  // max. num. adv. events
            APP_ASSERT_STATUS(sc);

#ifdef CODEDPHY
            sc = sl_bt_advertiser_create_set(&adv_handle_codedPHY);
            APP_ASSERT_STATUS(sc);

            sc = sl_bt_advertiser_set_timing(adv_handle_codedPHY,
                                             32, // min. adv. interval (milliseconds * 1.6)
                                             40, // max. adv. interval (milliseconds * 1.6)
                                             0,   // adv. duration
                                             0);  // max. num. adv. events
            APP_ASSERT_STATUS(sc);

            // Coded PHY
            sl_bt_extended_advertiser_set_phy (adv_handle_codedPHY,
                                               sl_bt_gap_phy_coded,
                                               sl_bt_gap_phy_coded);
#endif

            //
            // Set Tx Power & report settings
            //

            sc = sl_bt_system_halt(BT_SYSTEM_HALT);
            APP_ASSERT_STATUS(sc);

            int16_t set_power     = 0;
            int16_t set_min_power = 0, set_max_power = 0;
            int16_t support_min   = 0, support_max   = 0;
            int16_t rf_path_gain  = 0;

            // System Tx Power
            sc = sl_bt_system_set_tx_power(TX_POWER_MIN, TX_POWER_MAX,
                                           &set_min_power, &set_max_power);
            APP_ASSERT_STATUS(sc);

            // Advertising Tx Power
            sc = sl_bt_advertiser_set_tx_power(adv_handle_1M,
                                               TX_POWER_MAX,
                                               &set_power);
            APP_ASSERT_STATUS(sc);
#ifdef CODEDPHY
            sc = sl_bt_advertiser_set_tx_power(adv_handle_codedPHY,
                                               TX_POWER_MAX,
                                               &set_power);
            APP_ASSERT_STATUS(sc);
#endif
            sc = sl_bt_system_halt(BT_SYSTEM_RESUME);
            APP_ASSERT_STATUS(sc);

            sl_bt_system_get_tx_power_setting (&support_min, &support_max,
                                               &set_min_power, &set_max_power,
                                               &rf_path_gain);
            DBPRINT("BLE Power Settings\n");
            DBPRINT("   Sys Set:   min/max:    %3d/%3d\n",   TX_POWER_MIN,  TX_POWER_MAX);
            DBPRINT("   Adv Max:   Set/Cfg:    %3d/%3d\n",   TX_POWER_MAX,  set_power);
            DBPRINT("   Cfg'ed     min/max:    %3d/%3d\n",   set_min_power, set_max_power);
            DBPRINT("   Supported  min/max:    %3d/%3d\n",   support_min,   support_max);
            DBPRINT("   RF Path Gain:          %3d\n\n",     rf_path_gain);

            //DBPRINT("sl_bt_evt_system_boot_id...END\r\n");
            break;
        }

            // -------------------------------
            // This event indicates that a new connection was opened.
        case sl_bt_evt_connection_opened_id:
        {
            //DBPRINT("sl_bt_evt_connection_opened_id...START\r\n");
			
            leds_off();
            buzzer_off();

            supression_active = true;

            //DBPRINT("sl_bt_evt_connection_opened_id...END\r\n");
            break;
        }

            // -------------------------------
            // This event indicates that a connection was closed.
        case sl_bt_evt_connection_closed_id:
        {
            //DBPRINT("sl_bt_evt_connection_closed_id...START\r\n");

            supression_active = false;

            // port note: the reset into OTA DFU mode after an OTA-control
            // write is done by the in_place_ota_dfu component, which handles
            // this event before sl_bt_on_event(). The 4.3.1 code called
            // sl_bt_system_reset(2) here; that command no longer exists.

            //DBPRINT("sl_bt_evt_connection_closed_id...END\r\n");
            break;
        }

        ///////////////////////////////////////////////////////////////////////////
        // Add additional event handlers here as your application requires!      //
        ///////////////////////////////////////////////////////////////////////////

        case sl_bt_evt_system_soft_timer_id:
        {
            // Functionality using simple timers and corresponding callbacks.
            // Code now implemented in each of the timers' callback functions

            //DBPRINT("sl_bt_evt_system_soft_timer_id...START/END\r\n");
            break;
        }

        case sl_bt_evt_system_awake_id:
        {
            // just woke up, perform wakeup things.  Blink/beep twice

            //DBPRINT("sl_bt_evt_system_awake_id...START/END\r\n");
            break;
        }

        // port note: sl_bt_evt_scanner_scan_report was removed from the API.
        // With the legacy scanner feature the 4.3.1 stack already reported
        // through sl_bt_evt_scanner_legacy_advertisement_report.
        case sl_bt_evt_scanner_legacy_advertisement_report_id:
        {
            //DBPRINT("sl_bt_evt_scanner_legacy_advertisement_report_id...START\r\n");

            bd_addr incoming_address;
            bd_addr device_address;
            uint8_t *paddr;
            packet_versions_t version;

            paddr = evt->data.evt_scanner_legacy_advertisement_report.address.addr;
            data = evt->data.evt_scanner_legacy_advertisement_report.data.data;
            version = get_packet_version(data);

            // if address inside advertisement doesn't match actual device address
            // then this is from a repeater.  Don't repeat again.
            if ( memcmp( &data[5], paddr, 6 ) != 0 )
            {
                // Unless the address equals the repeater address value
                if ( memcmp( paddr, repeater_addr, 6 ) != 0 )
                {
                    break;
                }
            }

            if (version != PKT_UNKNOWN)
            {
                memcpy( device_address.addr, &data[5], 6 );
                memcpy( last_device_seen.addr, paddr, 6 );
                if ( should_trigger_off_addr( &device_address ) )
                {
                    if (version == PKT_PRE_2020)
                    {
                        // copy address into some local storage
                        memcpy( last_taz_seen.addr, device_address.addr, sizeof(last_taz_seen.addr));
                        run_machine(EVT_TAZ_FOUND);
                    }
                    else if (version == PKT_2020)
                    {
                        memcpy( incoming_address.addr, device_address.addr, sizeof(incoming_address.addr) );
                        if ( nextState == SM_ACTIVE )
                        {
                            if ( memcmp( incoming_address.addr,last_taz_seen.addr, 6) == 0 )
                            {
                                run_machine(EVT_TAZ_CLEAR);
                            }
                        }

                        bool packet_changed = non_taz_pkt.packetType != data[12] ||
                                              non_taz_pkt.version != data[11];
                        fill_non_taz_beacon(incoming_address, data[12], data[11]);
                        if (packet_changed)
                        {
                            // packet changed, need to start and stop
                            DBPRINT("Packet changed");
                            start_advertising_beacon();
                        }
                        run_machine(EVT_BEACON_FOUND);
                    }
                }
            }

            //DBPRINT("sl_bt_evt_scanner_legacy_advertisement_report_id...END\r\n");
            break;
        }

        case sl_bt_evt_system_external_signal_id:
        {
            //DBPRINT("sl_bt_evt_system_external_signal_id...START\r\n");
			
            if (evt->data.evt_system_external_signal.extsignals == EVT_MIDDLE_BTN)
            {
                if (is_powered_off)
                {
                    // Start a timer for POWER_BTN_PRESS_DURATION_MS
                    app_timer_start(&(simple_timer_def[TIMER_START_POWER_ON].timer),
                                    simple_timer_def[TIMER_START_POWER_ON].timeout_ms,
                                    simple_timer_def[TIMER_START_POWER_ON].pfunc,
                                    NO_CALLBACK_DATA,
                                    simple_timer_def[TIMER_START_POWER_ON].is_periodic);
                }
                else
                {
                    // middle button hit
                    DBPRINT("middle\r\n");
                    run_machine(EVT_MIDDLE_BTN);
                }
            }

            //DBPRINT("sl_bt_evt_system_external_signal_id...END\r\n");
            break;
        }

        // Events related to OTA upgrading
        //-----------------------------------------------------------------------------

        /* Check if the user-type OTA Control Characteristic was
         * written. If ota_control was written, boot the device into
         * Device Firmware Upgrade (DFU) mode. */
        case sl_bt_evt_gatt_server_user_write_request_id:
        {
            //DBPRINT("sl_bt_evt_gatt_server_user_write_request_id...START\r\n");
			
            //int err = user_write_request_handler(evt);
            user_write_request_handler(evt);
			
            // DBPRINT( "characteristic: %x\n",
            // evt->data.evt_gatt_server_user_write_request.characteristic);
			
            //DBPRINT("sl_bt_evt_gatt_server_user_write_request_id...END\r\n");
            break;
        }

        case sl_bt_evt_gatt_server_attribute_value_id:
        {
            //DBPRINT("sl_bt_evt_gatt_server_attribute_value_id...START/END\r\n");
            break;
        }


        case sl_bt_evt_gatt_server_user_read_request_id:
        {
            //DBPRINT("sl_bt_evt_gatt_server_user_read_request_id...START\r\n");
			
            //int err = user_read_request_handler(evt);
            user_read_request_handler(evt);
			
            // DBPRINT( "read req: %lx\n", BGLIB_MSG_ID(evt->header) );
            //DBPRINT("sl_bt_evt_gatt_server_user_read_request_id...END\r\n");
            break;
        }

        case sl_bt_evt_gatt_server_characteristic_status_id:
        {
            //DBPRINT("sl_bt_evt_gatt_server_characteristic_status_id...START\r\n");
            //DBPRINT( "characteristic: %lx\n", BGLIB_MSG_ID(evt->header) );
            //DBPRINT("sl_bt_evt_gatt_server_characteristic_status_id...END\r\n");
            break;
        }

        // ----------------------------------------------
        // Events received but handlers not implemented
        // ----------------------------------------------
        case sl_bt_evt_connection_remote_used_features_id:
        case sl_bt_evt_connection_parameters_id:
        case sl_bt_evt_gatt_mtu_exchanged_id:
        case sl_bt_evt_connection_phy_status_id:
        case sl_bt_evt_connection_get_remote_tx_power_completed_id:
        {
            //DBPRINT("sl_bt_evt_gatt_server_XXXXXXXXXXXXX other %ld...START/END\n", SL_BT_MSG_ID(evt->header));
            break;
        }

        // -------------------------------
        // Default event handler.
        // ----------------------------------------------
        default:
        {
            break;
        }
    }
}


/**************************************************************************//**
 * start_advertising_beacon
 *****************************************************************************/
void start_advertising_beacon()
{
    sl_status_t sc;

    //    DBPRINT("startingAdvBeacon\r\n");

    sc = sl_bt_advertiser_stop (adv_handle_1M);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_advertiser_set_timing(adv_handle_1M, 32, 40, 0, 0);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_legacy_advertiser_set_data(adv_handle_1M,
                                          sl_bt_advertiser_advertising_data_packet,
                                          sizeof(non_taz_pkt),
                                          (uint8_t *) &non_taz_pkt);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_legacy_advertiser_start(adv_handle_1M,
                                       sl_bt_legacy_advertiser_connectable);
    APP_ASSERT_STATUS(sc);

#ifdef CODEDPHY
    sc = sl_bt_advertiser_stop (adv_handle_codedPHY);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_advertiser_set_timing(adv_handle_codedPHY, 32, 40, 0, 0);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_extended_advertiser_set_data(adv_handle_codedPHY,
                                            sizeof(non_taz_pkt),
                                            (uint8_t *) &non_taz_pkt);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_extended_advertiser_start (adv_handle_codedPHY,
                                          sl_bt_extended_advertiser_connectable,
                                          SL_BT_EXTENDED_ADVERTISER_INCLUDE_TX_POWER);
    APP_ASSERT_STATUS(sc);
#endif

}


/**************************************************************************//**
 * start_advertising_taz
 *****************************************************************************/
void start_advertising_taz()
{
    sl_status_t sc;

    //  DBPRINT("startingAdv\r\n");

    sc = sl_bt_advertiser_stop (adv_handle_1M);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_advertiser_set_timing(adv_handle_1M, 32, 40, 0, 0);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_legacy_advertiser_set_data(adv_handle_1M,
                                         sl_bt_advertiser_advertising_data_packet,
                                         sizeof(tazpkt),
                                         (uint8_t *) &tazpkt);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_legacy_advertiser_start(adv_handle_1M,
                                       sl_bt_legacy_advertiser_connectable);
    APP_ASSERT_STATUS(sc);

#ifdef CODEDPHY
    sc = sl_bt_advertiser_stop (adv_handle_codedPHY);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_advertiser_set_timing(adv_handle_codedPHY, 32, 40, 0, 0);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_extended_advertiser_set_data(adv_handle_codedPHY,
                                         sizeof(tazpkt),
                                         (uint8_t *) &tazpkt);
    APP_ASSERT_STATUS(sc);

    sc = sl_bt_extended_advertiser_start (adv_handle_codedPHY,
                                          sl_bt_extended_advertiser_connectable,
                                          SL_BT_EXTENDED_ADVERTISER_INCLUDE_TX_POWER);
    APP_ASSERT_STATUS(sc);
#endif

}


/**************************************************************************//**
 * start_scanning
 *****************************************************************************/
void start_scanning()
{
    //DBPRINT("Starting scanning %d      ON/OFF %d/%d      WINDOW On/Off %d/%d\r\n",
    //        toggle_scan_method & SCAN_BY_STOP_AND_DELAY,
    //        SCAN_ON_MSEC, SCAN_OFF_MSEC,
    //        SCAN_WINDOW_MSEC,SCAN_INTERVAL_MSEC-SCAN_WINDOW_MSEC);

    // Detect Legacy.  Detection of Coded PHY using this method does not work.

#if 0
    if (toggle_scan_method & SCAN_BY_STOP_AND_DELAY)
    {
        sl_bt_scanner_set_timing(sl_bt_scanner_scan_phy_1m_and_coded,
                                 SCAN_ON_MSEC,
                                 SCAN_ON_MSEC);
    }
    else    // SCAN_BY_INTERVAL_AND_WINDOW - Detect Legacy or Coded PHY
    {
        sl_bt_scanner_set_timing(sl_bt_scanner_scan_phy_1m_and_coded,
                                 SCAN_INTERVAL_VALUE,
                                 SCAN_WINDOW_VALUE);
    }

    sl_bt_scanner_set_mode(sl_bt_scanner_scan_phy_1m_and_coded, sl_bt_scanner_scan_mode_passive);

#endif
    sl_bt_scanner_set_parameters( sl_bt_scanner_scan_mode_passive,
                                  SCAN_INTERVAL_VALUE, SCAN_WINDOW_VALUE);

    // Do not assign return code and check for valid status.  It is possible
    // this fails when powering off, removing batteries, and the powering on.
    sl_bt_scanner_start( sl_bt_scanner_scan_phy_1m_and_coded,
                         sl_bt_scanner_discover_observation );
    toggle_scan_method++;

}


/**************************************************************************//**
 * temperature_sensor_goto_low_power
 *
 * FIXME Changing the pin to an input via GPIO_PinModeSet(...) will "turn off"
 * the circuit and possibly save power.  Additional testing would be required
 * in order to verify whether power savings are realized.
 *****************************************************************************/
void temperature_sensor_goto_low_power()
{
    // GPIO_PinModeSet(NTC_SENSOR_EN_PORT, NTC_SENSOR_EN_PIN,  gpioModeInputPull, PULLDOWN);

    GPIO_PinOutClear( NTC_SENSOR_EN_PORT, NTC_SENSOR_EN_PIN );
}


/**************************************************************************//**
 * temperature_sensor_read_disable
 *
 * FIXME Changing the pin to an input via GPIO_PinModeSet(...) will "turn off"
 * the circuit and possibly save power.  Additional testing would be required
 * in order to verify whether power savings are realized.
 *****************************************************************************/
void temperature_sensor_read_disable()
{
    // GPIO_PinModeSet(NTC_SENSOR_EN_PORT, NTC_SENSOR_EN_PIN,  gpioModeInputPull, PULLDOWN);

    GPIO_PinOutClear(NTC_SENSOR_EN_PORT, NTC_SENSOR_EN_PIN );
}


/**************************************************************************//**
 * temperature_sensor_read_enable
 *
 * FIXME Changing the pin to an input via GPIO_PinModeSet(...) will "turn off"
 * the circuit and possibly save power.  Additional testing would be required
 * in order to verify whether power savings are realized.
 *****************************************************************************/
void temperature_sensor_read_enable()
{
    // GPIO_PinModeSet(NTC_SENSOR_EN_PORT, NTC_SENSOR_EN_PIN,  gpioModePushPull, PULLDOWN);

    GPIO_PinOutSet( NTC_SENSOR_EN_PORT, NTC_SENSOR_EN_PIN );
}


/**************************************************************************//**
 * stop_scanning
 *****************************************************************************/
void stop_scanning()
{
    // stop scanning
    sl_status_t sc = sl_bt_scanner_stop();
    APP_ASSERT_STATUS(sc);
}


/**************************************************************************//**
 * timer_cb_beacon_clear- BEACON timer's callback
 *****************************************************************************/
void timer_cb_beacon_clear(app_timer_t *timer, void *data)
{
    (void)timer;
    (void)data;
    //  DBPRINT("timer_cb_beacon_clear \r\n");

    run_machine(EVT_BEACON_CLEAR);

}


/**************************************************************************//**
 * timer_cb_boot - NOT USED
 *****************************************************************************/
//void timer_cb_boot()
//{
//  DBPRINT("timer_cb_boot \n");
//}


/**************************************************************************//**
 * timer_cb_start_power_on - POWER_ON timer's callback
 *****************************************************************************/
void timer_cb_start_power_on(app_timer_t *timer, void *data)
{
    (void)timer;
    (void)data;
    //  DBPRINT("timer_cb_start_power_on \r\n");

    // if button is still pressed, go to power on, else shutdown
    if (GPIO_PinInGet( PWR_BTN_PORT, PWR_BTN_PIN))
    {
        run_machine(EVT_TICK);
    }
    else
    {
        DBPRINT("going back to sleep\r\n");
        temperature_sensor_goto_low_power();
        kill_all_timers();
        sl_power_manager_sleep();
    }
}


/**************************************************************************//**
 * timer_cb_status- STATUS timer's callback
 *****************************************************************************/
void timer_cb_status(app_timer_t *timer, void *data)
{
    (void)timer;
    (void)data;
    //  DBPRINT("timer_cb_status \r\n");

    run_machine(EVT_STATUS);
}


/**************************************************************************//**
 * timer_cb_taz_clear - TAZ clear timer's callback
 *****************************************************************************/
void timer_cb_taz_clear(app_timer_t *timer, void *data)
{
    (void)timer;
    (void)data;
    //  DBPRINT("timer_cb_taz_clear \r\n");

    run_machine(EVT_TAZ_CLEAR);
}


/**************************************************************************//**
 * timer_cb_tick - Millisecond timer's callback
 *
 * FIXME - Double adc_read() is done when reading battery voltage and
 * temperature.  When this was being developed, information from the previous
 * adc_read() was "held" for the next adc_read().  As a result, the temperature
 * read returned the value of the battery read.  A FIXME was added.  At the
 * time of this writing (8/29/2022), it cannot sufficiently be fixed because
 * of board changes where the temperature is read from a new pin.
 *****************************************************************************/
void timer_cb_tick(app_timer_t *timer, void *data)
{
    (void)timer;
    (void)data;
    sl_status_t sc;
    static uint32_t tmpCout = 0;
    static uint8_t cycle = 0;
    int16_t adc_count;
    DBPRINT("timer_cb_tick %ld \r\n", tmpCout);

    // every 8th cycle @ 0
    if ( (cycle & 0x7) == 0 )
    {
        // See adc_read():
        // input      Dvdd / 4, where Dvdd = Vbatt
        // reference  1.21V internal

        // adcValue   = adc_read(ADC_BATTERY)         // {0, 1, 2, . . . (2^12)-1}
        // VBatt      = adcValue / 4095  * 1.21  * 4

        // mVBatt     = VBatt * 1000
        // Level      = 100   @ mVBatt = 3000     i.e.  3.0 Volts
        //            =   0   @ mVBatt = 2000           2.0 Volts
        adc_read(ADC_BATTERY);
        adc_count = adc_read(ADC_BATTERY);
        float   batteryVolts = (float)adc_count / 4095 * 1.21 * 4;
        uint32_t batteryMillivolts = (uint32_t)(batteryVolts * 1000);

        sc = sl_bt_gatt_server_write_attribute_value (gattdb_battery_voltage, 0, 2,
                                                      (uint8_t*) &batteryMillivolts);
        APP_ASSERT_STATUS(sc);

        int16_t pctBattVoltage = 0;
        if ( batteryMillivolts >= 3000 )
            pctBattVoltage = 100;
        else if ( batteryMillivolts >= 2000 )
            pctBattVoltage = (int16_t)(batteryMillivolts - 2000)/10;

        sc = sl_bt_gatt_server_write_attribute_value (gattdb_battery_level, 0, 1,
                                                      (uint8_t*) &pctBattVoltage);
        APP_ASSERT_STATUS(sc);
    }

    // every 16th cycle @ 1
    if ( (cycle & 0xF) == 0x1 )
    {
        // Enable reading the sensor 1 cycle before actually reading
        temperature_sensor_read_enable();
    }

    // every 16th cycle @ 2
    else if ( (cycle & 0xF) == 0x2 )
    {
        adc_read(ADC_TEMP);
        adc_count = adc_read(ADC_TEMP);
        uint32_t tempCelsius = convert_adc_count_to_celsius(adc_count);

        sc = sl_bt_gatt_server_write_attribute_value (gattdb_temperature, 0, 2,
                                                      (uint8_t*) &tempCelsius);
        APP_ASSERT_STATUS(sc);

        temperature_sensor_read_disable();
    }

    tmpCout++;
    cycle++;
    run_machine(EVT_TICK);
}


/**************************************************************************//**
 * timer_cb_wakeup - NOT USED
 *****************************************************************************/
//void timer_cb_wakeup()
//{
//  DBPRINT("timer_cb_wakeup \n");
//}


/**************************************************************************//**
 * timer_cb_wakeup - NOT USED
 *****************************************************************************/
void timer_cb_watchdog(app_timer_t *timer, void *data)
{
    (void)timer;
    (void)data;
    //DBPRINT("Watchdog timer FED\n");
    WDOGn_Feed(DEFAULT_WDOG);
}


/**************************************************************************//**
 * user_read_request_handler
 *****************************************************************************/
static int user_read_request_handler(sl_bt_msg_t *evt )
{
    sl_status_t sc;
    int err = 0;

    /* Per the NCP API: At most ATT_MTU - 1 amount of data can be sent once. */
    int16_t ParamLength = 0;
    bool SendResponse = false;

    uint16_t Characteristic = evt->data.evt_gatt_server_user_read_request.characteristic;
    uint8_t Connection      = evt->data.evt_gatt_server_user_read_request.connection;

    switch( Characteristic )
    {
        case gattdb_filter_value:
        {
            uint8_t filter_slot;
            sc = sl_bt_gatt_server_read_attribute_value( gattdb_filter_param,
                                                         0,
                                                         sizeof(filter_slot),
                                                         0,
                                                         (uint8_t *) &filter_slot);
            APP_ASSERT_STATUS(sc);

            if ( filter_slot == 0 )
                {
                    param_buffer[0] = MAX_TX_TO_STORE;
                    ParamLength = 1;
                }
            else if ( filter_slot <= MAX_TX_TO_STORE )
                {
                    memcpy( param_buffer, &addrs[filter_slot-1], sizeof( bd_addr ) );
                    ParamLength = sizeof( bd_addr );
                } else err = -1;

            SendResponse = true;
            break;
        }

        case gattdb_xgatt_suppress:
        {
            param_buffer[0] = suppress_filter;
            ParamLength = 1;

            SendResponse = true;
            break;
        }

        case gattdb_xgatt_repeat:
        {
            memcpy( param_buffer, repeater_addr, 6);
            ParamLength = 6;

            SendResponse = true;
            break;
        }

        default:
            err = -1;

            SendResponse = true;
            break;
    }

    if( SendResponse )
    {
        sl_status_t sc;
        sc = sl_bt_gatt_server_send_user_read_response(Connection,
                                                       Characteristic,
                                                       NO_ERROR,
                                                       ParamLength,
                                                       (uint8_t*) &param_buffer,
                                                       (void*) NULL);

        if( sc != SL_STATUS_OK)
        {
            err = -1;
        }
    }

    return( err );
}


/**************************************************************************//**
 * user_write_request_handler
 *****************************************************************************/
static int user_write_request_handler(sl_bt_msg_t *evt )
{
    sl_status_t sc;
    int err = 0;
    bool SendResponse = false;

    /* Easy access copies of common values */
    uint16_t Characteristic = evt->data.evt_gatt_server_user_write_request.characteristic;
    uint8_t Connection      = evt->data.evt_gatt_server_user_write_request.connection;
    void *DataPtr           = evt->data.evt_gatt_server_user_write_request.value.data;
    uint8_t DataLen         = evt->data.evt_gatt_server_user_write_request.value.len;

    switch( Characteristic )
    {
        case gattdb_filter_value:
        {
            uint8_t filter_slot;
            size_t readback_value_len;
            sc = sl_bt_gatt_server_read_attribute_value( gattdb_filter_param,
                                                         0,                 // offset
                                                         sizeof(filter_slot),
                                                         &readback_value_len,
                                                         (uint8_t *) &filter_slot);
            APP_ASSERT_STATUS(sc);

            if ( (filter_slot == 0) || (filter_slot > MAX_TX_TO_STORE) )
            {
                err = -1;
            }
            // port note: reject values that are not exactly one BLE address.
            // The 4.3.1 code copied DataLen bytes into the 6-byte addrs[] slot
            // (overwriting the next slots/globals), and a stored value of the
            // wrong size made fill_addrs() erase all of NVM at the next boot.
            else if ( DataLen != sizeof(bd_addr) )
            {
                err = -1;
            }
            else
            {
                err = ps_save_object( PS_STORED_ADDR_FIRST + (filter_slot - 1),
                                      DataPtr,
                                      DataLen );
                memcpy( &addrs[filter_slot-1], DataPtr, DataLen );

            }

            SendResponse = true;
            break;
        }

        case gattdb_xgatt_suppress:
        {
            uint8_t suppress_val = *(uint8_t *)DataPtr;
            if ( suppress_val != suppress_filter )
            {
                err = ps_save_object( PS_STORED_SUPPRESS , &suppress_val, 1 );
                suppress_filter = suppress_val;
            }

            SendResponse = true;
            break;
        }

        case gattdb_xgatt_repeat:
        {
            memcpy( repeater_addr, DataPtr, 6 );
            err = ps_save_object( PS_STORED_REPEATER , repeater_addr, 6 );

            SendResponse = true;
            break;
        }

        case gattdb_ota_control:
        {
            // port note: handled entirely by the in_place_ota_dfu component
            // (it runs before sl_bt_on_event): it sends the write response,
            // waits for the link to flush, closes the connection and resets
            // into the AppLoader. The app must not send a second response.
            SendResponse = false;
            break;
        }


        default:
        {
            err = -1;

            SendResponse = true;
            break;
        }
    }

    if( SendResponse )
    {
        sc = sl_bt_gatt_server_send_user_write_response (Connection, Characteristic, err);

        /* Silence the error code - it has already been communicated back to the BLE client */
    }

    return( err );
}


/**************************************************************************//**
 * vibrate_off - Turn the vibrator off
 *****************************************************************************/
void vibrate_off()
{
    vibrator_state = false;
    GPIO_PinOutClear(VIB_EN_PORT, VIB_EN_PIN);
}


/**************************************************************************//**
 * vibrate_on - Turn the vibrator on
 *****************************************************************************/
void vibrate_on()
{
    if ( !supression_active )
    {
        vibrator_state = true;
        GPIO_PinOutSet(VIB_EN_PORT, VIB_EN_PIN);
    }
}



