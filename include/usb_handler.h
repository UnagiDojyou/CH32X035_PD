// ===================================================================================
// USB Handler for CH32X035/X034/X033                                         * v1.0 *
// ===================================================================================
// 2023 by Stefan Wagner:   https://github.com/wagiminator
// 2026 by Unagi Dojyou:    https://unagidojyou.com

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"
#include "usb_descr.h"

// ===================================================================================
// USB Handler Parameters and Checks
// ===================================================================================
#ifndef USB_VDD
  #define USB_VDD                 0             // MCU supply voltage (0: 3.3V, 1: 5V)
#endif

#if SYS_USE_VECTORS == 0
  #error Interrupt vector table must be enabled (SYS_USE_VECTORS in system.h)!
#endif

// ===================================================================================
// Custom External USB Handler Functions
// ===================================================================================
uint8_t CDC_control(void);
void CDC_EP_init(void);
void CDC_EP_resetInterface(uint8_t interface);
void CDC_EP0_OUT(void);
void CDC_EP1_IN(void);
void CDC_EP2_IN(void);
void CDC_EP2_OUT(void);
void CDC_EP3_IN(void);
void CDC_EP3_OUT(void);
void CDC_EP5_IN(void);
void CDC_EP_clearHalt(uint8_t address);

// ===================================================================================
// USB Handler Defines
// ===================================================================================
// Custom USB handler functions
#define USB_INIT_endpoints        CDC_EP_init   // custom USB EP init handler
#define USB_CLASS_SETUP_handler   CDC_control   // handle custom class requests
#define USB_CLASS_OUT_handler     CDC_EP0_OUT   // handle class out

// Endpoint callback functions
#define EP0_SETUP_callback        USB_EP0_SETUP
#define EP0_IN_callback           USB_EP0_IN
#define EP0_OUT_callback          USB_EP0_OUT
#define EP1_IN_callback           CDC_EP1_IN
#define EP3_IN_callback           CDC_EP3_IN
#define EP3_OUT_callback          CDC_EP3_OUT
#define EP5_IN_callback           CDC_EP5_IN
#define EP2_IN_callback           CDC_EP2_IN
#define EP2_OUT_callback          CDC_EP2_OUT

// ===================================================================================
// Variables
// ===================================================================================
#define USB_SetupBuf     ((PUSB_SETUP_REQ)EP0_buffer)
extern volatile uint8_t  USB_SetupReq, USB_SetupTyp, USB_Config, USB_Addr, USB_ENUM_OK;
extern volatile uint16_t USB_SetupLen;
extern const uint8_t*    USB_pDescr;

// ===================================================================================
// Functions
// ===================================================================================
void USB_init(void);
void USB_EP0_copyDescr(uint8_t len);

#ifdef __cplusplus
}
#endif
