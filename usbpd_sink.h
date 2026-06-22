// ===================================================================================
// USB PD SINK Handler for CH32X035
// ===================================================================================
//
// Functions available:
// --------------------
// PD_connect()             Initialize USB-PD and connect, returns 0 if failed
// PD_negotiate()           Negotiate current settings, returns 0 if failed
// PD_setVoltage(mV)        Request specified voltage in millivolts, returns 0 if failed
//
// PD_getPDONum()           Get total number of PDOs
// PD_getFixedNum()         Get number of fixed power PDOs
// PD_getPPSNum()           Get number of programmable power PDOs
//
// PD_getPDOVoltage(p)      Get voltage of specified fixed power PDO (1..PD_getFixedNum())
// PD_getPDOMinVoltage(p)   Get min voltage of specified PDO (p = 1..PD_getPDONum())
// PD_getPDOMaxVoltage(p)   Get max voltage of specified PDO (p = 1..PD_getPDONum())
// PD_getPDOMaxCurrent(p)   Get max current of specified PDO (p = 1..PD_getPDONum())
//
// PD_getPDO()              Get active PDO
// PD_getVoltage()          Get active voltage
// PD_getCurrent()          Get active current
//
// Reference:               https://github.com/openwch/ch32x035
// 2023 by Stefan Wagner:   https://github.com/wagiminator
// 2025 by Unagi Dojyou:    https://unagidojyou.com

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include <system.h>
#include "usbpd.h"

// ===================================================================================
// Parameters and Checks
// ===================================================================================
#if SYS_USE_VECTORS == 0
  #error Interrupt vector table must be enabled (SYS_USE_VECTORS in system.h)!
#endif

#if   F_CPU == 48000000
  #define USBPD_TMR_TX    (80-1)        // timer value for USB PD BMC TX @ F_CPU=48MHz
  #define USBPD_TMR_RX    (120-1)       // timer value for USB PD BMC RX @ F_CPU=48MHz
#elif F_CPU == 24000000
  #define USBPD_TMR_TX    (40-1)        // timer value for USB PD BMC TX @ F_CPU=24MHz
  #define USBPD_TMR_RX    (60-1)        // timer value for USB PD BMC RX @ F_CPU=24MHz
#elif F_CPU == 12000000
  #define USBPD_TMR_TX    (20-1)        // timer value for USB PD BMC TX @ F_CPU=12MHz
  #define USBPD_TMR_RX    (30-1)        // timer value for USB PD BMC RX @ F_CPU=12MHz
#elif F_CPU ==  6000000
  #define USBPD_TMR_TX    (10-1)        // timer value for USB PD BMC TX @ F_CPU=6MHz
  #define USBPD_TMR_RX    (15-1)        // timer value for USB PD BMC RX @ F_CPU=6MHz
#else
  #error Unsupported system frequency for USBPD!
#endif

// ===================================================================================
// Type defines
// ===================================================================================
typedef struct {
  uint16_t Current;
  uint16_t Voltage;
  uint8_t  Index;
} FixedSourceCap_t;

typedef struct {
  uint16_t MinVoltage;
  uint16_t MaxVoltage;
  uint16_t Current;
  uint8_t PPSPowerLimited;
  uint8_t Index;
} PPSSourceCap_t;

typedef struct {
  uint16_t MinVoltage;
  uint16_t MaxVoltage;
  uint16_t Current_9to15V;
  uint16_t Current_15to20V;
  uint8_t  Index;
} SPRAVSSourceCap_t;

typedef struct {
  uint16_t MinVoltage;
  uint16_t MaxVoltage;
  uint8_t  PDP;
  uint8_t  Index;
} EPRAVSSourceCap_t;

typedef enum {
  PDO_TYPE_FIXED = 0u,
  PDO_TYPE_PPS,
  PDO_TYPE_SPR_AVS,
  PDO_TYPE_EPR_AVS,
  PDO_TYPE_UNKNOWN
} PD_pdo_type_t;

typedef enum {
  CC_IDLE = 0u,
  CC_CHECK_CONNECT,
  CC_CONNECT,
  CC_SOURCE_CAP,
  CC_SEND_REQUEST,
  CC_WAIT_ACCEPT,
  CC_ACCEPT,
  CC_WAIT_PS_RDY,
  CC_PS_RDY,
  CC_GET_SOURCE_CAP,
  CC_WAIT_SRC_CAP,
  CC_EPR_MODE_ENTRY,
  CC_SEND_CHUNK_REQUEST,
} cc_state_t;

// Request Types
typedef enum {
  REQ_FIXED = 0,
  REQ_PPS,
  REQ_SPR_AVS,
  REQ_EPR_AVS,
} pd_request_type_t;

typedef enum {
  PD_EPR_MODE_SPR = 0,
  PD_EPR_MODE_ENTERING,
  PD_EPR_MODE_ENTER_ACK,
  PD_EPR_MODE_EPR
} PD_epr_mode_t;

typedef struct {
  volatile cc_state_t CC_State;
  volatile cc_state_t CC_LastState;
  volatile uint8_t    CC_NoneTimes;
  volatile uint8_t    CC1_ConnectTimes;
  volatile uint8_t    CC2_ConnectTimes;
  FixedSourceCap_t*   FixedSourceCap;
  PPSSourceCap_t*     PPSSourceCap;
  SPRAVSSourceCap_t*  SPRAVSSourceCap;
  EPRAVSSourceCap_t*  EPRAVSSourceCap;
  volatile uint8_t    SourcePDONum;
  volatile uint8_t    SourcePPSNum;
  volatile uint8_t    SourceFixedNum;
  volatile uint8_t    SourceSPRAVSNum;
  volatile uint8_t    SourceEPRAVSNum;
  volatile uint8_t    PD_Version;
  volatile uint16_t   WaitTime;
  volatile uint8_t    SetPDONum;
  volatile uint8_t    LastSetPDONum;
  volatile uint16_t   SetVoltage;
  volatile uint16_t   SetCurrent;
  volatile pd_request_type_t SetRequestType; // Saved request type
  volatile uint8_t    PDO_Mismatch;          // Flag if re-matching failed
  volatile uint16_t   LastSetVoltage;
  volatile uint16_t   LastSetCurrent;
  volatile uint8_t    USBPD_READY;
  volatile uint8_t    SourceMessageID;
  volatile uint8_t    SinkMessageID;
  volatile uint8_t    SinkGoodCRCOver;
  volatile uint8_t    SourceGoodCRCOver;
  volatile uint8_t    SourceCapIsEPR;
  volatile uint8_t    EPRModeCapable;
  volatile PD_epr_mode_t EPR_Mode;
  volatile uint8_t    EPR_NextChunk;    // Next Chunk Number to request
  volatile uint8_t    Chunked;
  volatile uint16_t   RequestChunkMessageType;
} pd_control_t;

// ===================================================================================
// Functions
// ===================================================================================
uint8_t  PD_connect(void);                      // Initialize PD and connect
uint8_t  PD_negotiate(void);                    // Negotiate current settings
uint8_t  PD_Loop(void);                         // Main Loop function, handles Update and PPS Keep-Alive
uint8_t  PD_setVoltage(uint16_t voltage);       // Set specified voltage (in millivolts)

uint8_t  PD_setPPS(uint16_t voltage,uint16_t current); // Set specified voltage and current (in millivolts and milliampere)

uint8_t  PD_setEPRMode(uint8_t enable);         // Enter EPR Mode, exit is not supported yet
uint8_t  PD_getEPRCapable(void);                // Check if Source advertises EPR Mode capable
PD_epr_mode_t PD_getEPRMode(void);              // Get current EPR Mode status

uint8_t  PD_getPDONum(void);                    // Get total number of PDOs
uint8_t  PD_getFixedNum(void);                  // Get number of fixed power PDOs
uint8_t  PD_getPPSNum(void);                    // Get number of programmable power PDOs
uint8_t  PD_getSPRAVSNum(void);                 // Get number of SPR AVS PDOs
uint8_t  PD_getEPRAVSNum(void);                 // Get number of EPR AVS PDOs

PD_pdo_type_t PD_getPDOType(uint8_t pdonum);    // Get type of specified PDO

uint16_t PD_getPDOMinVoltage(uint8_t pdonum);   // Get minimum voltage of specified PDO
uint16_t PD_getPDOMaxVoltage(uint8_t pdonum);   // Get maximum voltage of specified PDO
uint16_t PD_getPDOMaxCurrent(uint8_t pdonum);   // Get max current of specified PDO
uint16_t PD_getPDOMaxCurrentWithVoltage(uint8_t pdonum, uint16_t voltage); // Get max current of specified PDO with voltage
uint16_t PD_getPDOPower(uint8_t pdonum);        // Get max power of specified PDO

uint8_t  PD_getPDO(void);                       // Get active PDO
uint16_t PD_getVoltage(void);                   // Get active voltage
uint16_t PD_getCurrent(void);                   // Get active Current
uint8_t  PD_getMismatch(void);                  // Get mismatch flag
void     PD_setMismatch(uint8_t mismatch);      // Set Capability Mismatch flag for next request
uint8_t  PD_getPPSPowerLimited(uint8_t pdonum); // Get PPS Power Limited flag
uint8_t  PD_getRevision(void);                  // Get PD Specification Revision

uint8_t PD_setPDO(uint8_t pdonum, uint16_t voltage);  // Set specified PDO and voltage
uint8_t PD_setPDOwithCurrent(uint8_t pdonum, uint16_t voltage ,uint16_t current); // Set specified PDO, Voltage and Current 


#ifdef __cplusplus
}
#endif
