// ===================================================================================
// USB PD SINK Handler for CH32X035
// ===================================================================================
//
// Reference:               https://github.com/openwch/ch32x035
// 2023 by Stefan Wagner:   https://github.com/wagiminator
// 2025 by Unagi Dojyou:    https://unagidojyou.com

#include "usbpd_sink.h"
#include <system.h>

// Variables
static pd_control_t PD_control = {
  .CC_State = CC_IDLE,
  .CC1_ConnectTimes = 0,
  .CC2_ConnectTimes = 0,
};

static FixedSourceCap_t PD_SC_fixed[15]; // Expanded for SPR + EPR
static PPSSourceCap_t   PD_SC_PPS[7];
static SPRAVSSourceCap_t   PD_SC_SPR_AVS[7];
static EPRAVSSourceCap_t   PD_SC_EPR_AVS[7];

// Buffers
static __attribute__ ((aligned(4))) uint8_t PD_TR_buffer[264];  // PD transmit/receive buffer
static __attribute__ ((aligned(4))) uint8_t PD_CH_buffer[264];  // PD chunk buffer
static __attribute__ ((aligned(4))) uint8_t PD_SC_buffer[64];  // PD Source Cap buffer (Increased to 16 PDOs)

// ===================================================================================
// USB PD SINK Front End Functions
// ===================================================================================

// Internal helpers
static uint8_t PD_checkCC(void);
static uint8_t PD_update(void);
static void PD_PDO_request(void);
static void PD_sendData(uint8_t length, uint16_t sop);

// Negotiate current settings and wait until finished (return 1) or timeout (return 0)
uint8_t PD_negotiate(void) {
  uint8_t counter = 255;
  PD_control.LastSetVoltage = 0;
  PD_control.USBPD_READY = 0;
  while((!PD_control.USBPD_READY) && (--counter)) {
    DLY_ms(5);
    PD_update();
  }
  return(counter > 0);
}

// Get total number of PDOs (fixed and programmable)
uint8_t PD_getPDONum(void) {
  return PD_control.SourcePDONum;
}

// Get number of fixed power PDOs
uint8_t PD_getFixedNum(void) {
  return PD_control.SourceFixedNum;
}

// Get number of programmable power PDOs
uint8_t PD_getPPSNum(void) {
  return PD_control.SourcePPSNum;
}

// Get number of SPR AVS PDOs
uint8_t PD_getSPRAVSNum(void) {
  return PD_control.SourceSPRAVSNum;
}

// Get number of EPR AVS PDOs
uint8_t PD_getEPRAVSNum(void) {
  return PD_control.SourceEPRAVSNum;
}

// Get type of specified PDO
PD_pdo_type_t PD_getPDOType(uint8_t pdonum) {
  uint8_t i;
  for(i=0; i<PD_control.SourceFixedNum; i++) {
    if(PD_control.FixedSourceCap[i].Index == pdonum) return PDO_TYPE_FIXED;
  }
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if(PD_control.PPSSourceCap[i].Index == pdonum) return PDO_TYPE_PPS;
  }
  for(i=0; i<PD_control.SourceSPRAVSNum; i++) {
    if(PD_control.SPRAVSSourceCap[i].Index == pdonum) return PDO_TYPE_SPR_AVS;
  }
  for(i=0; i<PD_control.SourceEPRAVSNum; i++) {
    if(PD_control.EPRAVSSourceCap[i].Index == pdonum) return PDO_TYPE_EPR_AVS;
  }
  return PDO_TYPE_UNKNOWN;
}
  
// Get minimum voltage of specified PDO (fixed and programmable)
uint16_t PD_getPDOMinVoltage(uint8_t pdonum) {
  uint8_t i;
  for(i=0; i<PD_control.SourceFixedNum; i++) {
    if(PD_control.FixedSourceCap[i].Index == pdonum) return PD_control.FixedSourceCap[i].Voltage;
  }
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if(PD_control.PPSSourceCap[i].Index == pdonum) return PD_control.PPSSourceCap[i].MinVoltage;
  }
  for(i=0; i<PD_control.SourceSPRAVSNum; i++) {
    if(PD_control.SPRAVSSourceCap[i].Index == pdonum) return PD_control.SPRAVSSourceCap[i].MinVoltage;
  }
  for(i=0; i<PD_control.SourceEPRAVSNum; i++) {
    if(PD_control.EPRAVSSourceCap[i].Index == pdonum) return PD_control.EPRAVSSourceCap[i].MinVoltage;
  }
  return 0;
}

// Get maximum voltage of specified PDO (fixed and programmable)
uint16_t PD_getPDOMaxVoltage(uint8_t pdonum) {
  uint8_t i;
  for(i=0; i<PD_control.SourceFixedNum; i++) {
    if(PD_control.FixedSourceCap[i].Index == pdonum) return PD_control.FixedSourceCap[i].Voltage;
  }
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if(PD_control.PPSSourceCap[i].Index == pdonum) return PD_control.PPSSourceCap[i].MaxVoltage;
  }
  for(i=0; i<PD_control.SourceSPRAVSNum; i++) {
    if(PD_control.SPRAVSSourceCap[i].Index == pdonum) return PD_control.SPRAVSSourceCap[i].MaxVoltage;
  }
  for(i=0; i<PD_control.SourceEPRAVSNum; i++) {
    if(PD_control.EPRAVSSourceCap[i].Index == pdonum) return PD_control.EPRAVSSourceCap[i].MaxVoltage;
  }
  return 0;
}

// Get max current of specified PDO
uint16_t PD_getPDOMaxCurrent(uint8_t pdonum) {
  uint8_t i;
  for(i=0; i<PD_control.SourceFixedNum; i++) {
    if(PD_control.FixedSourceCap[i].Index == pdonum) return PD_control.FixedSourceCap[i].Current;
  }
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if(PD_control.PPSSourceCap[i].Index == pdonum) return PD_control.PPSSourceCap[i].Current;
  }
  // AVS cannot get current
  return 0; 
}

// Get Port PDP
uint16_t PD_getMaxPDP(void) {
  uint8_t i;
  uint16_t maxPDP = 0;
  for(i=0; i<PD_control.SourceFixedNum; i++) {
    uint16_t pdp = (uint32_t)PD_control.FixedSourceCap[i].Voltage * PD_control.FixedSourceCap[i].Current / 1000000;
    if(pdp > maxPDP) maxPDP = pdp;
  }
  return maxPDP;
}

// Get max current of specified PDO with voltage
uint16_t PD_getPDOMaxCurrentWithVoltage(uint8_t pdonum, uint16_t voltage) {
  uint8_t i;
  if(voltage == 0) return 0;
  for(i=0; i<PD_getEPRAVSNum(); i++) {
    if(PD_control.EPRAVSSourceCap[i].Index == pdonum) {
      uint32_t current = (uint32_t)PD_control.EPRAVSSourceCap[i].PDP * 1000000u / voltage;
      return current > 5000u ? 5000u : (uint16_t)current;
    }
  }
  for(i=0; i<PD_getSPRAVSNum(); i++) {
    if(PD_control.SPRAVSSourceCap[i].Index == pdonum) {
      if(voltage >= 15000) return PD_control.SPRAVSSourceCap[i].Current_15to20V;
      else return PD_control.SPRAVSSourceCap[i].Current_9to15V;
    }
  }
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if(PD_control.PPSSourceCap[i].Index == pdonum) {
      if(PD_control.PPSSourceCap[i].PPSPowerLimited) {
        return (uint32_t)(PD_getMaxPDP() * 1000000) / voltage;
      } else {
        return PD_control.PPSSourceCap[i].Current;
      }
    }
  }
  for(i=0; i<PD_control.SourceFixedNum; i++) {
    if(PD_control.FixedSourceCap[i].Index == pdonum) return PD_control.FixedSourceCap[i].Current;
  }
  return 0; 
}

// Get max power of specified PDO
uint16_t PD_getPDOPower(uint8_t pdonum) {
  uint8_t i;
  for(i=0; i<PD_control.SourceEPRAVSNum; i++) {
    if(PD_control.EPRAVSSourceCap[i].Index == pdonum) return PD_control.EPRAVSSourceCap[i].PDP;
  }
  for(i=0; i<PD_control.SourceSPRAVSNum; i++) {
    if(PD_control.SPRAVSSourceCap[i].Index == pdonum) {
      uint16_t PDP_15to20V = (uint32_t)20000 * PD_control.SPRAVSSourceCap[i].Current_15to20V / 1000000;
      uint16_t PDP_9to15V = (uint32_t)15000 * PD_control.SPRAVSSourceCap[i].Current_9to15V / 1000000;
      if (PDP_15to20V > PDP_9to15V) {
        return PDP_15to20V;
      } else {
        return PDP_9to15V;
      }
    }
  }
  for(i=0; i<PD_control.SourceFixedNum; i++) {
    if(PD_control.FixedSourceCap[i].Index == pdonum) return (uint32_t)PD_control.FixedSourceCap[i].Voltage * PD_control.FixedSourceCap[i].Current / 1000000;
  }
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if(PD_control.PPSSourceCap[i].Index == pdonum) {
      if (PD_control.PPSSourceCap[i].PPSPowerLimited) {
        return PD_getMaxPDP();
      } else {
        return (uint32_t)PD_control.PPSSourceCap[i].MaxVoltage * PD_control.PPSSourceCap[i].Current / 1000000;
      }
    }
  }
  return 0;
}

// Get PPS Power Limited flag (p = 1..PD_getPDONum())
uint8_t PD_getPPSPowerLimited(uint8_t pdonum) {
  uint8_t i;
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if(PD_control.PPSSourceCap[i].Index == pdonum) return PD_control.PPSSourceCap[i].PPSPowerLimited;
  }
  return 0;
}

// Set specified PDO and voltage and current; returns 0:failed, 1:success
uint8_t PD_setPDOwithCurrent(uint8_t pdonum, uint16_t voltage ,uint16_t current) {
  PD_pdo_type_t type = PD_getPDOType(pdonum);
  if((PD_control.EPR_Mode == PD_EPR_MODE_EPR) && (!PD_control.SourceCapIsEPR || type == PDO_TYPE_PPS)) {
    return 0;
  }

  PD_control.SetPDONum  = pdonum;
  PD_control.SetVoltage = voltage;
  PD_control.SetCurrent = current;

  switch(type) {
    case PDO_TYPE_FIXED:
      PD_control.SetRequestType = REQ_FIXED;
      return PD_negotiate();
    case PDO_TYPE_PPS:
      PD_control.SetRequestType = REQ_PPS;
      return PD_negotiate();
    case PDO_TYPE_SPR_AVS:
      PD_control.SetRequestType = REQ_SPR_AVS;
      return PD_negotiate();
    case PDO_TYPE_EPR_AVS:
      PD_control.SetRequestType = REQ_EPR_AVS;
      return PD_negotiate();
    default:
      return 0;
  }
}

// Set specified PDO and voltage; returns 0:failed, 1:success
uint8_t PD_setPDO(uint8_t pdonum, uint16_t voltage) {
  uint16_t current;
  switch(PD_getPDOType(pdonum)) {
    case PDO_TYPE_FIXED:
      current = PD_getPDOMaxCurrent(pdonum);
      return PD_setPDOwithCurrent(pdonum, voltage, current);

    case PDO_TYPE_PPS:
      current = PD_getPDOMaxCurrentWithVoltage(pdonum, voltage);
      return PD_setPDOwithCurrent(pdonum, voltage, current);

    case PDO_TYPE_SPR_AVS:
      current = PD_getPDOMaxCurrentWithVoltage(pdonum, voltage);
      return PD_setPDOwithCurrent(pdonum, voltage, current);

    case PDO_TYPE_EPR_AVS:
      current = PD_getPDOMaxCurrentWithVoltage(pdonum, voltage);
      return PD_setPDOwithCurrent(pdonum, voltage, current);

    default:
      return 0;
  }
}

// Set specified voltage (in millivolts) if available; returns 0:failed, 1:success
uint8_t PD_setVoltage(uint16_t voltage) {
  uint8_t i;

  if(PD_control.EPR_Mode == PD_EPR_MODE_EPR) {
    if(!PD_control.SourceCapIsEPR) return 0;

    for(i=0; i<PD_control.SourceEPRAVSNum; i++) {
      if ((PD_control.EPRAVSSourceCap[i].MinVoltage <= voltage) && (PD_control.EPRAVSSourceCap[i].MaxVoltage >= voltage)) {
        return PD_setPDO(PD_control.EPRAVSSourceCap[i].Index, voltage);
      }
    }

    for(i=0; i<PD_control.SourceFixedNum; i++) {
      if (PD_control.FixedSourceCap[i].Voltage == voltage) {
        return PD_setPDO(PD_control.FixedSourceCap[i].Index, voltage);
      }
    }

    for(i=0; i<PD_control.SourceSPRAVSNum; i++) {
      if ((PD_control.SPRAVSSourceCap[i].MinVoltage <= voltage) && (PD_control.SPRAVSSourceCap[i].MaxVoltage >= voltage)) {
        return PD_setPDO(PD_control.SPRAVSSourceCap[i].Index, voltage);
      }
    }

    return 0;
  }
  
  for(i=0; i<PD_control.SourceFixedNum; i++) {
    if (PD_control.FixedSourceCap[i].Voltage == voltage) {
      return PD_setPDO(PD_control.FixedSourceCap[i].Index, voltage);
    }
  }
  
  for(i=0; i<PD_control.SourceSPRAVSNum; i++) {
    if ((PD_control.SPRAVSSourceCap[i].MinVoltage <= voltage) && (PD_control.SPRAVSSourceCap[i].MaxVoltage >= voltage)) {
      return PD_setPDO(PD_control.SPRAVSSourceCap[i].Index, voltage);
    }
  }
  
  for(i=0; i<PD_control.SourceEPRAVSNum; i++) {
    if ((PD_control.EPRAVSSourceCap[i].MinVoltage <= voltage) && (PD_control.EPRAVSSourceCap[i].MaxVoltage >= voltage)) {
      return PD_setPDO(PD_control.EPRAVSSourceCap[i].Index, voltage);
    }
  }
  
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if ((PD_control.PPSSourceCap[i].MinVoltage <= voltage) && (PD_control.PPSSourceCap[i].MaxVoltage >= voltage)) {
      return PD_setPDO(PD_control.PPSSourceCap[i].Index, voltage);
    }
  }
  
  return 0;
}

// Set specified voltage and current (in millivolts and milliampere) if available;
// returns 0:failed, 1:success
uint8_t PD_setPPS(uint16_t voltage,uint16_t current) {
  uint8_t i;
  if(PD_control.EPR_Mode == PD_EPR_MODE_EPR) return 0;
  if(!PD_control.SourcePPSNum) return 0;
  for(i=0; i<PD_control.SourcePPSNum; i++) {
    if((PD_control.PPSSourceCap[i].MinVoltage <= voltage) &&
        (PD_control.PPSSourceCap[i].MaxVoltage >= voltage) &&
        (PD_control.PPSSourceCap[i].Current >= current) &&
        (50 <= current)) { //check voltage and current
      return PD_setPDOwithCurrent(PD_control.PPSSourceCap[i].Index, voltage, current);
    }
  }
  return 0;
}

// Get current EPR Mode status
PD_epr_mode_t PD_getEPRMode(void) {
  return PD_control.EPR_Mode;
}

// Check if Source advertises EPR Mode capable
uint8_t PD_getEPRCapable(void) {
  return PD_control.EPRModeCapable;
}

// Set EPR Mode
uint8_t PD_setEPRMode(uint8_t enable) {
  if (!PD_control.USBPD_READY || !PD_control.EPRModeCapable) {
    return 0;
  }
  if (enable) {
    if (PD_control.EPR_Mode == PD_EPR_MODE_SPR) {
      PD_control.CC_LastState = PD_control.CC_State;
      PD_control.CC_State = CC_EPR_MODE_ENTRY;
      PD_control.EPR_Mode = PD_EPR_MODE_ENTERING;
      
      return PD_negotiate();
    }
    else if (PD_control.EPR_Mode == PD_EPR_MODE_EPR) return 1;
  }
  return 0;
}

// Get active PDO
uint8_t PD_getPDO(void) {
  return PD_control.SetPDONum;
}

// Get active voltage
uint16_t PD_getVoltage(void) {
  return PD_control.SetVoltage;
}

// Get active Current
uint16_t PD_getCurrent(void) {
  return PD_control.SetCurrent;
}

// Get PDO mismatch flag
uint8_t PD_getMismatch(void) {
  return PD_control.PDO_Mismatch;
}

// Set Capability Mismatch flag
void PD_setMismatch(uint8_t mismatch) {
  PD_control.PDO_Mismatch = mismatch ? 1 : 0;
}

// Initialize PD registers and states, then connect
uint8_t PD_connect(void) {
  RCC->APB2PCENR |= RCC_AFIOEN | RCC_IOPCEN;
  RCC->AHBPCENR  |= RCC_USBPD;
  GPIOB->CFGHR    = (GPIOB->CFGHR & ~( (uint32_t)0b1111<<(((14)&7)<<2) | (uint32_t)0b1111<<(((15)&7)<<2)))
                                  |  ( (uint32_t)0b0100<<(((14)&7)<<2) | (uint32_t)0b0100<<(((15)&7)<<2));
  #ifdef USB_VDD
    #if USB_VDD > 0
      AFIO->CTLR |= USBPD_IN_HVT;
    #else
      AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;
    #endif
  #else
    RCC->APB1PCENR |= RCC_PWREN;
    PWR->CTLR |= PWR_CTLR_PLS;
    if(PWR->CSR & PWR_CSR_PVDO) AFIO->CTLR |= USBPD_IN_HVT | USBPD_PHY_V33;
    else                        AFIO->CTLR |= USBPD_IN_HVT;
  #endif

  USBPD->DMA      = (uint32_t)PD_TR_buffer;
  USBPD->CONFIG   = USBPD_IE_RX_ACT | USBPD_IE_RX_RESET | USBPD_IE_TX_END  | USBPD_PD_DMA_EN;
  USBPD->STATUS   = USBPD_BUF_ERR   | USBPD_IF_RX_BIT   | USBPD_IF_RX_BYTE 
                  | USBPD_IF_RX_ACT | USBPD_IF_RX_RESET | USBPD_IF_TX_END;
  return PD_negotiate();
}

// ===================================================================================
// USB PD SINK Back End Functions
// ===================================================================================

// Enter reception mode
void PD_RX_mode(void) {
  USBPD->BMC_CLK_CNT =  USBPD_TMR_RX;
  USBPD->CONTROL     = (USBPD->CONTROL & ~USBPD_PD_TX_EN) | USBPD_BMC_START;
}

// Reset PD
void PD_reset(void) {
  USBPD->PORT_CC1 = USBPD_CC_CMP_66 | USBPD_CC_PD;
  USBPD->PORT_CC2 = USBPD_CC_CMP_66 | USBPD_CC_PD;
  PD_control.CC1_ConnectTimes  = 0;
  PD_control.CC2_ConnectTimes  = 0;
  PD_control.CC_NoneTimes      = 0;
  PD_control.SourcePDONum      = 0;
  PD_control.SourceFixedNum    = 0;
  PD_control.SourcePPSNum      = 0;
  PD_control.SourceSPRAVSNum   = 0;
  PD_control.SourceEPRAVSNum   = 0;
  PD_control.FixedSourceCap    = PD_SC_fixed;
  PD_control.PPSSourceCap      = PD_SC_PPS;
  PD_control.SPRAVSSourceCap   = PD_SC_SPR_AVS;
  PD_control.EPRAVSSourceCap   = PD_SC_EPR_AVS;
  PD_control.CC_State          = CC_IDLE;
  PD_control.CC_LastState      = CC_IDLE;
  PD_control.SinkMessageID     = 0;
  PD_control.SinkGoodCRCOver   = 0;
  PD_control.SourceGoodCRCOver = 0;
  PD_control.SourceCapIsEPR    = 0;
  PD_control.PD_Version        = USBPD_REVISION_20;
  PD_control.USBPD_READY       = 0;
  PD_control.SetPDONum         = 1;
  PD_control.LastSetPDONum     = 1;
  PD_control.SetVoltage        = 5000;
  PD_control.LastSetVoltage    = 5000;
  PD_control.SetCurrent        = 1000;
  PD_control.LastSetCurrent    = 1000;
  PD_control.SetRequestType    = REQ_FIXED;
  PD_control.PDO_Mismatch      = 0;
  PD_control.EPRModeCapable    = 0;
  PD_control.EPR_Mode          = PD_EPR_MODE_SPR;
  PD_control.EPR_NextChunk     = 0;
  PD_control.Chunked           = 0;
  PD_control.RequestChunkMessageType = 0;
}

// Copy buffers
void PD_memcpy(uint8_t* dest, const uint8_t* src, uint16_t n) {
  while(n--) *dest++ = *src++;
}

// Send PD data
static void PD_sendData(uint8_t length, uint16_t sop) {
  if((USBPD->CONFIG & USBPD_CC_SEL) == USBPD_CC_SEL) USBPD->PORT_CC2 |= USBPD_CC_LVE;
  else                                               USBPD->PORT_CC1 |= USBPD_CC_LVE;

  USBPD->BMC_CLK_CNT = USBPD_TMR_TX;
  USBPD->TX_SEL      = sop;
  USBPD->BMC_TX_SZ   = length;
  USBPD->STATUS      = 0;
  USBPD->CONTROL    |= USBPD_BMC_START | USBPD_PD_TX_EN;
}

// Detect CC connection; returns 0:No connection, 1:CC1 connection, 2:CC2 connection
static uint8_t PD_checkCC(void) {
  uint8_t ccLine = USBPD_CCNONE;

  USBPD->PORT_CC1 &= ~(USBPD_CC_CE | USBPD_PA_CC_AI);
  USBPD->PORT_CC1 |= USBPD_CC_CMP_22;
  if(USBPD->PORT_CC1 & USBPD_PA_CC_AI) ccLine = USBPD_CC1;

  USBPD->PORT_CC2 &= ~(USBPD_CC_CE | USBPD_PA_CC_AI);
  USBPD->PORT_CC2 |= USBPD_CC_CMP_22;
  if(USBPD->PORT_CC2 & USBPD_PA_CC_AI) ccLine = USBPD_CC2;

  return ccLine;
}

// Analyze PDOs
void PD_PDO_analyze(void) {
  USBPD_PDO_t test;
  PD_control.SourcePPSNum = 0;
  PD_control.SourceFixedNum  = 0;
  PD_control.SourceSPRAVSNum = 0;
  PD_control.SourceEPRAVSNum = 0;

  for(uint8_t i=0; i<PD_control.SourcePDONum; i++) { 
    test.d32 = *(uint32_t*)(&PD_SC_buffer[i*4]);
    
    // Check for EPR AVS (Augmented 11b, Subtype 01b)
    if((test.SourceEPRAVSPDO.AugmentedPowerDataObject == 3u) && (test.SourceEPRAVSPDO.EPRprogrammablePowerSupply == 1u)) {
       if (PD_control.SourceEPRAVSNum < 7) {
        PD_control.EPRAVSSourceCap[PD_control.SourceEPRAVSNum].MinVoltage = POWER_DECODE_100MV(test.SourceEPRAVSPDO.MinVoltageIn100mVincrements);
        PD_control.EPRAVSSourceCap[PD_control.SourceEPRAVSNum].MaxVoltage = POWER_DECODE_100MV(test.SourceEPRAVSPDO.MaxVoltageIn100mVincrements);
        PD_control.EPRAVSSourceCap[PD_control.SourceEPRAVSNum].PDP        = test.SourceEPRAVSPDO.MaxPowerIn1Wincrements;
        PD_control.EPRAVSSourceCap[PD_control.SourceEPRAVSNum].Index      = i + 1;
        PD_control.SourceEPRAVSNum++;
       }
    }
    // Check for SPR AVS (Augmented 11b, Subtype 10b)
    else if((test.SourceSPRAVSPDO.AugmentedPowerDataObject == 3u) && (test.SourceSPRAVSPDO.SPRprogrammablePowerSupply == 2u)) {
      if (PD_control.SourceSPRAVSNum < 7) {
        PD_control.SPRAVSSourceCap[PD_control.SourceSPRAVSNum].Current_9to15V  = POWER_DECODE_10MA(test.SourceSPRAVSPDO.MaxCurrentIn10mA_15V);
        PD_control.SPRAVSSourceCap[PD_control.SourceSPRAVSNum].Current_15to20V = POWER_DECODE_10MA(test.SourceSPRAVSPDO.MaxCurrentIn10mA_20V);
        PD_control.SPRAVSSourceCap[PD_control.SourceSPRAVSNum].MinVoltage      = 9000;
        
        if (test.SourceSPRAVSPDO.MaxCurrentIn10mA_20V > 0) {
            PD_control.SPRAVSSourceCap[PD_control.SourceSPRAVSNum].MaxVoltage  = 20000;
        } else {
            PD_control.SPRAVSSourceCap[PD_control.SourceSPRAVSNum].MaxVoltage  = 15000;
        }
        PD_control.SPRAVSSourceCap[PD_control.SourceSPRAVSNum].Index           = i + 1;
        PD_control.SourceSPRAVSNum++;
      }
    }
    // Check for PPS (Augmented 11b, Subtype 00b)
    else if((test.SourcePPSPDO.AugmentedPowerDataObject==3u) && (test.SourcePPSPDO.SPRprogrammablePowerSupply==0u)) {
      if (PD_control.SourcePPSNum < 7) {
        PD_control.PPSSourceCap[PD_control.SourcePPSNum].MaxVoltage      = POWER_DECODE_100MV(test.SourcePPSPDO.MaxVoltageIn100mVincrements);
        PD_control.PPSSourceCap[PD_control.SourcePPSNum].MinVoltage      = POWER_DECODE_100MV(test.SourcePPSPDO.MinVoltageIn100mVincrements);
        PD_control.PPSSourceCap[PD_control.SourcePPSNum].Current         = POWER_DECODE_50MA(test.SourcePPSPDO.MaxCurrentIn50mAincrements);
        PD_control.PPSSourceCap[PD_control.SourcePPSNum].PPSPowerLimited = test.SourcePPSPDO.PPSpowerLimited;
        PD_control.PPSSourceCap[PD_control.SourcePPSNum].Index           = i + 1;
        PD_control.SourcePPSNum++;
      }
    }
    else if ((test.SourceFixedPDO.FixedSupply==0u) && (test.SourceFixedPDO.VoltageIn50mVunits > 0)) {
      // SPR or EPR Fixed
      if (PD_control.SourceFixedNum < 15) {
        if (test.SourceFixedPDO.EPRModeCapable) {
          PD_control.EPRModeCapable = 1;
        }
        PD_control.FixedSourceCap[PD_control.SourceFixedNum].Current = POWER_DECODE_10MA(test.SourceFixedPDO.MaxCurrentIn10mAunits);
        PD_control.FixedSourceCap[PD_control.SourceFixedNum].Voltage = POWER_DECODE_50MV(test.SourceFixedPDO.VoltageIn50mVunits);
        PD_control.FixedSourceCap[PD_control.SourceFixedNum].Index   = i + 1;
        PD_control.SourceFixedNum++;
      }
    }
  }
}

// Send specified PDO
static void PD_PDO_request(void) {
  uint8_t pdoNum = PD_control.SetPDONum;
  USBPD_SINKRDO_t pdo;
  USBPD_MessageHeader_t mh;
  mh.d16  = 0u;
  pdo.d32 = 0u;

  switch(PD_control.SetRequestType) {
    case REQ_FIXED:
      pdo.SinkFixedVariableRDO.ObjectPosition               = pdoNum;
      pdo.SinkFixedVariableRDO.CapabilityMismatch           = PD_control.PDO_Mismatch;
      pdo.SinkFixedVariableRDO.USBCommunicationsCapable     = 0u; // V1V2 doesn't have USB communication
      pdo.SinkFixedVariableRDO.NoUSBSuspend                 = 1u;
      pdo.SinkFixedVariableRDO.UnchunkedExtendedMessage     = 1u;
      pdo.SinkFixedVariableRDO.EPRModeCapable               = PD_control.EPRModeCapable;
      pdo.SinkFixedVariableRDO.MaxOperatingCurrent10mAunits = PD_control.SetCurrent / 10;
      pdo.SinkFixedVariableRDO.OperatingCurrentIn10mAunits  = PD_control.SetCurrent / 10;
      break;
    case REQ_PPS:
      pdo.SinkPPSRDO.ObjectPosition              = pdoNum;
      pdo.SinkPPSRDO.CapabilityMismatch          = PD_control.PDO_Mismatch;
      pdo.SinkPPSRDO.USBCommunicationsCapable    = 0u; // V1V2 doesn't have USB communication
      pdo.SinkPPSRDO.NoUSBSuspend                = 1u;
      pdo.SinkPPSRDO.UnchunkedExtendedMessage    = 1u;
      pdo.SinkPPSRDO.EPRModeCapable              = PD_control.EPRModeCapable;
      pdo.SinkPPSRDO.OutputVoltageIn20mVunits    = PD_control.SetVoltage / 20;
      pdo.SinkPPSRDO.OperatingCurrentIn50mAunits = PD_control.SetCurrent / 50;
      break;
    case REQ_SPR_AVS:
    case REQ_EPR_AVS:
      pdo.SinkAVSRDO.ObjectPosition              = pdoNum;
      pdo.SinkAVSRDO.CapabilityMismatch          = PD_control.PDO_Mismatch;
      pdo.SinkAVSRDO.USBCommunicationsCapable    = 0u; // V1V2 doesn't have USB communication
      pdo.SinkAVSRDO.NoUSBSuspend                = 1u;
      pdo.SinkAVSRDO.UnchunkedExtendedMessage    = 1u;
      pdo.SinkAVSRDO.EPRModeCapable              = PD_control.EPRModeCapable;
      pdo.SinkAVSRDO.OutputVoltageIn25mVunits    = PD_control.SetVoltage / 25;
      pdo.SinkAVSRDO.OperatingCurrentIn50mAunits = PD_control.SetCurrent / 50;
      break;
    default:
      break;
  }

  mh.MessageHeader.MessageID             = PD_control.SinkMessageID ;
  mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
  if(PD_control.EPR_Mode == PD_EPR_MODE_EPR) {
    if(!PD_control.SourceCapIsEPR || PD_control.SetRequestType == REQ_PPS || pdoNum == 0 || pdoNum > PD_control.SourcePDONum) {
      return;
    }
    mh.MessageHeader.MessageType           = USBPD_DATA_MSG_EPR_REQUEST;
    mh.MessageHeader.NumberOfDataObjects   = 2u; // RDO + PDO Copy
    *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
    PD_memcpy(&PD_TR_buffer[2], (uint8_t*)&pdo.d32, 4);
    PD_memcpy(&PD_TR_buffer[6], &PD_SC_buffer[(pdoNum-1)*4], 4); // Copy requested PDO
    PD_sendData(10, USBPD_TX_SOP0); // Header + 2 Objects
  }else{
    mh.MessageHeader.MessageType           = USBPD_DATA_MSG_REQUEST;
    mh.MessageHeader.NumberOfDataObjects   = 1u;
    *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
    PD_memcpy(&PD_TR_buffer[2], (uint8_t*)&pdo.d32, 4);
    PD_sendData(6, USBPD_TX_SOP0);
  }
}

void PD_process(void) {
  cc_state_t temp = PD_control.CC_State;
  USBPD_MessageHeader_t mh;
  USBPD_ExtendedMessageHeader_t emh;
  USBPD_ExtendedControlDataBlock_t ecdb;

  switch (PD_control.CC_State) {

    case CC_IDLE:
      NVIC_DisableIRQ(USBPD_IRQn);
      PD_reset();
      PD_control.CC_State = CC_CHECK_CONNECT;
      break;

    case CC_CHECK_CONNECT:
      break;

    case CC_CONNECT:
      if(PD_control.CC_LastState != PD_control.CC_State) {
        PD_RX_mode();
        NVIC_SetPriority(USBPD_IRQn, 0x00);
        NVIC_EnableIRQ(USBPD_IRQn);
      }
      break;

    case CC_SOURCE_CAP:
      if(PD_control.SinkGoodCRCOver) {
        PD_control.SinkGoodCRCOver = 0;
        NVIC_DisableIRQ(USBPD_IRQn);
        PD_PDO_analyze();
        NVIC_EnableIRQ(USBPD_IRQn);
        PD_control.CC_State = CC_SEND_REQUEST;
      }
      break;

    case CC_SEND_REQUEST:
      if(PD_control.CC_LastState != PD_control.CC_State) {
        PD_PDO_request();
      }
      if(PD_control.SourceGoodCRCOver) {
        PD_control.SourceGoodCRCOver = 0;
        PD_control.CC_State = CC_WAIT_ACCEPT;
      }
      break;

    case CC_WAIT_PS_RDY:
      break;

    case CC_PS_RDY:
      if(PD_control.SinkGoodCRCOver) {
        PD_control.SinkGoodCRCOver = 0;
        PD_control.CC_State = CC_GET_SOURCE_CAP;
        PD_control.WaitTime = 0;
      }
      break;

    case CC_GET_SOURCE_CAP:
      PD_control.USBPD_READY = 1; 
      if((PD_control.SetPDONum   != PD_control.LastSetPDONum) ||
         (PD_control.SetVoltage  != PD_control.LastSetVoltage) ||
         (PD_control.SetCurrent  != PD_control.LastSetCurrent)) {
        PD_control.LastSetPDONum  = PD_control.SetPDONum;
        PD_control.LastSetVoltage = PD_control.SetVoltage;
        PD_control.LastSetCurrent = PD_control.SetCurrent;
        PD_control.USBPD_READY    = 0;

        if (PD_control.EPR_Mode > PD_EPR_MODE_SPR) {
          PD_control.SourceCapIsEPR = 0;
          mh.d16 = 0u;
          emh.d16 = 0u;
          ecdb.d16 = 0u;
          mh.MessageHeader.MessageID             = PD_control.SinkMessageID;
          mh.MessageHeader.MessageType           = USBPD_EXT_MSG_EXT_CTL;
          mh.MessageHeader.Extended              = 1u;
          mh.MessageHeader.NumberOfDataObjects   = 1u;
          mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
          emh.ExtendedMessageHeader.Chunked = PD_control.Chunked;
          emh.ExtendedMessageHeader.RequestChunk          = 0u;
          emh.ExtendedMessageHeader.ChunkNumber           = 0u;
          emh.ExtendedMessageHeader.DataSize              = 2u;
          ecdb.ExtendedControlDataBlock.ExtendedControlMessageType = USBPD_EXT_MSG_EXT_CTL_EPR_GET_SRC_CAP;
          *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
          *(uint16_t*)&PD_TR_buffer[2] = emh.d16;
          *(uint16_t*)&PD_TR_buffer[4] = ecdb.d16;
          PD_sendData(6, USBPD_TX_SOP0);
        } else {
          mh.d16 = 0u;
          mh.MessageHeader.MessageID             = PD_control.SinkMessageID;
          mh.MessageHeader.MessageType           = USBPD_CONTROL_MSG_GET_SRC_CAP;
          mh.MessageHeader.NumberOfDataObjects   = 0u;
          mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
          *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
          PD_sendData(2, USBPD_TX_SOP0);
        }
        PD_control.CC_State = CC_WAIT_SRC_CAP;
      }
      break;

    case CC_EPR_MODE_ENTRY:
      if(PD_control.CC_LastState != PD_control.CC_State) {
        if (PD_control.EPR_Mode == PD_EPR_MODE_ENTERING) {
          USBPD_MessageHeader_t mh;
          USBPD_EPRMode_DO_t edo;
          mh.d16 = 0;
          mh.MessageHeader.MessageID             = PD_control.SinkMessageID;
          mh.MessageHeader.MessageType           = USBPD_DATA_MSG_EPR_MODE;
          mh.MessageHeader.NumberOfDataObjects   = 1u;
          mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
          edo.d32 = 0;
          edo.Struct.Action = 0x01;
          edo.Struct.Data = 140; // EPR Sink Operational PDP (e.g. 140W)
          *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
          PD_memcpy(&PD_TR_buffer[2], (uint8_t*)&edo.d32, 4);
          PD_sendData(6, USBPD_TX_SOP0);
        }
      }
      if (PD_control.EPR_Mode == PD_EPR_MODE_ENTER_ACK) {
        // Received Enter Ack, Waiting for Enter Succeeded from Source
      }
      else if (PD_control.EPR_Mode == PD_EPR_MODE_EPR) {
        PD_control.LastSetVoltage = 0;
        PD_control.CC_State = CC_GET_SOURCE_CAP;
        PD_control.WaitTime = 0;
      }
      else if (PD_control.EPR_Mode == PD_EPR_MODE_SPR) {
        PD_control.CC_State = CC_GET_SOURCE_CAP;
        PD_control.WaitTime = 0;
      }
      break;

    case CC_SEND_CHUNK_REQUEST:
      if (PD_control.EPR_NextChunk > 0) {
        
        mh.d16 = 0;
        mh.MessageHeader.MessageType = PD_control.RequestChunkMessageType;
        mh.MessageHeader.Extended = 1u;
        mh.MessageHeader.NumberOfDataObjects = 1u;
        mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
        mh.MessageHeader.MessageID = PD_control.SinkMessageID;
        
        emh.d16 = 0;
        emh.ExtendedMessageHeader.Chunked = 1u;
        emh.ExtendedMessageHeader.RequestChunk = 1u;
        emh.ExtendedMessageHeader.ChunkNumber = PD_control.EPR_NextChunk;
        emh.ExtendedMessageHeader.DataSize = 0u;
        
        *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
        *(uint16_t*)&PD_TR_buffer[2] = emh.d16;
        PD_TR_buffer[4] = 0;
        PD_TR_buffer[5] = 0;
        PD_sendData(6, USBPD_TX_SOP0);
        PD_control.CC_State = PD_control.CC_LastState;
      }
      break;


    default:
      break;
  }
  PD_control.CC_LastState = temp;
}

// Update PD, return 1 if PDO is changed
static uint8_t PD_update(void) {
  uint8_t status = 0;

  if (!PD_control.USBPD_READY) {
    uint8_t ccLine = PD_checkCC();
    PD_control.WaitTime++;

    if(PD_control.CC_State == CC_CHECK_CONNECT) {
      if(ccLine == USBPD_CC1) {
        PD_control.CC2_ConnectTimes = 0;
        PD_control.CC1_ConnectTimes++;
        if(PD_control.CC1_ConnectTimes > 5) {
          PD_control.CC1_ConnectTimes = 0;
          PD_control.CC_State = CC_CONNECT;
          USBPD->CONFIG &= ~USBPD_CC_SEL;
        }
      }
      else if(ccLine == USBPD_CC2) {
        PD_control.CC1_ConnectTimes = 0;
        PD_control.CC2_ConnectTimes++;
        if(PD_control.CC2_ConnectTimes > 5) {
          PD_control.CC2_ConnectTimes = 0;
          PD_control.CC_State = CC_CONNECT;
          USBPD->CONFIG |= USBPD_CC_SEL;
        }
      }
      else {
        PD_control.CC1_ConnectTimes = 0;
        PD_control.CC2_ConnectTimes = 0;
      }
    }

    if(PD_control.CC_State > CC_CHECK_CONNECT) {
      if(ccLine == USBPD_CCNONE) {
        PD_control.CC_NoneTimes++;
        if(PD_control.CC_NoneTimes > 5) {
          PD_control.CC_NoneTimes = 0;
          PD_control.CC_State = CC_IDLE;
          NVIC_DisableIRQ(USBPD_IRQn);
        }
      } 
      else PD_control.CC_NoneTimes = 0;    
    }
  }

  cc_state_t old_state = PD_control.CC_State;
  PD_process();
  if (old_state == CC_SOURCE_CAP && PD_control.CC_State == CC_SEND_REQUEST) {
    status = 1; // PDO is changed
  }

  return status;
}

// Send EPR KeepAlive Message (Extended Message, Type 0x13, DataSize 4)
void PD_sendEPRKeepAlive(void) {
  USBPD_MessageHeader_t mh;
  USBPD_ExtendedMessageHeader_t emh;
  USBPD_ExtendedControlDataBlock_t ecdb;

  mh.d16 = 0u;
  mh.MessageHeader.MessageID             = PD_control.SinkMessageID;
  mh.MessageHeader.MessageType           = USBPD_EXT_MSG_EXT_CTL;
  mh.MessageHeader.Extended              = 1u;
  mh.MessageHeader.NumberOfDataObjects   = 1u;
  mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
  emh.ExtendedMessageHeader.RequestChunk          = 0u;
  emh.ExtendedMessageHeader.ChunkNumber           = 0u;
  emh.ExtendedMessageHeader.DataSize              = 2u;
  emh.ExtendedMessageHeader.Chunked               = PD_control.Chunked;
  ecdb.ExtendedControlDataBlock.ExtendedControlMessageType = USBPD_EXT_MSG_EXT_CTL_EPR_KEEP_ALIVE;
  *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
  *(uint16_t*)&PD_TR_buffer[2] = emh.d16;
  *(uint16_t*)&PD_TR_buffer[4] = ecdb.d16;
  PD_sendData(6, USBPD_TX_SOP0);
}

// Main Loop function, handles Update and PPS/EPR Keep-Alive
uint8_t PD_Loop(void) {
  static uint32_t last_time = 0;
  uint8_t status = PD_update();

  if (status != 0) {
    return status;
  } else if (PD_control.EPR_Mode == PD_EPR_MODE_EPR) {
    // EPR KeepAlive Logic (Every 375ms)
    if ((STK->CNTL - last_time) > (375 * DLY_MS_TIME)) {
      last_time = STK->CNTL;
      if (PD_control.USBPD_READY) {
        PD_sendEPRKeepAlive();
      }
    }
  } else if (PD_control.USBPD_READY && PD_control.SetRequestType == REQ_PPS && PD_control.PDO_Mismatch == 0) {
    // Handle timer wrap-around (using unsigned subtraction)
    // Check if 5000ms has passed
    if ((STK->CNTL - last_time) > (5000 * DLY_MS_TIME)) {
      last_time = STK->CNTL;
      if (PD_control.USBPD_READY) {
        PD_PDO_request();
      }
    }
  } else {
    // Reset timer when not in active PPS state to restart count upon entry
    last_time = STK->CNTL;
  }

  return status;
}

// Analyze received data
void PD_RX_analyze(void) {
  uint8_t sendGoodCRCFlag = 1;
  USBPD_MessageHeader_t mh;
  USBPD_ExtendedMessageHeader_t emh;
  mh.d16 = *(uint16_t*)PD_TR_buffer;

  if(mh.MessageHeader.Extended == 1u) {
    uint8_t chunkCompleted = 1;
    emh.d16 = *(uint16_t*)&PD_TR_buffer[2];
    if(emh.ExtendedMessageHeader.Chunked == 1u) {
      uint8_t chunkNum = emh.ExtendedMessageHeader.ChunkNumber;
      uint16_t dataSize = emh.ExtendedMessageHeader.DataSize;
      uint16_t chunkOffset = (uint16_t)chunkNum * 26u;
      uint16_t bytesInChunk = 26u;
      PD_control.Chunked = 1u;

      if(dataSize > sizeof(PD_CH_buffer) - 4u) {
        dataSize = sizeof(PD_CH_buffer) - 4u;
      }
      if(chunkOffset >= dataSize) {
        bytesInChunk = 0;
        chunkCompleted = 1;
      } else if(dataSize > chunkOffset + 26u) {
        PD_control.RequestChunkMessageType = mh.MessageHeader.MessageType;
        PD_control.EPR_NextChunk = chunkNum + 1u;
        PD_control.CC_State = CC_SEND_CHUNK_REQUEST;
        chunkCompleted = 0;
      } else {
        bytesInChunk = dataSize - chunkOffset;
        chunkCompleted = 1;
      }

      if(chunkNum == 0u) {
        PD_memcpy(&PD_CH_buffer[0], &PD_TR_buffer[0], 4u);
      }
      if(bytesInChunk > 0u) {
        PD_memcpy(&PD_CH_buffer[4u + chunkOffset], &PD_TR_buffer[4], bytesInChunk);
      }
    } else {
      PD_control.Chunked = 0u;
      chunkCompleted = 1;
    }

    if (chunkCompleted > 0) {
      uint16_t dataSize = emh.ExtendedMessageHeader.DataSize;
      if(dataSize > sizeof(PD_SC_buffer)) dataSize = sizeof(PD_SC_buffer);

      switch(mh.MessageHeader.MessageType) {

        case USBPD_EXT_MSG_EPR_SRC_CAP:
          PD_control.CC_State = CC_SOURCE_CAP;
          PD_control.EPR_Mode = PD_EPR_MODE_EPR;
          PD_control.SourceCapIsEPR = 1;
          PD_control.SourcePDONum = dataSize / 4u;
          if (PD_control.Chunked > 0) {
            PD_memcpy(PD_SC_buffer, &PD_CH_buffer[4], dataSize);
          } else {
            PD_memcpy(PD_SC_buffer, &PD_TR_buffer[4], dataSize);
          }
          PD_control.EPR_NextChunk = 0;
          break;

        case USBPD_EXT_MSG_EXT_CTL:
          USBPD_ExtendedControlDataBlock_t ecdb;
          PD_memcpy((uint8_t*)&ecdb.d16, &PD_TR_buffer[4], 2);
          switch(ecdb.ExtendedControlDataBlock.ExtendedControlMessageType) {
            case USBPD_EXT_MSG_EXT_CTL_EPR_KEEP_ALIVE_ACK:
              break;

            default:
              break;
          }
          break;
            
        default:
          break;
      }
    }
  } else {
    // Control Messages
    if(mh.MessageHeader.NumberOfDataObjects == 0u) {
      switch(mh.MessageHeader.MessageType) {

        case USBPD_CONTROL_MSG_GOODCRC:
          sendGoodCRCFlag = 0;
          PD_control.SourceGoodCRCOver = 1;
          PD_control.SinkMessageID++;
          break;

        case USBPD_CONTROL_MSG_ACCEPT:
          PD_control.CC_State = CC_WAIT_PS_RDY;
          break;

        case USBPD_CONTROL_MSG_PS_RDY:
          PD_control.CC_State = CC_PS_RDY;
          break;

        case USBPD_CONTROL_MSG_NOT_SUPPORTED:
        case USBPD_CONTROL_MSG_REJECT:
          if (PD_control.CC_State == CC_EPR_MODE_ENTRY) {
            PD_control.EPR_Mode = PD_EPR_MODE_SPR;
            PD_control.CC_State = CC_GET_SOURCE_CAP;
          }
          break;

        default:
          break;
      }
    }
    else {
      // Data Messages
      switch(mh.MessageHeader.MessageType) {

        case USBPD_DATA_MSG_SRC_CAP:
          PD_control.CC_State = CC_SOURCE_CAP;
          PD_control.EPR_Mode = PD_EPR_MODE_SPR;
          PD_control.SourceCapIsEPR = 0;
          PD_control.SourcePDONum = mh.MessageHeader.NumberOfDataObjects;
          if(PD_control.SourcePDONum > (sizeof(PD_SC_buffer) / 4u)) {
            PD_control.SourcePDONum = sizeof(PD_SC_buffer) / 4u;
          }
          PD_control.PD_Version = mh.MessageHeader.SpecificationRevision;
          PD_memcpy(PD_SC_buffer, &PD_TR_buffer[2], PD_control.SourcePDONum * 4u);
          break;
          
        case USBPD_DATA_MSG_EPR_MODE:
          USBPD_EPRMode_DO_t eprdo;
          PD_memcpy((uint8_t*)&eprdo.d32, &PD_TR_buffer[2], 4);
          if (eprdo.Struct.Action == 0x02) { // Enter Ack
            PD_control.EPR_Mode = PD_EPR_MODE_ENTER_ACK;
          } else if (eprdo.Struct.Action == 0x03) { // Enter Succeeded
            PD_control.EPR_Mode = PD_EPR_MODE_EPR;
            PD_control.LastSetVoltage = 0;
            PD_control.CC_State = CC_GET_SOURCE_CAP;
          } else if (eprdo.Struct.Action == 0x04) { // Enter Failed
            PD_control.EPR_Mode = PD_EPR_MODE_SPR;
            PD_control.CC_State = CC_GET_SOURCE_CAP;
          } else if (eprdo.Struct.Action == 0x05) { // Exit
            PD_control.EPR_Mode = PD_EPR_MODE_SPR;
            PD_control.CC_State = CC_GET_SOURCE_CAP;
          }
          break;

        default:
          break;
      }
    }
  }

  if(sendGoodCRCFlag) {
    DLY_us(30);
    PD_control.SinkGoodCRCOver = 0;
    USBPD_MessageHeader_t my_mh;
    my_mh.d16 = 0u;
    my_mh.MessageHeader.MessageID = mh.MessageHeader.MessageID;
    my_mh.MessageHeader.MessageType = USBPD_CONTROL_MSG_GOODCRC;
    my_mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
    *(uint16_t*)&PD_TR_buffer[0] =  my_mh.d16;
    PD_sendData(2, USBPD_TX_SOP0);
  }
}

// ===================================================================================
// USB PD Interrupt Service Routine
// ===================================================================================
void USBPD_IRQHandler(void) __attribute__((interrupt));
void USBPD_IRQHandler(void) {

  // Receive complete interrupt
  if(USBPD->STATUS & USBPD_IF_RX_ACT) {
    uint8_t status = (USBPD->STATUS & USBPD_BMC_AUX);
    if(status == USBPD_BMC_AUX_SOP0) {
      if(USBPD->BMC_BYTE_CNT >= 6) {
        PD_RX_analyze();
      }
    }
    USBPD->STATUS |= USBPD_IF_RX_ACT;
  }

  // Transmit complete interrupt (GoodCRC only)
  if(USBPD->STATUS & USBPD_IF_TX_END) {
    USBPD->PORT_CC1 &= ~USBPD_CC_LVE;
    USBPD->PORT_CC2 &= ~USBPD_CC_LVE;
    PD_RX_mode();
    PD_control.SinkGoodCRCOver = 1;
    USBPD->STATUS |= USBPD_IF_TX_END;
  }

  // Reset interrupt
  if(USBPD->STATUS & USBPD_IF_RX_RESET) {
    USBPD->STATUS |= USBPD_IF_RX_RESET;
    PD_reset();
  }
}


// Get PD Specification Revision 1,2,3
uint8_t PD_getRevision(void) {
  return PD_control.PD_Version + 1;
}
