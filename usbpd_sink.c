// ===================================================================================
// USB PD SINK Handler for CH32X035
// ===================================================================================
//
// Reference:               https://github.com/openwch/ch32x035
// 2023 by Stefan Wagner:   https://github.com/wagiminator
// 2025 by Unagi Dojyou:    https://unagidojyou.com

#include "usbpd_sink.h"
#include <system.h>
#include "usbpd_emarker.h"
#include "usb_cdc.h"

// Variables
static pd_control_t PD_control = {
  .CC_State = CC_IDLE,
  .CC1_ConnectTimes = 0,
  .CC2_ConnectTimes = 0,
};

FixedSourceCap_t PD_SC_fixed[7];
PPSSourceCap_t   PD_SC_PPS[7]; 
FixedSourceCap_t PD_SC_EPR_Fixed[7];
AVSSourceCap_t   PD_SC_AVS[7];

// Buffers
__attribute__ ((aligned(4))) uint8_t PD_TR_buffer[264];  // PD transmit/receive buffer (Increased for EPR) 
__attribute__ ((aligned(4))) uint8_t PD_SC_buffer[64];  // PD Source Cap buffer (Increased to 16 PDOs)

// ===================================================================================
// USB PD SINK Front End Functions
// ===================================================================================

// Prototype
uint8_t PD_checkCC(void);
uint8_t PD_update(void);
volatile uint16_t PD_RX_Log_Buffer[16];
volatile uint8_t PD_RX_Log_W = 0;
volatile uint8_t PD_RX_Log_R = 0;

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
  return PD_control.SourcePDONum - PD_control.SourcePPSNum - PD_control.SourceAVSNum - PD_control.SourceEPRFixedNum;
}

// Get number of programmable power PDOs
uint8_t PD_getPPSNum(void) {
  return PD_control.SourcePPSNum;
}

// Get number of EPR Fixed PDOs
uint8_t PD_getEPRFixedNum(void) {
  return PD_control.SourceEPRFixedNum;
}

// Get number of AVS PDOs
uint8_t PD_getAVSNum(void) {
  return PD_control.SourceAVSNum;
}

// Get voltage of specified fixed power PDO
uint16_t PD_getPDOVoltage(uint8_t pdonum) {
  return PD_control.FixedSourceCap[pdonum - 1].Voltage;
}

// Get minimum voltage of specified PDO (fixed and programmable)
uint16_t PD_getPDOMinVoltage(uint8_t pdonum) {
  if(pdonum <= PD_getFixedNum()) {
    return PD_control.FixedSourceCap[pdonum - 1].Voltage;
  } else if(pdonum <= PD_getFixedNum() + PD_getPPSNum()) {
    return PD_control.PPSSourceCap[pdonum - PD_getFixedNum() - 1].MinVoltage;
  } else if(pdonum <= PD_getFixedNum() + PD_getPPSNum() + PD_control.SourceEPRFixedNum) {
    // EPR Fixed
    return PD_control.EPRFixedSourceCap[pdonum - PD_getFixedNum() - PD_getPPSNum() - 1].Voltage;
  } else {
    // AVS
    return PD_control.AVSSourceCap[pdonum - PD_getFixedNum() - PD_getPPSNum() - PD_control.SourceEPRFixedNum - 1].MinVoltage;
  }
}

// Get maximum voltage of specified PDO (fixed and programmable)
uint16_t PD_getPDOMaxVoltage(uint8_t pdonum) {
  if(pdonum <= PD_getFixedNum()) {
    return PD_control.FixedSourceCap[pdonum - 1].Voltage;
  } else if(pdonum <= PD_getFixedNum() + PD_getPPSNum()) {
    return PD_control.PPSSourceCap[pdonum - PD_getFixedNum() - 1].MaxVoltage;
  } else if(pdonum <= PD_getFixedNum() + PD_getPPSNum() + PD_control.SourceEPRFixedNum) {
    // EPR Fixed
    return PD_control.EPRFixedSourceCap[pdonum - PD_getFixedNum() - PD_getPPSNum() - 1].Voltage;
  } else {
    // AVS
    return PD_control.AVSSourceCap[pdonum - PD_getFixedNum() - PD_getPPSNum() - PD_control.SourceEPRFixedNum - 1].MaxVoltage;
  }
}

// Get max current of specified PDO (fixed and programmable)
uint16_t PD_getPDOMaxCurrent(uint8_t pdonum) {
  if(pdonum <= PD_getFixedNum()) {
    return PD_control.FixedSourceCap[pdonum - 1].Current;
  } else if(pdonum <= PD_getFixedNum() + PD_getPPSNum()) {
    return PD_control.PPSSourceCap[pdonum - PD_getFixedNum() - 1].Current;
  } else if(pdonum <= PD_getFixedNum() + PD_getPPSNum() + PD_control.SourceEPRFixedNum) {
    // EPR Fixed
    return PD_control.EPRFixedSourceCap[pdonum - PD_getFixedNum() - PD_getPPSNum() - 1].Current;
  } else {
    // AVS
    return 0; // AVS defines Power, not Current
  }
}

// Get PPS Power Limited flag (p = 1..PD_getPDONum())
uint8_t PD_getPPSPowerLimited(uint8_t pdonum) {
  uint8_t ppspos = PD_control.SourcePDONum - PD_control.SourcePPSNum;
  if (pdonum <= ppspos)
    return 0; // Not a PPS or Fixed PDO
  else return PD_control.PPSSourceCap[pdonum - ppspos - 1].PPSPowerLimited;
}

// Set specified PDO and voltage; returns 0:failed, 1:success
uint8_t PD_setPDO(uint8_t pdonum, uint16_t voltage) {
  PD_control.SetPDONum  = pdonum;
  PD_control.SetVoltage = voltage;
  PD_control.SetCurrent = PD_getPDOMaxCurrent(pdonum); //modified by unagidojyou
  if (pdonum <= (PD_control.SourcePDONum - PD_control.SourcePPSNum)) {
    PD_control.SetRequestType = REQ_FIXED;
  } else {
    PD_control.SetRequestType = REQ_PPS;
  }
  return PD_negotiate();
}

// Set specified voltage (in millivolts) if available; returns 0:failed, 1:success
uint8_t PD_setVoltage(uint16_t voltage) {
  uint8_t i;
  uint8_t ppspos = PD_control.SourcePDONum - PD_control.SourcePPSNum;
  for(i=0; i<PD_control.SourcePDONum; i++) {
    if(i < ppspos) {
      if(PD_control.FixedSourceCap[i].Voltage == voltage) {
        return PD_setPDO(i + 1, voltage);
      }
    }
    else {
      if((PD_control.PPSSourceCap[i-ppspos].MinVoltage <= voltage) &&
         (PD_control.PPSSourceCap[i-ppspos].MaxVoltage >= voltage)) {
        return PD_setPDO(i + 1, voltage);
      }
    }
  }
  return 0;
}

// Set specified PDO and voltage; returns 0:failed, 1:success
uint8_t PD_setPDOwithCurrent(uint8_t pdonum, uint16_t voltage ,uint16_t current) {
  PD_control.SetPDONum  = pdonum;
  PD_control.SetVoltage = voltage;
  PD_control.SetCurrent = current;
  if (pdonum <= (PD_control.SourcePDONum - PD_control.SourcePPSNum)) {
    PD_control.SetRequestType = REQ_FIXED;
  } else {
    PD_control.SetRequestType = REQ_PPS;
  }
  return PD_negotiate();
}

// Set specified voltage and current (in millivolts and milliampere) if available;
// returns 0:failed, 1:success
uint8_t PD_setPPS(uint16_t voltage,uint16_t current) {
  uint8_t i;
  uint8_t ppspos = PD_control.SourcePDONum - PD_control.SourcePPSNum;  //num of nonpps pod
  if(!PD_control.SourcePPSNum) return 0;
  for(i=ppspos; i<PD_control.SourcePDONum; i++) {
    if((PD_control.PPSSourceCap[i-ppspos].MinVoltage <= voltage) &&
        (PD_control.PPSSourceCap[i-ppspos].MaxVoltage >= voltage) &&
        (PD_control.PPSSourceCap[i-ppspos].Current >= current) &&
        (50 <= current)) { //check voltage and current
      return PD_setPDOwithCurrent(i + 1, voltage, current);
    }
  }
  return 0;
}

// Set EPR Mode
uint8_t PD_setEPRMode(uint8_t enable) {
  uint8_t epr_cap_bit = (PD_SC_buffer[2] & 0x80) ? 1 : 0;
  CDC_printf("EPR: SetMode %d Ver=%d CapBit=%d\n", enable, PD_control.PD_Version, epr_cap_bit);
  if (!PD_control.USBPD_READY) {
       CDC_printf("EPR: Not Ready\n");
       return 0; // Check physical connection
  }
  PD_control.EPR_Mode = enable;
  PD_control.CC_State = CC_EPR_MODE_ENTRY;
  if (PD_negotiate()) {
      if (PD_control.SourceEPRNum > 0) {
           CDC_printf("EPR: Success (Num=%d)\n", PD_control.SourceEPRNum);
           return 1;
      }
      CDC_printf("EPR: Fail (No PDOs)\n");
  } else {
      CDC_printf("EPR: Timeout/Fail\n");
  }
  return 0;
}

// Set specified EPR PDO, Voltage and Current
uint8_t PD_setEPRPDO(uint8_t pdonum, uint16_t voltage, uint16_t current) {
  PD_control.SetPDONum  = pdonum;
  PD_control.SetVoltage = voltage;
  PD_control.SetCurrent = current;
  PD_control.SetRequestType = REQ_EPR_FIXED; // Default
  // Check if AVS
  // If pdoNum is in AVS range (After SPR Fixed, SPR PPS, and EPR Fixed)
  if (pdonum > (PD_getFixedNum() + PD_getPPSNum() + PD_control.SourceEPRFixedNum)) { 
      PD_control.SetRequestType = REQ_EPR_AVS;
  }
  return PD_negotiate();
}

// Set specified EPR Voltage/Current
uint8_t PD_setEPRVoltage(uint16_t voltage, uint16_t current) {
    uint8_t i;
    // Check Fixed PDOs (EPR Fixed are stored in FixedSourceCap)
    for(i=0; i<PD_getFixedNum(); i++) {
         if ((PD_control.FixedSourceCap[i].Voltage == voltage) && (PD_control.FixedSourceCap[i].Current >= current)) {
             return PD_setEPRPDO(i + 1, voltage, current);
         }
    }
    // Check EPR Fixed PDOs
    for(i=0; i<PD_control.SourceEPRFixedNum; i++) {
         if ((PD_control.EPRFixedSourceCap[i].Voltage == voltage) && (PD_control.EPRFixedSourceCap[i].Current >= current)) {
             // Index: FixedNum + PPSNum + i + 1
             uint8_t pdoIndex = PD_getFixedNum() + PD_getPPSNum() + i + 1;
             return PD_setEPRPDO(pdoIndex, voltage, current);
         }
    }
    // Check AVS PDOs
    for(i=0; i<PD_control.SourceAVSNum; i++) {
         // Index for AVS
         uint8_t pdoIndex = PD_getFixedNum() + PD_getPPSNum() + PD_control.SourceEPRFixedNum + i + 1;
         if ((PD_control.AVSSourceCap[i].MinVoltage <= voltage) && (PD_control.AVSSourceCap[i].MaxVoltage >= voltage)) {
               // AVS Request
               return PD_setEPRPDO(pdoIndex, voltage, current);
         }
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

// Check if PD is ready
uint8_t PD_isReady(void) {
  return PD_control.USBPD_READY;
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
  CDC_print("PD RESET From State="); CDC_printD(PD_control.CC_State); CDC_println("");
  USBPD->PORT_CC1 = USBPD_CC_CMP_66 | USBPD_CC_PD;
  USBPD->PORT_CC2 = USBPD_CC_CMP_66 | USBPD_CC_PD;
  PD_control.CC1_ConnectTimes  = 0;
  PD_control.CC2_ConnectTimes  = 0;
  PD_control.CC_NoneTimes      = 0;
  PD_control.SourcePDONum      = 0;
  PD_control.SourcePPSNum      = 0;
  PD_control.SourceEPRFixedNum = 0;
  PD_control.SourceAVSNum      = 0;
  PD_control.SourceEPRNum      = 0;
  PD_control.EPR_ChunkRequest  = 0;
  PD_control.EPR_NextChunk     = 0;
  PD_control.EPR_MessageStatus = 0;
  PD_control.FixedSourceCap    = PD_SC_fixed;
  PD_control.PPSSourceCap      = PD_SC_PPS;
  PD_control.EPRFixedSourceCap = PD_SC_EPR_Fixed;
  PD_control.AVSSourceCap      = PD_SC_AVS;
  PD_control.CC_State          = CC_IDLE;
  PD_control.CC_LastState      = CC_IDLE;
  PD_control.SinkMessageID     = 0;
  PD_control.SinkGoodCRCOver   = 0;
  PD_control.SourceGoodCRCOver = 0;
  PD_control.PD_Version        = USBPD_REVISION_20;
  PD_control.USBPD_READY       = 0;
  PD_control.SetPDONum         = 1;
  PD_control.LastSetPDONum     = 1;
  PD_control.SetVoltage        = 5000;
  PD_control.LastSetVoltage    = 5000;
  PD_control.SetRequestType    = REQ_FIXED;
  PD_control.PDO_Mismatch      = 0;
  PD_control.EPR_Mode          = 0;
}

// Copy buffers
void PD_memcpy(uint8_t* dest, const uint8_t* src, uint8_t n) {
  while(n--) *dest++ = *src++;
}

// Send PD data
void PD_sendData(uint8_t length, uint16_t sop) {
  if((USBPD->CONFIG & USBPD_CC_SEL) == USBPD_CC_SEL) USBPD->PORT_CC2 |= USBPD_CC_LVE;
  else                                               USBPD->PORT_CC1 |= USBPD_CC_LVE;

  USBPD->BMC_CLK_CNT = USBPD_TMR_TX;
  USBPD->TX_SEL      = sop;
  USBPD->BMC_TX_SZ   = length;
  USBPD->STATUS      = 0;
  USBPD->CONTROL    |= USBPD_BMC_START | USBPD_PD_TX_EN;
}

// Detect CC connection; returns 0:No connection, 1:CC1 connection, 2:CC2 connection
uint8_t PD_checkCC(void) {
  uint8_t ccLine = USBPD_CCNONE;

  USBPD->PORT_CC1 &= ~(USBPD_CC_CE | USBPD_PA_CC_AI);
  USBPD->PORT_CC1 |= USBPD_CC_CMP_22;
  if(USBPD->PORT_CC1 & USBPD_PA_CC_AI) ccLine = USBPD_CC1;

  USBPD->PORT_CC2 &= ~(USBPD_CC_CE | USBPD_PA_CC_AI);
  USBPD->PORT_CC2 |= USBPD_CC_CMP_22;
  if(USBPD->PORT_CC2 & USBPD_PA_CC_AI) ccLine = USBPD_CC2;

  return ccLine;
}

#warning "PDO送信時のEPRフラグの管理"
#warning "ログの削除"
#warning "空データのPDOの処理"

// Analyze PDOs
void PD_PDO_analyze(void) {
  USBPD_PDO_t test;
  PD_control.SourcePPSNum = 0;
  PD_control.SourceEPRFixedNum = 0;
  PD_control.SourceAVSNum = 0;
  // If EPR mode, we might want to clear SourcePDONum (Fixed count) or separate it.
  // But FixedSourceCap is reused.
  // Wait, SourcePDONum is usually total. And FixedNum derived.
  // Let explicitly count Fixed.
  uint8_t fixedCount = 0;

  uint8_t numPDOs = (PD_control.CC_State == CC_EPR_SOURCE_CAP) ? PD_control.SourceEPRNum : PD_control.SourcePDONum;
  
  //CDC_print("Analyze: State="); CDC_printD(PD_control.CC_State); CDC_print(" Num="); CDC_printD(numPDOs); CDC_println("");

  for(uint8_t i=0; i<numPDOs; i++) { 
    test.d32 = *(uint32_t*)(&PD_SC_buffer[i*4]);
    //CDC_print("PDO"); CDC_printD(i); CDC_print(": "); CDC_printH(test.d32 >> 16); CDC_printH(test.d32 & 0xFFFF); CDC_println("");
    
    //CDC_print("  Aug="); CDC_printD(test.SourceEPRPDO.AugmentedPowerDataObject);
    //CDC_print(" EPRProg="); CDC_printD(test.SourceEPRPDO.EPRprogrammablePowerSupply);
    //CDC_print(" SPRProg="); CDC_printD(test.SourcePPSPDO.SPRprogrammablePowerSupply);
    //CDC_println(""); 
    
    // Check for AVS (Augmented 11b, Subtype 01b)
    if((test.SourceEPRPDO.AugmentedPowerDataObject == 3u) && (test.SourceEPRPDO.EPRprogrammablePowerSupply == 1u)) {
       //CDC_print(" -> AVS"); CDC_println("");
       if (PD_control.SourceAVSNum < 7) {
           PD_control.AVSSourceCap[PD_control.SourceAVSNum].MinVoltage = POWER_DECODE_100MV(test.SourceEPRPDO.MinVoltageIn100mVincrements);
           PD_control.AVSSourceCap[PD_control.SourceAVSNum].MaxVoltage = POWER_DECODE_100MV(test.SourceEPRPDO.MaxVoltageIn100mVincrements);
           PD_control.AVSSourceCap[PD_control.SourceAVSNum].PDP        = test.SourceEPRPDO.MaxPowerIn1Wincrements;
           PD_control.SourceAVSNum++;
       }
    }
    // Check for PPS (Augmented 11b, Subtype 00b)
    else if((test.SourcePPSPDO.AugmentedPowerDataObject==3u) && (test.SourcePPSPDO.SPRprogrammablePowerSupply==0u)) {
      if (PD_control.SourcePPSNum < 7) {
          PD_control.PPSSourceCap[PD_control.SourcePPSNum].MaxVoltage = POWER_DECODE_100MV(test.SourcePPSPDO.MaxVoltageIn100mVincrements);
          PD_control.PPSSourceCap[PD_control.SourcePPSNum].MinVoltage = POWER_DECODE_100MV(test.SourcePPSPDO.MinVoltageIn100mVincrements);
          PD_control.PPSSourceCap[PD_control.SourcePPSNum].Current    = POWER_DECODE_50MA(test.SourcePPSPDO.MaxCurrentIn50mAincrements);
          PD_control.PPSSourceCap[PD_control.SourcePPSNum].PPSPowerLimited = test.SourcePPSPDO.PPSpowerLimited;
          PD_control.SourcePPSNum++;
      }
    }
    // Fixed Supply (00b)
    else {
      // Logic: If PPS has been seen OR Voltage > 20V, this is EPR Fixed.
      // Else, SPR Fixed.
      if ((PD_control.SourcePPSNum > 0) || (test.SourceFixedPDO.VoltageIn50mVunits > 400)) {
          // EPR Fixed
          if (PD_control.SourceEPRFixedNum < 7) {
              PD_control.EPRFixedSourceCap[PD_control.SourceEPRFixedNum].Current = POWER_DECODE_10MA(test.SourceFixedPDO.MaxCurrentIn10mAunits);
              PD_control.EPRFixedSourceCap[PD_control.SourceEPRFixedNum].Voltage = POWER_DECODE_50MV(test.SourceFixedPDO.VoltageIn50mVunits);
              PD_control.SourceEPRFixedNum++;
          }
      } else {
          // SPR Fixed
          if (fixedCount < 7) {
              PD_control.FixedSourceCap[fixedCount].Current = POWER_DECODE_10MA(test.SourceFixedPDO.MaxCurrentIn10mAunits);
              PD_control.FixedSourceCap[fixedCount].Voltage = POWER_DECODE_50MV(test.SourceFixedPDO.VoltageIn50mVunits);
              fixedCount++;
          }
      }
    }
  }
  // Update SourcePDONum to reflect total valid PDOs processed?
  if (PD_control.CC_State == CC_EPR_SOURCE_CAP) {
      //CDC_print("Update PDONum: ");
      PD_control.SourcePDONum = numPDOs; // Unify
      //CDC_printD(PD_control.SourcePDONum);
      //CDC_println("");
  }
}

#warning "チャンクのUnchunked対応のフラグは管理はどうなってる？"

// Send specified PDO
void PD_PDO_request(void) {
  uint8_t pdoNum = PD_control.SetPDONum;
  USBPD_SINKRDO_t pdo;
  USBPD_MessageHeader_t mh;
  mh.d16  = 0u;
  pdo.d32 = 0u;

  mh.MessageHeader.MessageID             = PD_control.SinkMessageID ;
  
  #warning "EPR Requestの実装方法"
  if (PD_control.SetRequestType >= REQ_EPR_FIXED || PD_control.EPR_Mode == 3) {
      mh.MessageHeader.MessageType           = USBPD_DATA_MSG_EPR_REQUEST; // 0x09
      mh.MessageHeader.NumberOfDataObjects   = 2u; // RDO + PDO Copy
      mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
      
      if (PD_control.SetRequestType == REQ_EPR_FIXED || PD_control.SetRequestType == REQ_FIXED) {
        pdo.SinkFixedVariableRDO.ObjectPosition = pdoNum;
        pdo.SinkFixedVariableRDO.MaxOperatingCurrent10mAunits = PD_control.SetCurrent / 10;
        pdo.SinkFixedVariableRDO.OperatingCurrentIn10mAunits = PD_control.SetCurrent / 10;
        pdo.SinkFixedVariableRDO.USBCommunicationsCapable     = 0u; // V1V2 doesn't have USB communication
        pdo.SinkFixedVariableRDO.UnchunkedExtendedMessage     = 1u;
        pdo.SinkFixedVariableRDO.EPRModeCapable               = 1u;
        pdo.SinkFixedVariableRDO.NoUSBSuspend                 = 1u;
      } else { // REQ_EPR_AVS
        pdo.SinkAVSRDO.ObjectPosition = pdoNum;
        pdo.SinkAVSRDO.OutputVoltageIn20mVunits = PD_control.SetVoltage / 20;
        pdo.SinkAVSRDO.OperatingCurrentIn50mAunits = PD_control.SetCurrent / 50;
        pdo.SinkAVSRDO.UnchunkedExtendedMessage = 1u;
        pdo.SinkAVSRDO.NoUSBSuspend = 1u;
      }
      *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
      CDC_print("TX: EPRR H="); CDC_printH(mh.d16); CDC_print(" DO="); CDC_printH(pdo.d32 >> 16); CDC_printH(pdo.d32 & 0xFFFF);
      CDC_print(" Mode="); CDC_printD(PD_control.EPR_Mode); CDC_println("");
      
      PD_memcpy(&PD_TR_buffer[2], (uint8_t*)&pdo.d32, 4);
      PD_memcpy(&PD_TR_buffer[6], &PD_SC_buffer[(pdoNum-1)*4], 4); // Copy requested PDO
      PD_sendData(10, USBPD_TX_SOP0); // Header + 2 Objects
  }
  else {
      mh.MessageHeader.MessageType           = USBPD_DATA_MSG_REQUEST;
      mh.MessageHeader.NumberOfDataObjects   = 1u;
      mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
    
      if(pdoNum > (PD_control.SourcePDONum - PD_control.SourcePPSNum)) {
        pdo.SinkPPSRDO.ObjectPosition              = pdoNum;
        pdo.SinkPPSRDO.OutputVoltageIn20mVunits    = PD_control.SetVoltage / 20;
        pdo.SinkPPSRDO.OperatingCurrentIn50mAunits = PD_control.SetCurrent / 50;
        pdo.SinkPPSRDO.NoUSBSuspend                = 1u;
        pdo.SinkPPSRDO.USBCommunicationsCapable    = 0u; // V1V2 doesn't have USB communication
        
        if (PD_control.PD_Version >= 2) {
          pdo.SinkPPSRDO.UnchunkedExtendedMessage     = 1u;
          pdo.SinkPPSRDO.EPRModeCapable               = 1u;
        }
      }
      else {
        pdo.SinkFixedVariableRDO.ObjectPosition               = pdoNum;
        pdo.SinkFixedVariableRDO.MaxOperatingCurrent10mAunits = PD_SC_fixed[pdoNum-1].Current/10;
        pdo.SinkFixedVariableRDO.OperatingCurrentIn10mAunits  = PD_SC_fixed[pdoNum-1].Current/10;
        pdo.SinkFixedVariableRDO.USBCommunicationsCapable     = 0u; // V1V2 doesn't have USB communication
        pdo.SinkFixedVariableRDO.NoUSBSuspend                 = 1u;
        pdo.SinkFixedVariableRDO.CapabilityMismatch           = PD_control.PDO_Mismatch;
        
        if (PD_control.PD_Version >= 2) { // このif文必要？
          pdo.SinkFixedVariableRDO.UnchunkedExtendedMessage     = 1u;
          pdo.SinkFixedVariableRDO.EPRModeCapable               = 1u;
        }
      }
    
      
      *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
      CDC_print("TX: RDO H="); CDC_printH(mh.d16); CDC_print(" DO="); CDC_printH(pdo.d32 >> 16); CDC_printH(pdo.d32 & 0xFFFF); CDC_println("");
      PD_memcpy(&PD_TR_buffer[2], (uint8_t*)&pdo.d32, 4);
      PD_sendData(6, USBPD_TX_SOP0);
  }
}

// Send EPR Mode Message
void PD_Send_EPR_Mode(uint8_t action) {
  USBPD_MessageHeader_t mh;
  
  // Manual Header Construction (Rev 3.0, EPR_MODE type)
  mh.d16 = (1 << 12) | ((PD_control.SinkMessageID & 0x7) << 9) | ((PD_control.PD_Version & 0x3) << 6) | USBPD_DATA_MSG_EPR_MODE;

  // Manual Byte Assignment for Action (Byte 3 of Data Object)
  // Wire: [0][1]Header, [2][3][4][5]Data.
  PD_TR_buffer[2] = 0;
  PD_TR_buffer[3] = 0;
  PD_TR_buffer[4] = 0;
  PD_TR_buffer[5] = action; 

  DLY_ms(15);
  *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
  CDC_print("TX H="); CDC_printH(mh.d16); CDC_println("");
  PD_sendData(6, USBPD_TX_SOP0);
  
  CDC_print("EPR: Sent Action "); CDC_printH(action); CDC_println("");
}

void PD_process(void) {
  cc_state_t temp = PD_control.CC_State;
  USBPD_MessageHeader_t mh;

  // RX Log Processing (Non-blocking print)
  while(PD_RX_Log_R != PD_RX_Log_W) {
      CDC_print("RX Log: "); CDC_printH(PD_RX_Log_Buffer[PD_RX_Log_R]); CDC_println("");
      PD_RX_Log_R = (PD_RX_Log_R + 1) & 0x0F;
  }

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
         (PD_control.SetVoltage  != PD_control.LastSetVoltage)) {
        PD_control.LastSetPDONum  = PD_control.SetPDONum;
        PD_control.LastSetVoltage = PD_control.SetVoltage;
        PD_control.USBPD_READY    = 0; 

        if (PD_control.EPR_Mode > 0) {
            // In EPR Mode, send Request directly
            PD_control.CC_State = CC_SEND_REQUEST;
        } else {
            mh.d16 = 0u;
            mh.MessageHeader.MessageID             = PD_control.SinkMessageID;
            mh.MessageHeader.MessageType           = USBPD_CONTROL_MSG_GET_SRC_CAP;
            mh.MessageHeader.NumberOfDataObjects   = 0u;
            mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
            *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
            PD_sendData(2, USBPD_TX_SOP0);
            PD_control.CC_State = CC_GET_SOURCE_CAP+1;
        }
      }
      break;

    case CC_EPR_MODE_ENTRY:
      if(PD_control.CC_LastState != PD_control.CC_State) {
          // Send EPR Mode Enter
          if (PD_control.EPR_Mode == 1) {
             PD_Send_EPR_Mode(0x01);
          }
      }
       if (PD_control.EPR_Mode == 2) {
             // Received Enter Ack, Waiting for Enter Succeeded from Source
             // Do nothing (or verify timeout)
       }
       else if (PD_control.EPR_Mode == 3) {
             // Received Enter Succeeded from Source. EPR Mode Active.
             // Wait for EPR Source Caps (MsgType 0x15: USBPD_EXT_MSG_SOURCE_CAP_EPR)
       }
       else if (PD_control.EPR_Mode == 0) {
          // EPR Entry Failed (Rejected or Not Supported)
          // Revert to SPR (Get Source Cap) or just set Ready?
          PD_control.CC_State = CC_GET_SOURCE_CAP;
          PD_control.WaitTime = 0;
      }
      break;

    case CC_EPR_SOURCE_CAP:
       if(PD_control.SinkGoodCRCOver) {
        PD_control.SinkGoodCRCOver = 0;
        
        if (PD_control.EPR_ChunkRequest) {
             // Send Chunk Request
             USBPD_MessageHeader_t mh;
             USBPD_ExtendedMessageHeader_t emh;
             
             mh.d16 = 0;
             mh.MessageHeader.MessageType = USBPD_EXT_MSG_SOURCE_CAP_EPR; // 0x15
             mh.MessageHeader.Extended = 1;
             mh.MessageHeader.NumberOfDataObjects = 1; // Ext Header is 1 object
             mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
             mh.MessageHeader.MessageID = PD_control.SinkMessageID;
             
             emh.d16 = 0;
             emh.d16 = (1 << 15) | ((PD_control.EPR_NextChunk & 0x0F) << 11) | (1 << 10);
//             emh.Header.Chunked = 1;
//             emh.Header.RequestChunk = 1;
//             emh.Header.ChunkNumber = PD_control.EPR_NextChunk;
//             emh.Header.DataSize = 0;
             
             *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
             *(uint16_t*)&PD_TR_buffer[2] = emh.d16;
             // Padding
             PD_TR_buffer[4] = 0;
             PD_TR_buffer[5] = 0;
             
             CDC_print("TX: EPR Chunk Req CNum="); CDC_printH(PD_control.EPR_NextChunk); 
             CDC_print(" H="); CDC_printH(mh.d16); CDC_print(" EMH="); CDC_printH(emh.d16); CDC_println("");
             
             DLY_ms(10);
             PD_sendData(6, USBPD_TX_SOP0);
             PD_control.EPR_ChunkRequest = 0; // Request Sent, wait for RX
        } else if (PD_control.EPR_MessageStatus == 2) {
             // Complete
             NVIC_DisableIRQ(USBPD_IRQn);
             PD_PDO_analyze();
             NVIC_EnableIRQ(USBPD_IRQn);
             PD_control.CC_State = CC_SEND_REQUEST;
             PD_control.EPR_MessageStatus = 0; // Reset
             // PD_control.USBPD_READY = 1; // Now Ready - REMOVED: Wait for RDO exchange to complete
        }
      }
      break;

    default:
      break;
  }
  PD_control.CC_LastState = temp;
}

// Update PD, return 1 if PDO is changed
uint8_t PD_update(void) {
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
  
  mh.d16 = 0;
  mh.MessageHeader.MessageID = PD_control.SinkMessageID;
  mh.MessageHeader.MessageType = USBPD_EXT_MSG_EPR_KEEP_ALIVE; 
  mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
  mh.MessageHeader.Extended = 1;
  
  emh.d16 = 0;
  emh.Header.DataSize = 4; // 1 Data Object (4 bytes)
  emh.Header.Chunked = 0; // Unchunked
  
  *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
  *(uint16_t*)&PD_TR_buffer[2] = emh.d16;
  PD_TR_buffer[4] = 0;
  PD_TR_buffer[5] = 0;
  PD_TR_buffer[6] = 0;
  PD_TR_buffer[7] = 0;
  
  PD_sendData(8, USBPD_TX_SOP0); 
}

// Main Loop function, handles Update and PPS/EPR Keep-Alive
uint8_t PD_Loop(void) {
  static uint32_t last_time = 0;
  uint8_t status = PD_update();

  // EPR KeepAlive Logic (Every 375ms)
  if (PD_control.EPR_Mode == 3) {
    if ((STK->CNTL - last_time) > (375 * DLY_MS_TIME)) {
      last_time = STK->CNTL;
      if (PD_isReady()) {
        PD_sendEPRKeepAlive();
      }
    }
  } else if (PD_control.USBPD_READY && PD_control.SetRequestType == REQ_PPS && PD_control.PDO_Mismatch == 0) {
    // Handle timer wrap-around (using unsigned subtraction)
    // Check if 5000ms has passed
    if ((STK->CNTL - last_time) > (5000 * DLY_MS_TIME)) {
      last_time = STK->CNTL;
      if (PD_isReady()) {
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
  mh.d16 = *(uint16_t*)PD_TR_buffer;
  
  // Non-blocking Log
  PD_RX_Log_Buffer[PD_RX_Log_W] = mh.d16;
  PD_RX_Log_W = (PD_RX_Log_W + 1) & 0x0F;
  
  //CDC_print("RX: All ");
  //CDC_printH(mh.d16);
  //CDC_println("");
  
  USBPD_ExtendedMessageHeader_t emh;
  // If Extended, the 2nd word is Extended Header.
  // But wait, the buffer order:
  // [0..1] Header
  // [2..3] Extended Header (if extended)
  // [4..] Data

  if(mh.d16 & 0x8000) {
    emh.d16 = *(uint16_t*)&PD_TR_buffer[2];
     // Handle Extended Messages
      // Allow Unchunked or Chunked messages (Logic inside switch handles specifics)

      #warning "チャンクの実装これでいいのか？"
      if (1) { 
          switch(mh.MessageHeader.MessageType) {
              case USBPD_EXT_MSG_SOURCE_CAP_EPR:
                 PD_control.CC_State = CC_EPR_SOURCE_CAP;
                 
                 uint16_t totalDataSize = emh.d16 & 0x3FF;
                 uint8_t chunkNum = (emh.d16 >> 11) & 0x0F;
                 
                 // CDC_printf("RX: EPR Cap DSize=%d CNum=%d\n", totalDataSize, chunkNum);
                 // CDC_print("MH="); CDC_printH(mh.d16); CDC_print(" EMH="); CDC_printH(emh.d16); CDC_println("");

                 // CDC_printf("RX: EPR Cap CNum=%d\n", chunkNum);
                 CDC_print("MH="); CDC_printH(mh.d16); CDC_print(" EMH="); CDC_printH(emh.d16); CDC_println("");
                 
                 // Extended: Len = (NumObj * 4) - 2
                 uint8_t numObj = (mh.d16 >> 12) & 0x7;
                 if (numObj == 0) numObj = 1; // Safety?
                 uint8_t payloadSize = (numObj * 4) - 2;
                 
                 // Fallback: If Header DataSize is 0 (invalid/parsing error) but we have payload, use payload size
                 if (totalDataSize == 0 && payloadSize > 0) {
                     totalDataSize = payloadSize;
                 }
                 
                 // Safety copy
                 if ((chunkNum * 26) + payloadSize <= 64) {
                     PD_memcpy(&PD_SC_buffer[chunkNum * 26], &PD_TR_buffer[4], payloadSize);
                 }
                 
                 // Check if complete
                 uint8_t complete = 0;
                 uint16_t currentTotal = (chunkNum * 26) + payloadSize;
                 
                 if (totalDataSize > 0) {
                     if (currentTotal >= totalDataSize) complete = 1;
                 } else {
                     // Heuristic: If payload is less than max chunk size (26), it's the last one.
                     if (payloadSize < 26) complete = 1;
                 }
                 
                 if (complete) {
                     // All chunks received
                     PD_control.EPR_ChunkRequest = 0;
                     PD_control.EPR_MessageStatus = 2; // Complete
                     if (totalDataSize > 0) {
                         PD_control.SourceEPRNum = totalDataSize / 4;
                     } else {
                         PD_control.SourceEPRNum = currentTotal / 4;
                     }
                     
                     // Fallback check for 0
                     if (PD_control.SourceEPRNum == 0 && currentTotal > 0) {
                          PD_control.SourceEPRNum = currentTotal / 4;
                     }
                     // Ready to analyze in PD_process
                 } else {
                     // Need more chunks
                     PD_control.EPR_NextChunk = chunkNum + 1;
                     PD_control.EPR_ChunkRequest = 1;
                     PD_control.EPR_MessageStatus = 1; // Incomplete/Wait
                 }
                 break;
                 
             case USBPD_EXT_MSG_PPS_STATUS:
                 // Handle PPS/EPR Status?
                 break;
                 
             default:
                 break;
         }
    }
  }
  else {
    // Standard Messages
    if(mh.MessageHeader.NumberOfDataObjects == 0u) {
      switch(mh.MessageHeader.MessageType) {

        case USBPD_CONTROL_MSG_GOODCRC:
          sendGoodCRCFlag = 0;
          PD_control.SourceGoodCRCOver = 1;
          PD_control.SinkMessageID++;
          break;

        case USBPD_CONTROL_MSG_ACCEPT:
          if(PD_control.CC_State == CC_EPR_MODE_ENTRY) {
               // Accept of EPR Mode? No, EPR Mode uses Data Messages.
          }
          PD_control.CC_State = CC_WAIT_PS_RDY;
          break;

        case USBPD_CONTROL_MSG_PS_RDY:
          PD_control.CC_State = CC_PS_RDY;
          break;

        case USBPD_CONTROL_MSG_NOT_SUPPORTED:
          PD_control.PPS_Not_Supported = 1;
          if (PD_control.CC_State == CC_EPR_MODE_ENTRY) {
            #warning "ここらへんの実装いる？"
              PD_control.EPR_Mode = 0; // Failed
          }
          break;

        case USBPD_CONTROL_MSG_REJECT:
          PD_control.PPS_Not_Supported = 1;
           if (PD_control.CC_State == CC_EPR_MODE_ENTRY) {
              PD_control.EPR_Mode = 0; // Failed
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
          CDC_println("SPR PDO");
          PD_control.CC_State = CC_SOURCE_CAP;
          PD_control.SourcePDONum = mh.MessageHeader.NumberOfDataObjects;
          PD_control.PD_Version = mh.MessageHeader.SpecificationRevision;
          PD_memcpy(PD_SC_buffer, &PD_TR_buffer[2], 28);
          // Standard SPR Caps
          break;
          
        case USBPD_DATA_MSG_EPR_MODE:
          // Handle EPR Mode Handshake
          {
               USBPD_EPRMode_DO_t eprdo;
               PD_memcpy((uint8_t*)&eprdo.d32, &PD_TR_buffer[2], 4);
               // Action: 0x01 Enter, 0x02 Enter Ack, 0x03 Enter Succeeded, 0x04 Enter Failed, 0x05 Exit
               uint8_t action = (eprdo.d32 >> 24) & 0xFF;
               //CDC_print("RX: EPR Mode Action "); CDC_printH(action); CDC_println("");
                              if (action == 0x02) { // Enter Ack (Source -> Sink)
                    // Sink waits for Enter Succeeded (Action 0x03) from Source
                    PD_control.EPR_Mode = 2; 
                }
                else if (action == 0x03) { // Enter Succeeded (Source -> Sink)
                    // Source confirmed EPR Entry.
                    //CDC_println("EPR: Enter Succeeded Received");
                    PD_control.EPR_Mode = 3; 
                }
                else if (action == 0x04) { // Enter Failed
                    //CDC_println("EPR: Enter Failed Received");
                    PD_control.EPR_Mode = 0;
                }
                else if (action == 0x05) { // Exit
                    //CDC_println("EPR: Exit Received");
                     PD_control.EPR_Mode = 0;
                }
           }
          break;

        case USBPD_DATA_MSG_ALERT:
          PD_memcpy((uint8_t*)&PD_control.AlertMsg.d32, &PD_TR_buffer[2], 4);
          PD_control.AlertReceived = 1;
          break;

        case USBPD_DATA_MSG_PPS_STATUS:
          PD_memcpy((uint8_t*)&PD_control.PPS_Status.d32, &PD_TR_buffer[2], 4);
          PD_control.PPS_Status_Received = 1;
          PD_control.PPS_Not_Supported = 0;
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
    } else if(status == USBPD_BMC_AUX_SOP1_HRST) {
      if(USBPD->BMC_BYTE_CNT >= 6) {
         USBPD_MessageHeader_t mh;
         mh.d16 = *(uint16_t*)PD_TR_buffer;
         PD_eMarker_Handle_Msg(mh, &PD_TR_buffer[2]);
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

// Get last Alert Data Object
uint32_t PD_getAlert(void) {
  return PD_control.AlertMsg.d32;
}

// Check if Alert received
uint8_t PD_hasAlert(void) {
  return PD_control.AlertReceived;
}

// Clear Alert flag
void PD_clearAlert(void) {
  PD_control.AlertReceived = 0;
}

void PD_getPPSStatus(void) {
  PD_control.PPS_Status_Received = 0;
  PD_control.PPS_Not_Supported = 0;
  
  USBPD_MessageHeader_t mh;
  mh.d16 = 0;
  mh.MessageHeader.MessageID = PD_control.SinkMessageID;
  mh.MessageHeader.MessageType = USBPD_CONTROL_MSG_GET_PPS_STATUS;
  mh.MessageHeader.SpecificationRevision = PD_control.PD_Version;
  
  *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
  PD_sendData(2, USBPD_TX_SOP0);
}

uint16_t PD_getPPSStatusVoltage(void) {
  if(PD_control.PPS_Not_Supported) return 0xFFFF;
  if(PD_control.PPS_Status.Struct.OutputVoltageIn20mVunits == 0xFFFF) return 0xFFFF;
  return (uint16_t)PD_control.PPS_Status.Struct.OutputVoltageIn20mVunits * 20;
}

uint16_t PD_getPPSStatusCurrent(void) {
  if(PD_control.PPS_Not_Supported) return 0xFFFF;
  if(PD_control.PPS_Status.Struct.OutputCurrentIn50mAunits == 0xFFFF) return 0xFFFF;
  return (uint16_t)PD_control.PPS_Status.Struct.OutputCurrentIn50mAunits * 50;
}

uint8_t PD_getPPSStatusFlag(void) {
    if(PD_control.PPS_Not_Supported) return 0xFF;
    if(!PD_control.PPS_Status_Received) return 0;
    return (uint8_t)PD_control.PPS_Status.Struct.OMF;
}

// Get PD Specification Revision 1,2,3
uint8_t PD_getRevision(void) {
  return PD_control.PD_Version + 1;
}
