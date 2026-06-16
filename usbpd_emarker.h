#ifndef __USBPD_EMARKER_H
#define __USBPD_EMARKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include <system.h>
#include <usbpd.h>

// VDM Header
typedef union {
  uint32_t d32;
  struct {
    uint32_t Command:5;
    uint32_t Reserved:1;
    uint32_t CommandType:2; // 0=Req, 1=ACK, 2=NAK, 3=BSY
    uint32_t ObjectPosition:3;
    uint32_t Reserved_11_12:2;
    uint32_t VDMVersion:2; // 0=1.0, 1=2.0
    uint32_t VDMType:1; // 1=Structured, 0=Unstructured
    uint32_t SVID:16;
  } Struct;
} USBPD_VDMHeader_t;

// eMarker Information Structure
typedef struct {
  uint32_t ID_Header;
  uint32_t Cert_Stat;
  uint32_t Product;
  uint32_t Cable_VDO_1;
  uint32_t Cable_VDO_2;
  uint32_t Ama_VDO;
  uint8_t  active; // 1 if data is valid
} pd_emarker_info_t;

// Functions
void PD_ask_eMarker(void);
void PD_eMarker_Handle_Msg(USBPD_MessageHeader_t mh, uint8_t *data);
pd_emarker_info_t* PD_get_eMarker_data(void);

// Helper Parsing Functions
uint16_t PD_eMarker_GetMaxCurrent_mA(void); // Returns 3000 or 5000
uint16_t PD_eMarker_GetMaxVoltage_mV(void); // Returns 20000, 30000...
#ifdef __cplusplus
}
#endif

#endif // __USBPD_EMARKER_H
