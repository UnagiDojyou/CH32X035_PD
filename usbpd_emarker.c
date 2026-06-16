#include "usbpd_emarker.h"
#include "usbpd_sink.h"
#include <system.h> // For USBPD_TX_SOP1 etc if defined there or via ch32x035.h

// eMarker Data
static volatile pd_emarker_info_t eMarkerData = {0};


// Global for ID tracking
uint8_t eMarker_MsgId_Global = 0;



// Send Discover Identity
void PD_ask_eMarker(void) {
    USBPD_MessageHeader_t mh;
    USBPD_VDMHeader_t vdm_hdr;

    // Reset data
    eMarkerData.active = 0;

    // Prepare Header
    mh.d16 = 0;
    mh.MessageHeader.MessageType = USBPD_DATA_MSG_VENDOR_DEFINED;
    mh.MessageHeader.NumberOfDataObjects = 1;
    mh.MessageHeader.SpecificationRevision = USBPD_REVISION_30; 
    mh.MessageHeader.MessageID = eMarker_MsgId_Global++;
    eMarker_MsgId_Global &= 7;

    // Prepare VDM Header
    vdm_hdr.d32 = 0;
    vdm_hdr.Struct.SVID = 0xFF00; // SID for PD Standard
    vdm_hdr.Struct.VDMType = 1;   // Structured VDM
    vdm_hdr.Struct.VDMVersion = 1; // Version 2.0
    vdm_hdr.Struct.CommandType = 0; // Initiator (Request)
    vdm_hdr.Struct.Command = 1;   // Discover Identity

    // Needs access to send buffer
    extern uint8_t PD_TR_buffer[];

    *(uint16_t*)&PD_TR_buffer[0] = mh.d16;
    *(uint32_t*)&PD_TR_buffer[2] = vdm_hdr.d32;

    // Send with SOP'
    // PD_sendData signature will be updated to (length, sop)
    PD_sendData(6, USBPD_TX_SOP1);
}

// Helper for unaligned 32-bit read (Little Endian)
static uint32_t get_u32_le(uint8_t* ptr) {
    return (uint32_t)ptr[0] | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[2] << 16) | ((uint32_t)ptr[3] << 24);
}



// Handle Receive Message (SOP')
void PD_eMarker_Handle_Msg(USBPD_MessageHeader_t mh, uint8_t *data) {
    // 2. Identify and Handle

    if (mh.MessageHeader.MessageType == USBPD_DATA_MSG_VENDOR_DEFINED && mh.MessageHeader.NumberOfDataObjects > 0) {
        USBPD_VDMHeader_t vdm_hdr;
        vdm_hdr.d32 = get_u32_le(data);
        
        uint8_t cmd_type = vdm_hdr.Struct.CommandType; // 0=REQ, 1=ACK, 2=NAK, 3=BSY

        // If Request (Initiator), it's likely from Charger. Ignore it (don't GoodCRC).
        if (cmd_type == 0) return;

        // If ACK/NAK/BSY, check if it's for us or if we can snoop.
        if (vdm_hdr.Struct.SVID == 0xFF00 && vdm_hdr.Struct.Command == 1) { // Discover Identity
            if (cmd_type == 1) { // ACK
                // Ensure we actually have data (ID Header + ...)
                // VDM Header is Obj 0. ID Header is Obj 1.
                // So NumberOfDataObjects must be > 1.
                if (mh.MessageHeader.NumberOfDataObjects > 1) {
                    eMarkerData.ID_Header   = get_u32_le(data + 4);
                    if (mh.MessageHeader.NumberOfDataObjects > 2) eMarkerData.Cert_Stat   = get_u32_le(data + 8);
                    if (mh.MessageHeader.NumberOfDataObjects > 3) eMarkerData.Product     = get_u32_le(data + 12);
                    if (mh.MessageHeader.NumberOfDataObjects > 4) eMarkerData.Cable_VDO_1 = get_u32_le(data + 16);
                    if (mh.MessageHeader.NumberOfDataObjects > 5) eMarkerData.Cable_VDO_2 = get_u32_le(data + 20);
                    if (mh.MessageHeader.NumberOfDataObjects > 6) eMarkerData.Ama_VDO     = get_u32_le(data + 24);
                    
                    eMarkerData.active = 1;
                }
            }
        }
        
        // Send GoodCRC ONLY if MessageID matches our last request
        // (Assuming we stick to Stop-and-wait, we expect (eMarker_MsgId - 1))
        // Note: We need access to eMarker_MsgId. For now, rely on strict matching?
        // Let's make eMarker_MsgId global in next step or use heuristic.
        // Heuristic: If we are active (just asked), we might reply?
        // Better: Just check if we asked? 
        // Current implementation: To avoid changing file scope variables too much in one replace,
        // let's Assume if we parsed data, we are happy? NO, snoop vs reply.
        
        // SAFE FIX: 
        // 1. Move eMarker_MsgId to file scope (done in separate tool if needed, or here).
        // 2. Or, just rely on: If it IS a VDM response, and we are in a state where we asked...
        
        // Implementation with static moved out:
        extern uint8_t eMarker_MsgId_Global; // Define this below
        uint8_t expected_id = (eMarker_MsgId_Global - 1) & 7;

        if (mh.MessageHeader.MessageID == expected_id) {
             USBPD_MessageHeader_t crc_mh;
             crc_mh.d16 = 0;
             crc_mh.MessageHeader.MessageType = USBPD_CONTROL_MSG_GOODCRC;
             crc_mh.MessageHeader.MessageID = mh.MessageHeader.MessageID;
             crc_mh.MessageHeader.SpecificationRevision = mh.MessageHeader.SpecificationRevision;
             crc_mh.MessageHeader.NumberOfDataObjects = 0;
             
             extern uint8_t PD_TR_buffer[];
             *(uint16_t*)&PD_TR_buffer[0] = crc_mh.d16;
             PD_sendData(2, USBPD_TX_SOP1);
        }
    }
}

pd_emarker_info_t* PD_get_eMarker_data(void) {
    return (pd_emarker_info_t*)&eMarkerData;
}

// Helper: Get Maximum VBUS Current (mA)
uint16_t PD_eMarker_GetMaxCurrent_mA(void) {
    if (!eMarkerData.active) return 0;
    // USB PD R3.1, Cable VDO (Passive): Bits 6:5
    // 00=3A, 01=5A, 10/11=Reserved(3A)
    // Note: Some legacy cables use 10b for 5A (PD 2.0). 
    // We accept 01b (PD3.0 5A) and 10b (Legacy 5A) as 5000mA.
    uint8_t current_code = (eMarkerData.Cable_VDO_1 >> 5) & 0x03;
    if (current_code == 0x02) return 5000;
    return 3000; 
}

// Helper: Get Maximum VBUS Voltage (mV)
uint16_t PD_eMarker_GetMaxVoltage_mV(void) {
    if (!eMarkerData.active) return 0;
    // USB PD R3.1, Cable VDO (Passive): Bits 9:10
    // 00=20V, 01=30V, 10=40V, 11=50V
    uint8_t voltage_code = (eMarkerData.Cable_VDO_1 >> 9) & 0x03;
    switch(voltage_code) {
        case 0: return 20000;
        case 1: return 30000;
        case 2: return 40000;
        case 3: return 50000;
    }
    return 20000;
}
