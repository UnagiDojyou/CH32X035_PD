// ===================================================================================
// User configurations
// ===================================================================================

#pragma once

// Pin definitions
#define PIN_SCL             PA5       // I2C SCL connected to OLED
#define PIN_SDA             PA6       // I2C SDA connected to OLED
#define PIN_LED             PB1       // pin connected to LED (active low)

// MCU supply voltage
#define USB_VDD             1         // 0: 3.3V, 1: 5V

// USB device descriptor
#define USB_VENDOR_ID       0x16C0    // VID (shared www.voti.nl)
#define USB_PRODUCT_ID      0x27DD    // PID (shared CDC-ACM)
#define USB_DEVICE_VERSION  0x0200    // v2.0 (BCD-format)
#define USB_LANGUAGE        0x0409    // US English

// USB configuration descriptor
#define USB_MAX_POWER_mA    50        // max power in mA 

// USB descriptor strings
#define MANUF_STR           "wagiminator"
#define PROD_STR            "Dual USB-to-Serial Converter"
#define SERIAL_STR          "CH32X035CDC"
#define INTERF_STR          "CDC-Serial"
#define INTERF2_STR         "CDC-Serial 2"

// Application TX ring sizes (one byte per ring is reserved for the full check).
#define CDC_TX_BUFFER_SIZE  256    // CDC1: 255 usable bytes
#define CDC2_TX_BUFFER_SIZE 266    // CDC2: 265 usable bytes
