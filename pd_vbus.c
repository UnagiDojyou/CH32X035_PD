// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================
#include <config.h>                       // user configurations
#include <gpio.h>                         // GPIO functions
#include "usbpd_sink.h"                   // USB PD sink functions
#include <usb_cdc.h>

#define TARGET_VOLTAGE    7500            // define target voltage in millivolts
#define TARGET_CURRENT    2000            // define target current in milliampere
#define PIN_ONOFF PA5

void print_PDO_list() {
  CDC_println("PDO List:");
  for (uint8_t i = 1; i <= PD_getPDONum(); i++) {
    switch (PD_getPDOType(i)) {
      case PDO_TYPE_FIXED:
        CDC_printD(i);
        CDC_print(" ");
        CDC_print("Fixed ");
        CDC_printD(PD_getPDOMaxVoltage(i));
        CDC_print("mV ");
        CDC_printD(PD_getPDOMaxCurrent(i));
        CDC_println("mA");
        break;
      case PDO_TYPE_PPS:
        CDC_printD(i);
        CDC_print(" ");
        CDC_print("PPS ");
        CDC_printD(PD_getPDOMinVoltage(i));
        CDC_print("-");
        CDC_printD(PD_getPDOMaxVoltage(i));
        CDC_print("mV ");
        CDC_printD(PD_getPDOMaxCurrent(i));
        CDC_print("mA");
        if (PD_getPPSPowerLimited(i)) {
          CDC_println(" Limited");
        } else {
          CDC_println("");
        }
        break;
      case PDO_TYPE_SPR_AVS:
        CDC_printD(i);
        CDC_print(" ");
        CDC_print("SPR AVS ");
        CDC_printD(PD_getPDOMinVoltage(i));
        CDC_print("-");
        CDC_printD(PD_getPDOMaxVoltage(i));
        CDC_print("mV ");
        CDC_printD(PD_getPDOMaxCurrentWithVoltage(i, 9000));
        CDC_print("mA@9-15V ");
        CDC_printD(PD_getPDOMaxCurrentWithVoltage(i, 20000));
        CDC_println("mA@15-20V");
        break;
      case PDO_TYPE_EPR_AVS:
        CDC_printD(i);
        CDC_print(" ");
        CDC_print("EPR AVS ");
        CDC_printD(PD_getPDOMinVoltage(i));
        CDC_print("-");
        CDC_printD(PD_getPDOMaxVoltage(i));
        CDC_print("mV ");
        CDC_printD(PD_getPDOPower(i));
        CDC_println("W");
        break;
      default:
        CDC_printD(i);
        CDC_println(" Unknown PDO Type");
        break;
    }
  }
}

// ===================================================================================
// Main Function
// ===================================================================================
int main(void) {
  PIN_output(PIN_ONOFF);
  PIN_low(PIN_ONOFF);
  uint8_t count = 0;
  CDC_init();
  while (!PD_connect()) {
    count++;
    if (count > 10) {
      CDC_print(".");
      count = 0;
    }
    DLY_ms(100);
  }
  CDC_print("PD Version");
  CDC_printD(PD_getRevision());
  CDC_println("");

  if (PD_setPDO(1, 5000)) {
    CDC_println("PDO Set 5V");
    DLY_ms(1000);
  } else {
    CDC_println("Fail to set 5V");
    while(1);
  }

  if (PD_getEPRCapable()) {
    if (PD_setEPRMode(1)) {
      CDC_println("EPR Mode Set");
      print_PDO_list();
    } else {
      CDC_println("Fail to set EPR Mode!");
      while(1);
    }
  } else {
    CDC_println("EPR Mode is not supported");
    print_PDO_list();
  }

  DLY_ms(5);
  if (PD_setVoltage(TARGET_VOLTAGE)) {
    CDC_println("TARGET_VOLTAGE V Set");
  } else {
    CDC_println("Fail to set TARGET_VOLTAGE V");
    while(1);
  }

  while (1) {
    if (PD_Loop()) {
      PD_setMismatch(0);
      if (PD_setVoltage(TARGET_VOLTAGE)) {
        CDC_println("TARGET_VOLTAGE V Set");
      } else {
        DLY_ms(5);
        PD_setMismatch(1);
        PD_setVoltage(5000);
      }
      print_PDO_list();
    }
    DLY_ms(1);
  }
}