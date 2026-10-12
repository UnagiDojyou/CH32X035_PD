// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================
#include <config.h>                       // user configurations
#include <gpio.h>                         // GPIO functions
#include "usbpd_sink.h"                   // USB PD sink functions
#include <usb_cdc.h>

#define TARGET_VOLTAGE    6000            // define target voltage in millivolts
#define TARGET_CURRENT    2000            // define target current in milliampere
#define PIN_ONOFF PA5

static void CDC_reset_bootloader(void) {
  DLY_ms(100);
  RST_bootloader();
}

// Match a complete line across CDC packets. Reject extra or mismatched characters.
static void CDC_poll_commands(void) {
  static const char command[] = "bootloader";
  static uint8_t position[2] = {0, 0};
  static uint8_t invalid[2] = {0, 0};
  for(uint8_t port = 0; port < 2; port++) {
    // Bound each visit so a continuous stream cannot starve PD or the other CDC.
    for(uint8_t count = 0; count < 64; count++) {
      char c;
      if(port) {
        // CDC2 is a lossless echo port. Leave OUT pending while its TX queue is full.
        if(!CDC2_ready() || !CDC2_tryRead(&c)) break;
        CDC2_tryWrite(c);
      } else if(!CDC_tryRead(&c)) break;
      if(c == '\r' || c == '\n') {
        uint8_t boot = !invalid[port] && position[port] == sizeof(command) - 1;
        position[port] = 0;
        invalid[port] = 0;
        if(boot) CDC_reset_bootloader();
      } else if(!invalid[port]) {
        if(position[port] < sizeof(command) - 1 && c == command[position[port]]) position[port]++;
        else invalid[port] = 1;
      }
    }
  }
  CDC2_flush();
}

// Keep both CDC ports responsive while the PD library waits for a contract.
void PD_wait_callback(void) {
  CDC_poll_commands();
}

static void CDC_delay(uint16_t milliseconds) {
  while(milliseconds--) {
    CDC_poll_commands();
    DLY_ms(1);
  }
}

// Keep the bootloader command available when a PD operation fails.
static void CDC_wait_commands(void) __attribute__((noreturn));
static void CDC_wait_commands(void) {
  while (1) {
    CDC_poll_commands();
    DLY_ms(1);
  }
}

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
    CDC_poll_commands();
    count++;
    if (count > 10) {
      CDC_print(".");
      count = 0;
    }
    CDC_delay(100);
  }
  CDC_print("PD Version");
  CDC_printD(PD_getRevision());
  CDC_println("");

  if (PD_setPDO(1, 5000)) {
    CDC_println("PDO Set 5V");
    CDC_delay(1000);
  } else {
    CDC_println("Fail to set 5V");
    CDC_wait_commands();
  }

  if (PD_getEPRCapable()) {
    if (PD_setEPRMode(1)) {
      CDC_println("EPR Mode Set");
      print_PDO_list();
      for (uint16_t i = 0; i < 1000; i++) {
        CDC_poll_commands();
        PD_Loop();
        DLY_ms(1);
      }
      if (PD_setEPRMode(0)) {
        CDC_println("EPR Mode exit");
      }else {
        CDC_println("Fail to exit EPR Mode!");
        CDC_wait_commands();
      }
    } else {
      CDC_println("Fail to set EPR Mode!");
      CDC_wait_commands();
    }
  } else {
    CDC_println("EPR Mode is not supported");
    print_PDO_list();
  }

  DLY_ms(5);
  if (PD_setVoltage(TARGET_VOLTAGE)) {
    CDC_printD(TARGET_VOLTAGE);
    CDC_println(" mV Set");
  } else {
    CDC_printD(TARGET_VOLTAGE);
    CDC_println(" mV Set Failed");
    CDC_wait_commands();
  }

  while (1) {
    CDC_poll_commands();
    if (PD_Loop()) {
      PD_setMismatch(0);
      if (PD_setVoltage(TARGET_VOLTAGE)) {
        CDC_printD(TARGET_VOLTAGE);
        CDC_println(" mV Set");
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
