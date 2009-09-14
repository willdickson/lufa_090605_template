#ifndef _PWM_H_
#define _PWM_H_

/* Includes: */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include "descriptors.h"
#include <LUFA/Version.h>                    // Library Version Information
#include <LUFA/Scheduler/Scheduler.h>        // Simple scheduler for task management
#include <LUFA/Drivers/USB/USB.h>            // USB Functionality
#include <LUFA/Drivers/Board/LEDs.h>         // LEDs driver

/* Macros: */
/* USB Commands */
#define USB_CMD_TEST 0
#define USB_CMD_AVR_RESET       200
#define USB_CMD_AVR_DFU_MODE    201

/* Software reset */
#define AVR_RESET() wdt_enable(WDTO_30MS); while(1) {}
#define AVR_IS_WDT_RESET()  ((MCUSR&(1<<WDRF)) ? 1:0)
#define DFU_BOOT_KEY_VAL 0xAA55AA55

typedef struct
{
    uint8_t CommandID;
    uint8_t Data;
} USBPacketOutWrapper_t;

typedef struct
{
    uint8_t CommandID;
    uint8_t Data;
} USBPacketInWrapper_t;

/* Enums: */
/** Enum for the possible status codes for passing to the UpdateStatus() function. */
enum PWM_StatusCodes_t
{
    Status_USBNotReady          = 0, /**< USB is not ready (disconnected from a USB host) */
    Status_USBEnumerating       = 1, /**< USB interface is enumerating */
    Status_USBReady             = 2, /**< USB interface is connected and ready */
    Status_ProcessingPacket     = 3, /**< Processing packet */
};

/* Global Variables: */
uint8_t count=0;
USBPacketOutWrapper_t   USBPacketOut;
USBPacketInWrapper_t    USBPacketIn;

/* Task Definitions: */
TASK(USB_ProcessPacket);

/* Function Prototypes: */
void EVENT_USB_Connect(void);
void EVENT_USB_Disconnect(void);
void EVENT_USB_ConfigurationChanged(void);
void EVENT_USB_UnhandledControlPacket(void);

void UpdateStatus(uint8_t CurrentStatus);

#if defined(INCLUDE_FROM_PWM_C)
static void USBPacket_Read(void);
static void USBPacket_Write(void);
static void IO_Init(void);
static void IO_Disconnect(void);
static void REG_16bit_Write(volatile uint16_t *reg, volatile uint16_t val);
#endif

#endif
