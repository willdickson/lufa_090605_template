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
#define USB_CMD_TEST8 0
#define USB_CMD_TEST16 1
#define USB_CMD_TEST32 2
#define USB_CMD_AVR_RESET       200
#define USB_CMD_AVR_DFU_MODE    201

/* Software reset */
#define AVR_RESET() wdt_enable(WDTO_30MS); while(1) {}
#define AVR_IS_WDT_RESET()  ((MCUSR&(1<<WDRF)) ? 1:0)
#define DFU_BOOT_KEY_VAL 0xAA55AA55

#define DATAARRAY_INT8_SIZE 60
#define DATAARRAY_UINT8_SIZE 60
#define DATAARRAY_INT16_SIZE 30
#define DATAARRAY_UINT16_SIZE 30
#define DATAARRAY_INT32_SIZE 15 
#define DATAARRAY_UINT32_SIZE 15

typedef union 
{
    int8_t int8[DATAARRAY_INT8_SIZE];
    uint8_t uint8[DATAARRAY_UINT8_SIZE];
    int16_t int16[DATAARRAY_INT16_SIZE];
    uint16_t uint16[DATAARRAY_UINT16_SIZE];
    int32_t int32[DATAARRAY_INT32_SIZE];
    uint32_t uint32[DATAARRAY_UINT32_SIZE];
} DataArray_t;

typedef struct
{
    uint8_t Len;
    DataArray_t Data;
} DataPacket_t;

typedef struct
{
    uint8_t CommandID;
    DataPacket_t DataPacket;
} USBPacketOutWrapper_t;

typedef struct
{
    uint8_t CommandID;
    DataPacket_t DataPacket;
} USBPacketInWrapper_t;

/* Enums: */
/** Enum for the possible status codes for passing to the UpdateStatus() function. */
enum USB_StatusCodes_t
{
    Status_USBNotReady          = 0, /**< USB is not ready (disconnected from a USB host) */
    Status_USBEnumerating       = 1, /**< USB interface is enumerating */
    Status_USBReady             = 2, /**< USB interface is connected and ready */
    Status_ProcessingPacket     = 3, /**< Processing packet */
};

/* Global Variables: */
uint32_t count=0;
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
