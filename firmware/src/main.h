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
#define USB_CMD_TEST_SET 3
#define USB_CMD_TEST_GET 4
#define USB_CMD_STRUCT_SET 5
#define USB_CMD_STRUCT_GET 6
#define USB_CMD_FLOAT_SET 7
#define USB_CMD_FLOAT_GET 8
#define USB_CMD_AVR_RESET       200
#define USB_CMD_AVR_DFU_MODE    201

/* Software reset */
#define AVR_RESET() wdt_enable(WDTO_30MS); while(1) {}
#define AVR_IS_WDT_RESET()  ((MCUSR&(1<<WDRF)) ? 1:0)
#define DFU_BOOT_KEY_VAL 0xAA55AA55

//#define DATAARRAY_MAX_LEN 60 
#define PASS 0
#define FAIL 1

enum USB_StatusCodes_t
{
    Status_USBNotReady          = 0, /**< USB is not ready (disconnected from a USB host) */
    Status_USBEnumerating       = 1, /**< USB interface is enumerating */
    Status_USBReady             = 2, /**< USB interface is connected and ready */
    Status_ProcessingPacket     = 3, /**< Processing packet */
};

typedef struct {
    char Buf[IN_EPSIZE];
} InPacket_t;

typedef struct {
    char Buf[OUT_EPSIZE];
} OutPacket_t;

typedef struct {
    InPacket_t Packet;
    uint8_t Pos;
} USBIn_t;

typedef struct {
    OutPacket_t Packet;
    uint8_t Pos;
} USBOut_t;

// Sytem state structure
typedef struct {
    uint8_t Val8;
    uint16_t Val16;
    uint32_t Val32; 
} SysState_t;


/* Global Variables: */
uint32_t count=0;
USBIn_t USBIn;
USBOut_t USBOut;
SysState_t SysState = {Val8:0,Val16:0,Val32:0};
float test_float;

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
static uint8_t USBIn_SetData(void *data, size_t len); 
static uint8_t USBOut_GetData(void *data, size_t len);
static void USBIn_ResetData(void);
static void USBOut_ResetData(void);
#endif

#endif
