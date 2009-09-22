#define INCLUDE_FROM_PWM_C
#include "main.h"

/* Scheduler Task List */
TASK_LIST
{
    { .Task = USB_USBTask          , .TaskStatus = TASK_STOP },
    { .Task = USB_ProcessPacket    , .TaskStatus = TASK_STOP },
};

/* DFU Bootloader Declarations */
uint32_t  boot_key __attribute__ ((section (".noinit")));
typedef void (*AppPtr_t)(void) __attribute__ ((noreturn));
AppPtr_t Bootloader = (AppPtr_t)0xf000;

/* Main program entry point. This routine configures the hardware required 
 * by the application, then starts the scheduler to run the USB management 
 * task.
 */
int main(void)
{
    /* After reset start bootloader? */
    if ((AVR_IS_WDT_RESET()) && (boot_key == DFU_BOOT_KEY_VAL)) {
        boot_key = 0;
        Bootloader();
    }

    /* Disable watchdog if enabled by bootloader/fuses */
    MCUSR &= ~(1 << WDRF);
    wdt_disable();

    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    /* Hardware Initialization */
    LEDs_Init();

    /* Indicate USB not ready */
    UpdateStatus(Status_USBNotReady);

    /* Initialize Scheduler so that it can be used */
    Scheduler_Init();

    /* Initialize USB Subsystem */
    USB_Init();

    /* Initialize I/O lines */
    IO_Init();

    /* Scheduling - routine never returns, so put this last in the main function */
    Scheduler_Start();

}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Connect(void)
{
    /* Start USB management task */
    Scheduler_SetTaskMode(USB_USBTask, TASK_RUN);

    /* Indicate USB enumerating */
    UpdateStatus(Status_USBEnumerating);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management task.
 */
void EVENT_USB_Disconnect(void)
{
    /* Stop running HID reporting and USB management tasks */
    Scheduler_SetTaskMode(USB_ProcessPacket, TASK_STOP);
    Scheduler_SetTaskMode(USB_USBTask, TASK_STOP);

    /* Indicate USB not ready */
    UpdateStatus(Status_USBNotReady);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration, and configures the generic HID device endpoints.
 */
void EVENT_USB_ConfigurationChanged(void)
{
    /* Setup USB In and Out Endpoints */
    Endpoint_ConfigureEndpoint(OUT_EPNUM, EP_TYPE_BULK,
            ENDPOINT_DIR_OUT, OUT_EPSIZE,
            ENDPOINT_BANK_SINGLE); 

    Endpoint_ConfigureEndpoint(IN_EPNUM, EP_TYPE_BULK,
            ENDPOINT_DIR_IN, IN_EPSIZE,
            ENDPOINT_BANK_SINGLE);

    /* Indicate USB connected and ready */
    UpdateStatus(Status_USBReady);

    /* Start ProcessPacket task */
    Scheduler_SetTaskMode(USB_ProcessPacket, TASK_RUN);
}

/** Function to manage status updates to the user. This is done via LEDs on the given board, if available, but may be changed to
 *  log to a serial port, or anything else that is suitable for status updates.
 */
void UpdateStatus(uint8_t CurrentStatus)
{
    uint8_t LEDMask = LEDS_NO_LEDS;

    /* Set the LED mask to the appropriate LED mask based on the given status code */
    switch (CurrentStatus) {

        case Status_USBNotReady:
            LEDMask = (LEDS_LED1);
            break;
        case Status_USBEnumerating:
            LEDMask = (LEDS_LED1 | LEDS_LED2);
            break;
        case Status_USBReady:
            LEDMask = (LEDS_LED2 | LEDS_LED4);
            break;
        case Status_ProcessingPacket:
            LEDMask = (LEDS_LED1 | LEDS_LED2);
            break;
    }

    /* Set the board LEDs to the new LED mask */
    LEDs_SetAllLEDs(LEDMask);
}

TASK(USB_ProcessPacket)
{
    uint8_t commandID;

    /* Check if the USB System is connected to a Host */
    if (USB_IsConnected) {

        /* Select the Data Out Endpoint */
        Endpoint_SelectEndpoint(OUT_EPNUM);

        /* Check if OUT Endpoint contains a packet */
        if (Endpoint_IsOUTReceived()) {

            /* Check to see if a command from the host has been issued */
            if (Endpoint_IsReadWriteAllowed()) {

                /* Indicate busy */
                UpdateStatus(Status_ProcessingPacket);

                /* Read USB packet from the host */
                USBPacket_Read();

                /* Reset buffer for reading and writing */
                USBOut_ResetData();
                USBIn_ResetData();

                /* Get command ID from bulkout buffer */
                USBOut_GetData(&commandID,sizeof(uint8_t));

                /* Return same command ID in bulkin buffer */
                USBIn_SetData(&commandID,sizeof(uint8_t));

                /* Process USB packet */
                switch (commandID) {

                    case USB_CMD_TEST8:
                        count += 1;
                        for (int j=0; j< 60; j++) {
                            uint8_t val = (uint8_t)(count+j);
                            USBIn_SetData(&val,sizeof(uint8_t));
                        }
                        break;

                    case USB_CMD_TEST16:
                        count += 1;
                        for (int j=0; j< 30; j++) {
                            uint16_t val = (uint16_t)(count+j);
                            USBIn_SetData(&val,sizeof(uint16_t));
                        }
                        break;

                    case USB_CMD_TEST32:
                        count += 1;
                        for (int j=0; j< 15; j++) {
                            uint32_t val = (uint32_t)(count+j);
                            USBIn_SetData(&val,sizeof(uint32_t));
                        }
                        break;

                    case USB_CMD_TEST_SET:
                        USBOut_GetData(&SysState.Val8,sizeof(uint8_t));
                        USBOut_GetData(&SysState.Val16,sizeof(uint16_t));
                        USBOut_GetData(&SysState.Val32,sizeof(uint32_t));
                        break;

                    case USB_CMD_TEST_GET:
                        USBIn_SetData(&SysState.Val8,sizeof(uint8_t));
                        USBIn_SetData(&SysState.Val16,sizeof(uint16_t));
                        USBIn_SetData(&SysState.Val32,sizeof(uint32_t));
                        break;

                    case USB_CMD_STRUCT_SET:
                        USBOut_GetData(&SysState, sizeof(SysState_t));
                        break;

                    case USB_CMD_STRUCT_GET:
                        USBIn_SetData(&SysState, sizeof(SysState_t));
                        break;

                    case USB_CMD_FLOAT_SET:
                        USBOut_GetData(&test_float, sizeof(float));
                        break;

                    case USB_CMD_FLOAT_GET:
                        USBIn_SetData(&test_float, sizeof(float));
                        break;
                        
                    case USB_CMD_AVR_RESET:
                        USBPacket_Write();
                        AVR_RESET();
                        break;

                    case USB_CMD_AVR_DFU_MODE:
                        USBPacket_Write();
                        boot_key = DFU_BOOT_KEY_VAL;
                        AVR_RESET();
                        break;

                    default:
                        break;
                }

                /* Write the return USB packet */
                USBPacket_Write();

                /* Indicate ready */
                LEDs_SetAllLEDs(LEDS_LED2 | LEDS_LED4);
            }
        }
    }
}

static void USBIn_ResetData(void)
{
   USBIn.Pos = 0;
    return;
}

static void USBOut_ResetData(void)
{
    USBOut.Pos = 0;
    return;
}

static uint8_t USBIn_SetData(void *data, size_t len) 
{
    uint8_t rval = FAIL;
    if (USBIn.Pos + len <= IN_EPSIZE) { 
        memcpy((void *)(USBIn.Packet.Buf + USBIn.Pos), data, len);
        USBIn.Pos += len;
        rval = PASS;
    }
    return rval;
}

static uint8_t USBOut_GetData(void *data, size_t len)
{
    uint8_t rval = FAIL;
    if (USBOut.Pos + len <= OUT_EPSIZE) {
        memcpy(data, (void *)(USBOut.Packet.Buf + USBOut.Pos), len);
        USBOut.Pos += len;
        rval = PASS;
    }
    return rval;
}

static void USBPacket_Read(void)
{
    uint8_t* USBPacketOutPtr = (uint8_t*)&USBOut.Packet;

    /* Select the Data Out endpoint */
    Endpoint_SelectEndpoint(OUT_EPNUM);

    /* Read in USB packet header */
    Endpoint_Read_Stream_LE(USBPacketOutPtr, sizeof(USBOut.Packet));

    /* Finalize the stream transfer to send the last packet */
    Endpoint_ClearOUT();
}

static void USBPacket_Write(void)
{
    uint8_t* USBPacketInPtr = (uint8_t*)&USBIn.Packet;

    /* Select the Data In endpoint */
    Endpoint_SelectEndpoint(IN_EPNUM);

    /* Wait until read/write to IN data endpoint allowed */
    while (!(Endpoint_IsReadWriteAllowed() && Endpoint_IsINReady()));

    /* Write the return data to the endpoint */
    Endpoint_Write_Stream_LE(USBPacketInPtr, sizeof(USBIn.Packet));

    /* Finalize the stream transfer to send the last packet */
    Endpoint_ClearIN();
}

static void IO_Init(void)
{
}

static void IO_Disconnect(void)
{
}

static void REG_16bit_Write(volatile uint16_t *reg, volatile uint16_t val)
{
    /* See "Accessing 16-bit Registers" of the AT90USB1287 datasheet */
    uint8_t sreg;
    /* Save global interrupt flag */
    sreg = SREG;
    /* Disable interrupts */
    cli();
    *reg = val;
    /* Restore global interrupt flag */
    SREG = sreg;
}
