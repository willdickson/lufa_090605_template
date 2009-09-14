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
    Endpoint_ConfigureEndpoint(IN_EPNUM, EP_TYPE_BULK,
            ENDPOINT_DIR_IN, IN_EPSIZE,
            ENDPOINT_BANK_SINGLE);

    Endpoint_ConfigureEndpoint(OUT_EPNUM, EP_TYPE_BULK,
            ENDPOINT_DIR_OUT, OUT_EPSIZE,
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

                /* Return the same CommandID that was received */
                USBPacketIn.CommandID = USBPacketOut.CommandID;

                /* Process USB packet */
                switch (USBPacketOut.CommandID) {

                    case USB_CMD_TEST:
                        count += 1;
                        USBPacketIn.Data = count;
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
static void USBPacket_Read(void)
{
    uint8_t* USBPacketOutPtr = (uint8_t*)&USBPacketOut;

    /* Select the Data Out endpoint */
    Endpoint_SelectEndpoint(OUT_EPNUM);

    /* Read in USB packet header */
    Endpoint_Read_Stream_LE(USBPacketOutPtr, sizeof(USBPacketOut));

    /* Finalize the stream transfer to send the last packet */
    Endpoint_ClearOUT();
}

static void USBPacket_Write(void)
{
    uint8_t* USBPacketInPtr = (uint8_t*)&USBPacketIn;

    /* Select the Data In endpoint */
    Endpoint_SelectEndpoint(IN_EPNUM);

    /* Wait until read/write to IN data endpoint allowed */
    while (!(Endpoint_IsReadWriteAllowed() && Endpoint_IsINReady()));

    /* Write the return data to the endpoint */
    Endpoint_Write_Stream_LE(USBPacketInPtr, sizeof(USBPacketIn));

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
